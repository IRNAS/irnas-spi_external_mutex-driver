# SPI External Mutex

This driver provides a mechanism for 2 SPI Masters (MCUs) to use the same bus to communicate with spi peripherals (e.g. external flash).

It uses a single GPIO line to signal the mutex/lock state.

## Instalation

This driver was written and tested for nrf-sdk v1.5.0

To install, first modify `.../ncs/nrf/west.yml` and add the following sections:

1. In `remotes`, add the following if not already added:

```yaml
 - name: irnas
   url-base: https://github.com/irnas
```

2. In the `projects` section add at the bottom:

```yaml
- name: irnas-spi_external_mutex-driver
  repo-path: irnas-spi_external_mutex-driver
  path: irnas/irnas-spi_external_mutex-driver
  remote: irnas
  revision: v1.0.0
```

Then run west update in your freshly created bash/command prompt session.

Above command will clone `irnas-spi_external_mutex-driver` repository inside of `ncs/irnas/`. You can now use the driver in your application projects.

## Usage

To enable this driver, device tree, KConfig and peripheral driver modifications are reqired.

### Device tree

In your boards DTS file, modify the desired SPI peripheral and add the following fields:

- `compatible = "nordic,nrf-spim-ext-mutex";` - This must replace the default compatible value.
- `signal-gpios = <...>` - Gpio, pin, and flags for the singal pin. Recommended flags are `(GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH)` for both sides.
- `mutex-role = "..."` - the mutex role. Must be set to `"master"` on one MCU and to `"slave"` on the other one.

The MCU that uses spi more often should be `"master"`, since `"slave"` performs an additional check of the `sclk` line
and is therefore a bit slower to acquire the lock.

See [nordic,nrf-spim-ext-mutex.yaml](../../../dts/bindings/nordic,nrf-spim-ext-mutex.yaml) for details.

An example is provided here:

``` dts
&spi3 {
    status = "okay";

    compatible = "nordic,nrf-spim-ext-mutex";
    signal-gpios = <&gpio0 1 (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH)>;
    mutex-role = "master";

    sck-pin = <28>;
    mosi-pin = <29>;
    miso-pin = <27>;

    cs-gpios = <&gpio0 2 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;

    flash42: en25qh32b@0 {
        // ...
    };

};
```

### Kconfig / prj.conf

In your projects `prj.conf`, you must set `CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT` to the number of chip select pins in use.

For the DTS example above, `CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT` has to be set to `1`.

An optional `CONFIG_NRFX_SPIM_EXT_MUTEX_ACQUIRE_TIMEOUT_MS` can also be set to a non-default value (default is 2000), if you expect the mutex to be held for longer.

### Usage in a peripheral driver

The SPI external lock must be acquired before a set of spi transactions that must remain uninterrupted by the other MCU with writes to the same peripheral.
Locally, for thread safety, this is already done with calls to `acquire` and `release`.

Thus, it is recommended to call `spi_ext_mutex_acquire` before  `acquire` and `spi_ext_mutex_release` after `release`.
If `spi_ext_mutex_acquire` returns an error, `spi_ext_mutex_release` must be called. The driver can then retry to acquire the lock or exit with an error.

See the en25 flash driver as an example.

## Notes

In order to get a custom spi driver to compile and replace an existing zephyr spi driver, at least one of the "in-tree" drivers must be selected. See the spi cmakefile in `ncs/zephyr/drivers/spi/CMakeLists.txt` for details.

We thus select the original SPIM driver, which also includes all the corrent HAL files from nordic.

For this reason you might get the following compiler warning for unused functions:

``` txt
/home/tjaz/NCS/ncs_v1.5.0/zephyr/drivers/spi/spi_nrfx_spim.c:302:12: warning: 'init_spim' defined but not used [-Wunused-function]
  302 | static int init_spim(const struct device *dev)
      |            ^~~~~~~~~
/home/tjaz/NCS/ncs_v1.5.0/zephyr/drivers/spi/spi_nrfx_spim.c:281:36: warning: 'spi_nrfx_driver_api' defined but not used [-Wunused-const-variable=]
  281 | static const struct spi_driver_api spi_nrfx_driver_api = {
      |                                    ^~~~~~~~~~~~~~~~~~~

```

It is safe to ignore them.

## Notes about boot procedures

The spi external lock might not work as expected if one of the MCUs is booting/has been reset while the other attempts to use it.

It is recommended to check the return of `device_get_binding` for all spi peripherals after boot and reset a couple of times if one of them is `NULL`.

Also, do not immediately start spi transactions after reseting the other MCU (e.g. via a reset pin).

