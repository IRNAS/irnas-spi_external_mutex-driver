/*
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/spi.h>
#include <nrfx_spim.h>
#include <hal/nrf_clock.h>
#include <string.h>

#define LOG_DOMAIN "spi_nrfx_spim_ext_mutex"
#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_nrfx_spim_ext_mutex);

#include "spi_context.h"

enum mutex_role
{
	MUTEX_ROLE_MASTER = 0,
	MUTEX_ROLE_SLAVE = 1,
};
struct spi_nrfx_data
{
	struct spi_context ctx;
	const struct device *dev;
	size_t chunk_len;
	bool busy;
	bool initialized;
#if (CONFIG_SPI_NRFX_RAM_BUFFER_SIZE > 0)
	uint8_t buffer[CONFIG_SPI_NRFX_RAM_BUFFER_SIZE];
#endif

	// new:
	const struct device *signal_pin_dev;

	const struct device *cs_pin_devs[CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT];
	int cs_pin_values[CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT];
};

struct spi_nrfx_config
{
	nrfx_spim_t spim;
	size_t max_chunk_len;
	uint32_t max_freq;
	nrfx_spim_config_t def_config;

	// new:
	char *signal_pin_dev_label;
	int signal_pin_num;
	int signal_pin_flags;

	enum mutex_role mutex_role;

	char *cs_pin_dev_labels[CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT];
	int cs_pin_nums[CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT];
	int cs_pin_flags[CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT];
};

static void event_handler(const nrfx_spim_evt_t *p_event, void *p_context);

static inline struct spi_nrfx_data *get_dev_data(const struct device *dev)
{
	return dev->data;
}

static inline const struct spi_nrfx_config *get_dev_config(const struct device *dev)
{
	return dev->config;
}

static inline nrf_spim_frequency_t get_nrf_spim_frequency(uint32_t frequency)
{
	/* Get the highest supported frequency not exceeding the requested one.
	 */
	if (frequency < 250000)
	{
		return NRF_SPIM_FREQ_125K;
	}
	else if (frequency < 500000)
	{
		return NRF_SPIM_FREQ_250K;
	}
	else if (frequency < 1000000)
	{
		return NRF_SPIM_FREQ_500K;
	}
	else if (frequency < 2000000)
	{
		return NRF_SPIM_FREQ_1M;
	}
	else if (frequency < 4000000)
	{
		return NRF_SPIM_FREQ_2M;
	}
	else if (frequency < 8000000)
	{
		return NRF_SPIM_FREQ_4M;
/* Only the devices with HS-SPI can use SPI clock higher than 8 MHz and
 * have SPIM_FREQUENCY_FREQUENCY_M32 defined in their own bitfields.h
 */
#if defined(SPIM_FREQUENCY_FREQUENCY_M32)
	}
	else if (frequency < 16000000)
	{
		return NRF_SPIM_FREQ_8M;
	}
	else if (frequency < 32000000)
	{
		return NRF_SPIM_FREQ_16M;
	}
	else
	{
		return NRF_SPIM_FREQ_32M;
#else
	}
	else
	{
		return NRF_SPIM_FREQ_8M;
#endif
	}
}

static inline nrf_spim_mode_t get_nrf_spim_mode(uint16_t operation)
{
	if (SPI_MODE_GET(operation) & SPI_MODE_CPOL)
	{
		if (SPI_MODE_GET(operation) & SPI_MODE_CPHA)
		{
			return NRF_SPIM_MODE_3;
		}
		else
		{
			return NRF_SPIM_MODE_2;
		}
	}
	else
	{
		if (SPI_MODE_GET(operation) & SPI_MODE_CPHA)
		{
			return NRF_SPIM_MODE_1;
		}
		else
		{
			return NRF_SPIM_MODE_0;
		}
	}
}

static inline nrf_spim_bit_order_t get_nrf_spim_bit_order(uint16_t operation)
{
	if (operation & SPI_TRANSFER_LSB)
	{
		return NRF_SPIM_BIT_ORDER_LSB_FIRST;
	}
	else
	{
		return NRF_SPIM_BIT_ORDER_MSB_FIRST;
	}
}

/**
 * @brief Set all CS pins to input
 */
// TODO: rewrite with struct device* of whole spi device
static int cs_set_to_input(const struct device *dev)
{
	struct spi_nrfx_data *data = get_dev_data(dev);
	const struct spi_nrfx_config *cfg = get_dev_config(dev);

	int err = 0;
	for (int i = 0; i < CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT; i++)
	{
		err = gpio_pin_configure(data->cs_pin_devs[i], cfg->cs_pin_nums[i], GPIO_INPUT | cfg->cs_pin_flags[i]);
		if (err)
		{
			LOG_ERR("gpio_pin_configure err: %d", err);
		}
	}

	return 0;
}

static int signal_pin_set_input(const struct device *dev)
{
	struct spi_nrfx_data *data = get_dev_data(dev);
	const struct spi_nrfx_config *cfg = get_dev_config(dev);

	int err = gpio_pin_configure(data->signal_pin_dev, cfg->signal_pin_num, GPIO_INPUT | cfg->signal_pin_flags);
	if (err)
	{
		LOG_ERR("gpio_pin_configure, err: %d", err);
		return err;
	}
	LOG_DBG("Signal pin is input");

	return 0;
}

static int signal_pin_set_output_high(const struct device *dev)
{
	int err;
	struct spi_nrfx_data *data = get_dev_data(dev);
	const struct spi_nrfx_config *cfg = get_dev_config(dev);

	// put signal pin to high
	err = gpio_pin_configure(data->signal_pin_dev, cfg->signal_pin_num, GPIO_OUTPUT_ACTIVE | cfg->signal_pin_flags);
	if (err)
	{
		LOG_ERR("gpio_pin_configure, err: %d", err);
		return -EIO;
	}
	LOG_DBG("Signal pin is output high");

	return 0;
}

// HACK: is there a better way to do this?
static int config_raw_pin(int raw_pin, int mode_flags)
{
	int err = 0;
	if (raw_pin < 32)
	{
		err = gpio_pin_configure(device_get_binding("GPIO_0"), raw_pin, mode_flags);
	}
	else if (raw_pin < 32 * 2)
	{
		err = gpio_pin_configure(device_get_binding("GPIO_1"), raw_pin - 32, mode_flags);
	}
	else if (raw_pin < 32 * 3)
	{
		err = gpio_pin_configure(device_get_binding("GPIO_1"), raw_pin - 64, mode_flags);
	}
	else
	{
		LOG_ERR("????");
		err = -EIO;
	}
	return err;
}

// HACK: is there a better way to do this?
static int get_raw_pin(int raw_pin)
{
	int err = 0;
	if (raw_pin < 32)
	{
		err = gpio_pin_get(device_get_binding("GPIO_0"), raw_pin);
	}
	else if (raw_pin < 32 * 2)
	{
		err = gpio_pin_get(device_get_binding("GPIO_1"), raw_pin - 32);
	}
	else if (raw_pin < 32 * 3)
	{
		err = gpio_pin_get(device_get_binding("GPIO_1"), raw_pin - 64);
	}
	else
	{
		LOG_ERR("????");
		err = -EIO;
	}
	return err;
}

static int sck_pin_set_input(const struct device *dev)
{
	int err;
	const struct spi_nrfx_config *cfg = get_dev_config(dev);

	// configure clock pin as input,
	err = config_raw_pin(cfg->def_config.sck_pin, GPIO_INPUT);
	if (err)
	{
		LOG_ERR("config_raw_pin, err: %d", err);
		return -EIO;
	}

	return 0;
}

static int sck_pin_get(const struct device *dev)
{
	int err = 0;
	const struct spi_nrfx_config *cfg = get_dev_config(dev);

	err = get_raw_pin(cfg->def_config.sck_pin);

	return err; // the value
}

static int spi_pins_to_input(const struct device *dev)
{
	int err = 0;
	const struct spi_nrfx_config *cfg = get_dev_config(dev);

	// configure clock pin as input dc
	err = config_raw_pin(cfg->def_config.sck_pin, GPIO_INPUT);
	if (err)
	{
		LOG_ERR("config_raw_pin, err: %d", err);
		return -EIO;
	}

	// configure mosi pin as input dc
	err = config_raw_pin(cfg->def_config.mosi_pin, GPIO_INPUT);
	if (err)
	{
		LOG_ERR("config_raw_pin, err: %d", err);
		return -EIO;
	}

	// configure miso pin as input dc
	err = config_raw_pin(cfg->def_config.miso_pin, GPIO_INPUT);
	if (err)
	{
		LOG_ERR("config_raw_pin, err: %d", err);
		return -EIO;
	}

	return 0;
}

static int sck_pin_check(const struct device *dev)
{
	//  read it for a bit, ...
	int val_sck = 0;
	for (int i = 0; i < 10000; i++) // 10 ms total
	{
		val_sck = sck_pin_get(dev);
		if (val_sck == 1) // only one high means clock is active - spi is in use by master
		{
			break;
		}
		k_busy_wait(1); // 1us
	}

	// if sck pin is high, other MCU is using SPI
	if (val_sck)
	{
		return -EAGAIN;
	}
	return 0;
}

int spi_ext_mutex_acquire(const struct device *dev)
{
	int err;
	struct spi_nrfx_data *data = get_dev_data(dev);
	const struct spi_nrfx_config *cfg = get_dev_config(dev);

	// check if signal pin is low (3 times)
	bool sp_high = false;
	for (int i = 0; i < 3; i++)
	{
		int val = gpio_pin_get(data->signal_pin_dev, cfg->signal_pin_num);
		if (val == 1)
		{
			sp_high = true;
			break;
		}
		k_busy_wait(1000); // 1 ms
	}

	if (sp_high)
	{
		// signal pin is high, wait for it to go low
		int low_c = 0;
		for (int i = 0; i < CONFIG_NRFX_SPIM_EXT_MUTEX_ACQUIRE_TIMEOUT_MS / 2; i++)
		{
			int val = gpio_pin_get(data->signal_pin_dev, cfg->signal_pin_num);
			if (val == 0)
			{
				low_c++;
			}
			if (low_c >= 3)
			{
				sp_high = false;
				break;
			}
			k_busy_wait(1000); // 1 ms
		}
	}

	// if waiting timed out, we can not lock
	if (sp_high)
	{
		LOG_ERR("Timed out waiting for signal pin to go to inactive state");
		return -EAGAIN;
	}

	// put signal pin to high
	err = signal_pin_set_output_high(dev);
	if (err)
	{
		LOG_ERR("signal_pin_set_output_high, err: %d", err);
		return -EIO;
	}

	// if slave role, check if clock is active
	if (cfg->mutex_role == MUTEX_ROLE_SLAVE)
	{
		err = sck_pin_set_input(dev);
		if (err)
		{
			LOG_ERR("sck_pin_set_input, err: %d", err);
			return -EIO;
		}

		for (int i = 0; i < CONFIG_NRFX_SPIM_EXT_MUTEX_ACQUIRE_TIMEOUT_MS / (2 * 2); i++)
		{
			err = sck_pin_check(dev); // this takes 10 ms
			if (err == 0)
			{
				break;
			}
			k_busy_wait(10000); // 10 ms
		}

		if (err)
		{
			LOG_ERR("SCK pin is high even after waiting, err: %d", err);
			return -EAGAIN;
		}
	}

	// unset initialized flag to force reconfiguration of SPI in transcieve
	data->initialized = false;
	return 0;
}

int spi_ext_mutex_release(const struct device *dev)
{
	int err;
	// struct spi_nrfx_data *data = get_dev_data(dev);
	const struct spi_nrfx_config *cfg = get_dev_config(dev);

	// uninit spi
	nrfx_spim_uninit(&cfg->spim);

	// set spi pins to input disconnected
	err = spi_pins_to_input(dev);
	if (err)
	{
		LOG_ERR("spi_pins_to_input, err: %d", err);
	}

	// set cs pins to input pull up
	cs_set_to_input(dev);

	// put signal pin back to input
	err += signal_pin_set_input(dev);

	if (err)
	{
		return -EIO;
	}

	return 0;
}

static int spi_context_mutex_init(const struct device *dev)
{

	struct spi_nrfx_data *data = get_dev_data(dev);
	const struct spi_nrfx_config *cfg = get_dev_config(dev);

	LOG_INF("MUTEX ROLE IS: %s", cfg->mutex_role == 0 ? "master" : "slave");

	// get devices for all CS pins
	for (int i = 0; i < CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT; i++)
	{
		data->cs_pin_devs[i] = device_get_binding(cfg->cs_pin_dev_labels[i]);
	}

	// init signal pin
	data->signal_pin_dev = device_get_binding(cfg->signal_pin_dev_label);
	if (data->signal_pin_dev == NULL)
	{
		LOG_ERR("Signal pin device can not be bound");
		return -ENODEV;
	}

	// call release here to put all pins in correct starting state
	int err = spi_ext_mutex_release(dev);
	if (err)
	{
		LOG_ERR("spi_ext_mutex_release, err: %d", err);
		return err;
	}

	return 0;
}

static int configure(const struct device *dev,
					 const struct spi_config *spi_cfg)
{
	struct spi_nrfx_data *dev_data = get_dev_data(dev);
	const struct spi_nrfx_config *dev_config = get_dev_config(dev);
	struct spi_context *ctx = &dev_data->ctx;
	uint32_t max_freq = dev_config->max_freq;
	nrfx_spim_config_t config;
	nrfx_err_t result;

	if (dev_data->initialized && spi_context_configured(ctx, spi_cfg))
	{
		/* Already configured. No need to do it again. */
		return 0;
	}

	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_MASTER)
	{
		LOG_ERR("Slave mode is not supported on %s", dev->name);
		return -EINVAL;
	}

	if (spi_cfg->operation & SPI_MODE_LOOP)
	{
		LOG_ERR("Loopback mode is not supported");
		return -EINVAL;
	}

	if ((spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE)
	{
		LOG_ERR("Only single line mode is supported");
		return -EINVAL;
	}

	if (SPI_WORD_SIZE_GET(spi_cfg->operation) != 8)
	{
		LOG_ERR("Word sizes other than 8 bits are not supported");
		return -EINVAL;
	}

	if (spi_cfg->frequency < 125000)
	{
		LOG_ERR("Frequencies lower than 125 kHz are not supported");
		return -EINVAL;
	}

#if defined(CONFIG_SOC_NRF5340_CPUAPP)
	/* On nRF5340, the 32 Mbps speed is supported by the application core
	 * when it is running at 128 MHz (see the Timing specifications section
	 * in the nRF5340 PS).
	 */
	if (max_freq > 16000000 &&
		nrf_clock_hfclk_div_get(NRF_CLOCK) != NRF_CLOCK_HFCLK_DIV_1)
	{
		max_freq = 16000000;
	}
#endif
	config = dev_config->def_config;
	/* Limit the frequency to that supported by the SPIM instance. */
	config.frequency = get_nrf_spim_frequency(MIN(spi_cfg->frequency,
												  max_freq));
	config.mode = get_nrf_spim_mode(spi_cfg->operation);
	config.bit_order = get_nrf_spim_bit_order(spi_cfg->operation);
	if (dev_data->initialized)
	{
		nrfx_spim_uninit(&dev_config->spim);
		dev_data->initialized = false;
	}
	result = nrfx_spim_init(&dev_config->spim, &config,
							event_handler, dev_data);
	if (result != NRFX_SUCCESS)
	{
		LOG_ERR("Failed to initialize nrfx driver: %08x", result);
		return -EIO;
	}

	dev_data->initialized = true;

	ctx->config = spi_cfg;
	spi_context_cs_configure(ctx);

	return 0;
}

static void transfer_next_chunk(const struct device *dev)
{
	struct spi_nrfx_data *dev_data = get_dev_data(dev);
	const struct spi_nrfx_config *dev_config = get_dev_config(dev);
	struct spi_context *ctx = &dev_data->ctx;
	int error = 0;

	size_t chunk_len = spi_context_max_continuous_chunk(ctx);

	if (chunk_len > 0)
	{
		nrfx_spim_xfer_desc_t xfer;
		nrfx_err_t result;
		const uint8_t *tx_buf = ctx->tx_buf;
#if (CONFIG_SPI_NRFX_RAM_BUFFER_SIZE > 0)
		if (spi_context_tx_buf_on(ctx) && !nrfx_is_in_ram(tx_buf))
		{
			if (chunk_len > sizeof(dev_data->buffer))
			{
				chunk_len = sizeof(dev_data->buffer);
			}

			memcpy(dev_data->buffer, tx_buf, chunk_len);
			tx_buf = dev_data->buffer;
		}
#endif
		if (chunk_len > dev_config->max_chunk_len)
		{
			chunk_len = dev_config->max_chunk_len;
		}

		dev_data->chunk_len = chunk_len;

		xfer.p_tx_buffer = tx_buf;
		xfer.tx_length = spi_context_tx_buf_on(ctx) ? chunk_len : 0;
		xfer.p_rx_buffer = ctx->rx_buf;
		xfer.rx_length = spi_context_rx_buf_on(ctx) ? chunk_len : 0;

		/* This SPIM driver is only used by the NRF52832 if
		   SOC_NRF52832_ALLOW_SPIM_DESPITE_PAN_58 is enabled */
		if (IS_ENABLED(CONFIG_SOC_NRF52832) &&
			(xfer.rx_length == 1 && xfer.tx_length <= 1))
		{
			LOG_WRN("Transaction aborted since it would trigger nRF52832 PAN 58");
			error = -EIO;
		}

		if (!error)
		{
			result = nrfx_spim_xfer(&dev_config->spim, &xfer, 0);
			if (result == NRFX_SUCCESS)
			{
				return;
			}
			error = -EIO;
		}
	}

	spi_context_cs_control(ctx, false);

	LOG_DBG("Transaction finished with status %d", error);

	spi_context_complete(ctx, error);
	dev_data->busy = false;
}

static void event_handler(const nrfx_spim_evt_t *p_event, void *p_context)
{
	struct spi_nrfx_data *dev_data = p_context;
	if (p_event->type == NRFX_SPIM_EVENT_DONE)
	{
		spi_context_update_tx(&dev_data->ctx, 1, dev_data->chunk_len);
		spi_context_update_rx(&dev_data->ctx, 1, dev_data->chunk_len);
		transfer_next_chunk(dev_data->dev);
	}
}

static int transceive(const struct device *dev,
					  const struct spi_config *spi_cfg,
					  const struct spi_buf_set *tx_bufs,
					  const struct spi_buf_set *rx_bufs,
					  bool asynchronous,
					  struct k_poll_signal *signal)
{
	struct spi_nrfx_data *dev_data = get_dev_data(dev);
	int error = 0;

	spi_context_lock(&dev_data->ctx, asynchronous, signal, spi_cfg);

	error = configure(dev, spi_cfg);
	if (error == 0)
	{
		dev_data->busy = true;

		spi_context_buffers_setup(&dev_data->ctx, tx_bufs, rx_bufs, 1);
		spi_context_cs_control(&dev_data->ctx, true);

		transfer_next_chunk(dev);

		error = spi_context_wait_for_completion(&dev_data->ctx);
	}
	spi_context_release(&dev_data->ctx, error);

	return error;
}

static int spi_nrfx_transceive(const struct device *dev,
							   const struct spi_config *spi_cfg,
							   const struct spi_buf_set *tx_bufs,
							   const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_nrfx_transceive_async(const struct device *dev,
									 const struct spi_config *spi_cfg,
									 const struct spi_buf_set *tx_bufs,
									 const struct spi_buf_set *rx_bufs,
									 struct k_poll_signal *async)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_nrfx_release(const struct device *dev,
							const struct spi_config *spi_cfg)
{
	struct spi_nrfx_data *dev_data = get_dev_data(dev);

	if (!spi_context_configured(&dev_data->ctx, spi_cfg))
	{
		return -EINVAL;
	}

	if (dev_data->busy)
	{
		return -EBUSY;
	}

	spi_context_unlock_unconditionally(&dev_data->ctx);

	return 0;
}

static const struct spi_driver_api spi_nrfx_driver_api = {
	.transceive = spi_nrfx_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_nrfx_transceive_async,
#endif
	.release = spi_nrfx_release,
};

#ifdef CONFIG_PM_DEVICE
static int spim_nrfx_pm_control(const struct device *dev,
								enum pm_device_action action)
{
	int ret = 0;
	struct spi_nrfx_data *data = get_dev_data(dev);
	const struct spi_nrfx_config *config = get_dev_config(dev);

	switch (action)
	{
	case PM_DEVICE_ACTION_RESUME:
		/* No action needed at this point, nrfx_spim_init() will be
		 * called at configuration before the next transfer.
		 */
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		nrfx_spim_uninit(&config->spim);
		data->initialized = false;
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/*
 * We use NODELABEL here because the nrfx API requires us to call
 * functions which are named according to SoC peripheral instance
 * being operated on. Since DT_INST() makes no guarantees about that,
 * it won't work.
 */
#define SPIM(idx) DT_NODELABEL(spi##idx)
#define SPIM_PROP(idx, prop) DT_PROP(SPIM(idx), prop)

#define SPIM_NRFX_MISO_PULL_DOWN(idx) DT_PROP(SPIM(idx), miso_pull_down)
#define SPIM_NRFX_MISO_PULL_UP(idx) DT_PROP(SPIM(idx), miso_pull_up)

#define SPIM_NRFX_MISO_PULL(idx)                \
	(SPIM_NRFX_MISO_PULL_UP(idx)                \
		 ? SPIM_NRFX_MISO_PULL_DOWN(idx)        \
			   ? -1 /* invalid configuration */ \
			   : NRF_GPIO_PIN_PULLUP            \
	 : SPIM_NRFX_MISO_PULL_DOWN(idx)            \
		 ? NRF_GPIO_PIN_PULLDOWN                \
		 : NRF_GPIO_PIN_NOPULL)

#define SPI_NRFX_SPIM_EXTENDED_CONFIG(idx)                      \
	IF_ENABLED(NRFX_SPIM_EXTENDED_ENABLED,                      \
			   (.dcx_pin = NRFX_SPIM_PIN_NOT_USED,              \
				IF_ENABLED(SPIM##idx##_FEATURE_RXDELAY_PRESENT, \
						   (.rx_delay = CONFIG_SPI_##idx##_NRF_RX_DELAY, ))))

// NOTE: based on the count of CS pins, a different macro for spi device initialization
//       is selected. I found no better way to do this.
#if CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT == 1
#define SPI_NRFX_SPIM_DEVICE(idx)                                             \
	BUILD_ASSERT(                                                             \
		!SPIM_NRFX_MISO_PULL_UP(idx) || !SPIM_NRFX_MISO_PULL_DOWN(idx),       \
		"SPIM" #idx                                                           \
		": cannot enable both pull-up and pull-down on MISO line");           \
	static int spi_##idx##_init(const struct device *dev)                     \
	{                                                                         \
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM##idx),                       \
					DT_IRQ(SPIM(idx), priority),                              \
					nrfx_isr, nrfx_spim_##idx##_irq_handler, 0);              \
                                                                              \
		int err = spi_context_mutex_init(dev);                                \
		if (err)                                                              \
		{                                                                     \
			LOG_ERR("spi_context_mutex_init, err: %d", err);                  \
			return err;                                                       \
		}                                                                     \
		spi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);          \
		return 0;                                                             \
	}                                                                         \
	static struct spi_nrfx_data spi_##idx##_data = {                          \
		SPI_CONTEXT_INIT_LOCK(spi_##idx##_data, ctx),                         \
		SPI_CONTEXT_INIT_SYNC(spi_##idx##_data, ctx),                         \
		.dev = DEVICE_DT_GET(SPIM(idx)),                                      \
		.busy = false,                                                        \
	};                                                                        \
	static const struct spi_nrfx_config spi_##idx##z_config = {               \
		.spim = NRFX_SPIM_INSTANCE(idx),                                      \
		.max_chunk_len = (1 << SPIM##idx##_EASYDMA_MAXCNT_SIZE) - 1,          \
		.max_freq = SPIM##idx##_MAX_DATARATE * 1000000,                       \
		.def_config = {                                                       \
			.sck_pin = SPIM_PROP(idx, sck_pin),                               \
			.mosi_pin = SPIM_PROP(idx, mosi_pin),                             \
			.miso_pin = SPIM_PROP(idx, miso_pin),                             \
			.ss_pin = NRFX_SPIM_PIN_NOT_USED,                                 \
			.orc = CONFIG_SPI_##idx##_NRF_ORC,                                \
			.miso_pull = SPIM_NRFX_MISO_PULL(idx),                            \
			SPI_NRFX_SPIM_EXTENDED_CONFIG(idx)},                              \
		.signal_pin_dev_label = DT_GPIO_LABEL(SPIM(idx), signal_gpios),       \
		.signal_pin_num = DT_GPIO_PIN(SPIM(idx), signal_gpios),               \
		.signal_pin_flags = DT_GPIO_FLAGS(SPIM(idx), signal_gpios),           \
		.mutex_role = DT_ENUM_IDX(SPIM(idx), mutex_role),                     \
		.cs_pin_dev_labels[0] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 0), \
		.cs_pin_nums[0] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 0),         \
		.cs_pin_flags[0] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 0)};     \
	DEVICE_DT_DEFINE(SPIM(idx),                                               \
					 spi_##idx##_init,                                        \
					 spim_nrfx_pm_control,                                    \
					 &spi_##idx##_data,                                       \
					 &spi_##idx##z_config,                                    \
					 POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                   \
					 &spi_nrfx_driver_api)
#endif

#if CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT == 2
#define SPI_NRFX_SPIM_DEVICE(idx)                                             \
	BUILD_ASSERT(                                                             \
		!SPIM_NRFX_MISO_PULL_UP(idx) || !SPIM_NRFX_MISO_PULL_DOWN(idx),       \
		"SPIM" #idx                                                           \
		": cannot enable both pull-up and pull-down on MISO line");           \
	static int spi_##idx##_init(const struct device *dev)                     \
	{                                                                         \
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM##idx),                       \
					DT_IRQ(SPIM(idx), priority),                              \
					nrfx_isr, nrfx_spim_##idx##_irq_handler, 0);              \
                                                                              \
		int err = spi_context_mutex_init(dev);                                \
		if (err)                                                              \
		{                                                                     \
			LOG_ERR("spi_context_mutex_init, err: %d", err);                  \
			return err;                                                       \
		}                                                                     \
		spi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);          \
		return 0;                                                             \
	}                                                                         \
	static struct spi_nrfx_data spi_##idx##_data = {                          \
		SPI_CONTEXT_INIT_LOCK(spi_##idx##_data, ctx),                         \
		SPI_CONTEXT_INIT_SYNC(spi_##idx##_data, ctx),                         \
		.dev = DEVICE_DT_GET(SPIM(idx)),                                      \
		.busy = false,                                                        \
	};                                                                        \
	static const struct spi_nrfx_config spi_##idx##z_config = {               \
		.spim = NRFX_SPIM_INSTANCE(idx),                                      \
		.max_chunk_len = (1 << SPIM##idx##_EASYDMA_MAXCNT_SIZE) - 1,          \
		.max_freq = SPIM##idx##_MAX_DATARATE * 1000000,                       \
		.def_config = {                                                       \
			.sck_pin = SPIM_PROP(idx, sck_pin),                               \
			.mosi_pin = SPIM_PROP(idx, mosi_pin),                             \
			.miso_pin = SPIM_PROP(idx, miso_pin),                             \
			.ss_pin = NRFX_SPIM_PIN_NOT_USED,                                 \
			.orc = CONFIG_SPI_##idx##_NRF_ORC,                                \
			.miso_pull = SPIM_NRFX_MISO_PULL(idx),                            \
			SPI_NRFX_SPIM_EXTENDED_CONFIG(idx)},                              \
		.signal_pin_dev_label = DT_GPIO_LABEL(SPIM(idx), signal_gpios),       \
		.signal_pin_num = DT_GPIO_PIN(SPIM(idx), signal_gpios),               \
		.signal_pin_flags = DT_GPIO_FLAGS(SPIM(idx), signal_gpios),           \
		.mutex_role = DT_ENUM_IDX(SPIM(idx), mutex_role),                     \
		.cs_pin_dev_labels[0] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 0), \
		.cs_pin_nums[0] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 0),         \
		.cs_pin_flags[0] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 0),      \
		.cs_pin_dev_labels[1] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 1), \
		.cs_pin_nums[1] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 1),         \
		.cs_pin_flags[1] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 1)};     \
	DEVICE_DT_DEFINE(SPIM(idx),                                               \
					 spi_##idx##_init,                                        \
					 spim_nrfx_pm_control,                                    \
					 &spi_##idx##_data,                                       \
					 &spi_##idx##z_config,                                    \
					 POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                   \
					 &spi_nrfx_driver_api)
#endif

#if CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT == 3
#define SPI_NRFX_SPIM_DEVICE(idx)                                             \
	BUILD_ASSERT(                                                             \
		!SPIM_NRFX_MISO_PULL_UP(idx) || !SPIM_NRFX_MISO_PULL_DOWN(idx),       \
		"SPIM" #idx                                                           \
		": cannot enable both pull-up and pull-down on MISO line");           \
	static int spi_##idx##_init(const struct device *dev)                     \
	{                                                                         \
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM##idx),                       \
					DT_IRQ(SPIM(idx), priority),                              \
					nrfx_isr, nrfx_spim_##idx##_irq_handler, 0);              \
                                                                              \
		int err = spi_context_mutex_init(dev);                                \
		if (err)                                                              \
		{                                                                     \
			LOG_ERR("spi_context_mutex_init, err: %d", err);                  \
			return err;                                                       \
		}                                                                     \
		spi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);          \
		return 0;                                                             \
	}                                                                         \
	static struct spi_nrfx_data spi_##idx##_data = {                          \
		SPI_CONTEXT_INIT_LOCK(spi_##idx##_data, ctx),                         \
		SPI_CONTEXT_INIT_SYNC(spi_##idx##_data, ctx),                         \
		.dev = DEVICE_DT_GET(SPIM(idx)),                                      \
		.busy = false,                                                        \
	};                                                                        \
	static const struct spi_nrfx_config spi_##idx##z_config = {               \
		.spim = NRFX_SPIM_INSTANCE(idx),                                      \
		.max_chunk_len = (1 << SPIM##idx##_EASYDMA_MAXCNT_SIZE) - 1,          \
		.max_freq = SPIM##idx##_MAX_DATARATE * 1000000,                       \
		.def_config = {                                                       \
			.sck_pin = SPIM_PROP(idx, sck_pin),                               \
			.mosi_pin = SPIM_PROP(idx, mosi_pin),                             \
			.miso_pin = SPIM_PROP(idx, miso_pin),                             \
			.ss_pin = NRFX_SPIM_PIN_NOT_USED,                                 \
			.orc = CONFIG_SPI_##idx##_NRF_ORC,                                \
			.miso_pull = SPIM_NRFX_MISO_PULL(idx),                            \
			SPI_NRFX_SPIM_EXTENDED_CONFIG(idx)},                              \
		.signal_pin_dev_label = DT_GPIO_LABEL(SPIM(idx), signal_gpios),       \
		.signal_pin_num = DT_GPIO_PIN(SPIM(idx), signal_gpios),               \
		.signal_pin_flags = DT_GPIO_FLAGS(SPIM(idx), signal_gpios),           \
		.mutex_role = DT_ENUM_IDX(SPIM(idx), mutex_role),                     \
		.cs_pin_dev_labels[0] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 0), \
		.cs_pin_nums[0] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 0),         \
		.cs_pin_flags[0] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 0),      \
		.cs_pin_dev_labels[1] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 1), \
		.cs_pin_nums[1] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 1),         \
		.cs_pin_flags[1] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 1),      \
		.cs_pin_dev_labels[2] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 2), \
		.cs_pin_nums[2] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 2),         \
		.cs_pin_flags[2] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 2)};     \
	DEVICE_DT_DEFINE(SPIM(idx),                                               \
					 spi_##idx##_init,                                        \
					 spim_nrfx_pm_control,                                    \
					 &spi_##idx##_data,                                       \
					 &spi_##idx##z_config,                                    \
					 POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                   \
					 &spi_nrfx_driver_api)
#endif

#if CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT == 4
#define SPI_NRFX_SPIM_DEVICE(idx)                                             \
	BUILD_ASSERT(                                                             \
		!SPIM_NRFX_MISO_PULL_UP(idx) || !SPIM_NRFX_MISO_PULL_DOWN(idx),       \
		"SPIM" #idx                                                           \
		": cannot enable both pull-up and pull-down on MISO line");           \
	static int spi_##idx##_init(const struct device *dev)                     \
	{                                                                         \
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM##idx),                       \
					DT_IRQ(SPIM(idx), priority),                              \
					nrfx_isr, nrfx_spim_##idx##_irq_handler, 0);              \
                                                                              \
		int err = spi_context_mutex_init(dev);                                \
		if (err)                                                              \
		{                                                                     \
			LOG_ERR("spi_context_mutex_init, err: %d", err);                  \
			return err;                                                       \
		}                                                                     \
		spi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);          \
		return 0;                                                             \
	}                                                                         \
	static struct spi_nrfx_data spi_##idx##_data = {                          \
		SPI_CONTEXT_INIT_LOCK(spi_##idx##_data, ctx),                         \
		SPI_CONTEXT_INIT_SYNC(spi_##idx##_data, ctx),                         \
		.dev = DEVICE_DT_GET(SPIM(idx)),                                      \
		.busy = false,                                                        \
	};                                                                        \
	static const struct spi_nrfx_config spi_##idx##z_config = {               \
		.spim = NRFX_SPIM_INSTANCE(idx),                                      \
		.max_chunk_len = (1 << SPIM##idx##_EASYDMA_MAXCNT_SIZE) - 1,          \
		.max_freq = SPIM##idx##_MAX_DATARATE * 1000000,                       \
		.def_config = {                                                       \
			.sck_pin = SPIM_PROP(idx, sck_pin),                               \
			.mosi_pin = SPIM_PROP(idx, mosi_pin),                             \
			.miso_pin = SPIM_PROP(idx, miso_pin),                             \
			.ss_pin = NRFX_SPIM_PIN_NOT_USED,                                 \
			.orc = CONFIG_SPI_##idx##_NRF_ORC,                                \
			.miso_pull = SPIM_NRFX_MISO_PULL(idx),                            \
			SPI_NRFX_SPIM_EXTENDED_CONFIG(idx)},                              \
		.signal_pin_dev_label = DT_GPIO_LABEL(SPIM(idx), signal_gpios),       \
		.signal_pin_num = DT_GPIO_PIN(SPIM(idx), signal_gpios),               \
		.signal_pin_flags = DT_GPIO_FLAGS(SPIM(idx), signal_gpios),           \
		.mutex_role = DT_ENUM_IDX(SPIM(idx), mutex_role),                     \
		.cs_pin_dev_labels[0] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 0), \
		.cs_pin_nums[0] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 0),         \
		.cs_pin_flags[0] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 0),      \
		.cs_pin_dev_labels[1] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 1), \
		.cs_pin_nums[1] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 1),         \
		.cs_pin_flags[1] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 1),      \
		.cs_pin_dev_labels[2] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 2), \
		.cs_pin_nums[2] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 2),         \
		.cs_pin_flags[2] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 2),      \
		.cs_pin_dev_labels[3] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 3), \
		.cs_pin_nums[3] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 3),         \
		.cs_pin_flags[3] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 3)};     \
	DEVICE_DT_DEFINE(SPIM(idx),                                               \
					 spi_##idx##_init,                                        \
					 spim_nrfx_pm_control,                                    \
					 &spi_##idx##_data,                                       \
					 &spi_##idx##z_config,                                    \
					 POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                   \
					 &spi_nrfx_driver_api)
#endif

#if CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT == 5
#define SPI_NRFX_SPIM_DEVICE(idx)                                             \
	BUILD_ASSERT(                                                             \
		!SPIM_NRFX_MISO_PULL_UP(idx) || !SPIM_NRFX_MISO_PULL_DOWN(idx),       \
		"SPIM" #idx                                                           \
		": cannot enable both pull-up and pull-down on MISO line");           \
	static int spi_##idx##_init(const struct device *dev)                     \
	{                                                                         \
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM##idx),                       \
					DT_IRQ(SPIM(idx), priority),                              \
					nrfx_isr, nrfx_spim_##idx##_irq_handler, 0);              \
                                                                              \
		int err = spi_context_mutex_init(dev);                                \
		if (err)                                                              \
		{                                                                     \
			LOG_ERR("spi_context_mutex_init, err: %d", err);                  \
			return err;                                                       \
		}                                                                     \
		spi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);          \
		return 0;                                                             \
	}                                                                         \
	static struct spi_nrfx_data spi_##idx##_data = {                          \
		SPI_CONTEXT_INIT_LOCK(spi_##idx##_data, ctx),                         \
		SPI_CONTEXT_INIT_SYNC(spi_##idx##_data, ctx),                         \
		.dev = DEVICE_DT_GET(SPIM(idx)),                                      \
		.busy = false,                                                        \
	};                                                                        \
	static const struct spi_nrfx_config spi_##idx##z_config = {               \
		.spim = NRFX_SPIM_INSTANCE(idx),                                      \
		.max_chunk_len = (1 << SPIM##idx##_EASYDMA_MAXCNT_SIZE) - 1,          \
		.max_freq = SPIM##idx##_MAX_DATARATE * 1000000,                       \
		.def_config = {                                                       \
			.sck_pin = SPIM_PROP(idx, sck_pin),                               \
			.mosi_pin = SPIM_PROP(idx, mosi_pin),                             \
			.miso_pin = SPIM_PROP(idx, miso_pin),                             \
			.ss_pin = NRFX_SPIM_PIN_NOT_USED,                                 \
			.orc = CONFIG_SPI_##idx##_NRF_ORC,                                \
			.miso_pull = SPIM_NRFX_MISO_PULL(idx),                            \
			SPI_NRFX_SPIM_EXTENDED_CONFIG(idx)},                              \
		.signal_pin_dev_label = DT_GPIO_LABEL(SPIM(idx), signal_gpios),       \
		.signal_pin_num = DT_GPIO_PIN(SPIM(idx), signal_gpios),               \
		.signal_pin_flags = DT_GPIO_FLAGS(SPIM(idx), signal_gpios),           \
		.mutex_role = DT_ENUM_IDX(SPIM(idx), mutex_role),                     \
		.cs_pin_dev_labels[0] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 0), \
		.cs_pin_nums[0] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 0),         \
		.cs_pin_flags[0] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 0),      \
		.cs_pin_dev_labels[1] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 1), \
		.cs_pin_nums[1] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 1),         \
		.cs_pin_flags[1] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 1),      \
		.cs_pin_dev_labels[2] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 2), \
		.cs_pin_nums[2] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 2),         \
		.cs_pin_flags[2] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 2),      \
		.cs_pin_dev_labels[3] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 3), \
		.cs_pin_nums[3] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 3),         \
		.cs_pin_flags[3] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 3),      \
		.cs_pin_dev_labels[4] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 4), \
		.cs_pin_nums[4] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 4),         \
		.cs_pin_flags[4] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 4)};     \
	DEVICE_DT_DEFINE(SPIM(idx),                                               \
					 spi_##idx##_init,                                        \
					 spim_nrfx_pm_control,                                    \
					 &spi_##idx##_data,                                       \
					 &spi_##idx##z_config,                                    \
					 POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                   \
					 &spi_nrfx_driver_api)
#endif

#if CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT == 6
#define SPI_NRFX_SPIM_DEVICE(idx)                                             \
	BUILD_ASSERT(                                                             \
		!SPIM_NRFX_MISO_PULL_UP(idx) || !SPIM_NRFX_MISO_PULL_DOWN(idx),       \
		"SPIM" #idx                                                           \
		": cannot enable both pull-up and pull-down on MISO line");           \
	static int spi_##idx##_init(const struct device *dev)                     \
	{                                                                         \
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM##idx),                       \
					DT_IRQ(SPIM(idx), priority),                              \
					nrfx_isr, nrfx_spim_##idx##_irq_handler, 0);              \
                                                                              \
		int err = spi_context_mutex_init(dev);                                \
		if (err)                                                              \
		{                                                                     \
			LOG_ERR("spi_context_mutex_init, err: %d", err);                  \
			return err;                                                       \
		}                                                                     \
		spi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);          \
		return 0;                                                             \
	}                                                                         \
	static struct spi_nrfx_data spi_##idx##_data = {                          \
		SPI_CONTEXT_INIT_LOCK(spi_##idx##_data, ctx),                         \
		SPI_CONTEXT_INIT_SYNC(spi_##idx##_data, ctx),                         \
		.dev = DEVICE_DT_GET(SPIM(idx)),                                      \
		.busy = false,                                                        \
	};                                                                        \
	static const struct spi_nrfx_config spi_##idx##z_config = {               \
		.spim = NRFX_SPIM_INSTANCE(idx),                                      \
		.max_chunk_len = (1 << SPIM##idx##_EASYDMA_MAXCNT_SIZE) - 1,          \
		.max_freq = SPIM##idx##_MAX_DATARATE * 1000000,                       \
		.def_config = {                                                       \
			.sck_pin = SPIM_PROP(idx, sck_pin),                               \
			.mosi_pin = SPIM_PROP(idx, mosi_pin),                             \
			.miso_pin = SPIM_PROP(idx, miso_pin),                             \
			.ss_pin = NRFX_SPIM_PIN_NOT_USED,                                 \
			.orc = CONFIG_SPI_##idx##_NRF_ORC,                                \
			.miso_pull = SPIM_NRFX_MISO_PULL(idx),                            \
			SPI_NRFX_SPIM_EXTENDED_CONFIG(idx)},                              \
		.signal_pin_dev_label = DT_GPIO_LABEL(SPIM(idx), signal_gpios),       \
		.signal_pin_num = DT_GPIO_PIN(SPIM(idx), signal_gpios),               \
		.signal_pin_flags = DT_GPIO_FLAGS(SPIM(idx), signal_gpios),           \
		.mutex_role = DT_ENUM_IDX(SPIM(idx), mutex_role),                     \
		.cs_pin_dev_labels[0] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 0), \
		.cs_pin_nums[0] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 0),         \
		.cs_pin_flags[0] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 0),      \
		.cs_pin_dev_labels[1] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 1), \
		.cs_pin_nums[1] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 1),         \
		.cs_pin_flags[1] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 1),      \
		.cs_pin_dev_labels[2] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 2), \
		.cs_pin_nums[2] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 2),         \
		.cs_pin_flags[2] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 2),      \
		.cs_pin_dev_labels[3] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 3), \
		.cs_pin_nums[3] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 3),         \
		.cs_pin_flags[3] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 3),      \
		.cs_pin_dev_labels[4] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 4), \
		.cs_pin_nums[4] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 4),         \
		.cs_pin_flags[4] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 4),      \
		.cs_pin_dev_labels[5] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 5), \
		.cs_pin_nums[5] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 5),         \
		.cs_pin_flags[5] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 5)};     \
	DEVICE_DT_DEFINE(SPIM(idx),                                               \
					 spi_##idx##_init,                                        \
					 spim_nrfx_pm_control,                                    \
					 &spi_##idx##_data,                                       \
					 &spi_##idx##z_config,                                    \
					 POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                   \
					 &spi_nrfx_driver_api)
#endif

#if CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT == 7
#define SPI_NRFX_SPIM_DEVICE(idx)                                             \
	BUILD_ASSERT(                                                             \
		!SPIM_NRFX_MISO_PULL_UP(idx) || !SPIM_NRFX_MISO_PULL_DOWN(idx),       \
		"SPIM" #idx                                                           \
		": cannot enable both pull-up and pull-down on MISO line");           \
	static int spi_##idx##_init(const struct device *dev)                     \
	{                                                                         \
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM##idx),                       \
					DT_IRQ(SPIM(idx), priority),                              \
					nrfx_isr, nrfx_spim_##idx##_irq_handler, 0);              \
                                                                              \
		int err = spi_context_mutex_init(dev);                                \
		if (err)                                                              \
		{                                                                     \
			LOG_ERR("spi_context_mutex_init, err: %d", err);                  \
			return err;                                                       \
		}                                                                     \
		spi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);          \
		return 0;                                                             \
	}                                                                         \
	static struct spi_nrfx_data spi_##idx##_data = {                          \
		SPI_CONTEXT_INIT_LOCK(spi_##idx##_data, ctx),                         \
		SPI_CONTEXT_INIT_SYNC(spi_##idx##_data, ctx),                         \
		.dev = DEVICE_DT_GET(SPIM(idx)),                                      \
		.busy = false,                                                        \
	};                                                                        \
	static const struct spi_nrfx_config spi_##idx##z_config = {               \
		.spim = NRFX_SPIM_INSTANCE(idx),                                      \
		.max_chunk_len = (1 << SPIM##idx##_EASYDMA_MAXCNT_SIZE) - 1,          \
		.max_freq = SPIM##idx##_MAX_DATARATE * 1000000,                       \
		.def_config = {                                                       \
			.sck_pin = SPIM_PROP(idx, sck_pin),                               \
			.mosi_pin = SPIM_PROP(idx, mosi_pin),                             \
			.miso_pin = SPIM_PROP(idx, miso_pin),                             \
			.ss_pin = NRFX_SPIM_PIN_NOT_USED,                                 \
			.orc = CONFIG_SPI_##idx##_NRF_ORC,                                \
			.miso_pull = SPIM_NRFX_MISO_PULL(idx),                            \
			SPI_NRFX_SPIM_EXTENDED_CONFIG(idx)},                              \
		.signal_pin_dev_label = DT_GPIO_LABEL(SPIM(idx), signal_gpios),       \
		.signal_pin_num = DT_GPIO_PIN(SPIM(idx), signal_gpios),               \
		.signal_pin_flags = DT_GPIO_FLAGS(SPIM(idx), signal_gpios),           \
		.mutex_role = DT_ENUM_IDX(SPIM(idx), mutex_role),                     \
		.cs_pin_dev_labels[0] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 0), \
		.cs_pin_nums[0] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 0),         \
		.cs_pin_flags[0] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 0),      \
		.cs_pin_dev_labels[1] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 1), \
		.cs_pin_nums[1] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 1),         \
		.cs_pin_flags[1] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 1),      \
		.cs_pin_dev_labels[2] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 2), \
		.cs_pin_nums[2] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 2),         \
		.cs_pin_flags[2] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 2),      \
		.cs_pin_dev_labels[3] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 3), \
		.cs_pin_nums[3] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 3),         \
		.cs_pin_flags[3] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 3),      \
		.cs_pin_dev_labels[4] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 4), \
		.cs_pin_nums[4] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 4),         \
		.cs_pin_flags[4] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 4),      \
		.cs_pin_dev_labels[5] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 5), \
		.cs_pin_nums[5] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 5),         \
		.cs_pin_flags[5] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 5),      \
		.cs_pin_dev_labels[6] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 6), \
		.cs_pin_nums[6] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 6),         \
		.cs_pin_flags[6] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 6)};     \
	DEVICE_DT_DEFINE(SPIM(idx),                                               \
					 spi_##idx##_init,                                        \
					 spim_nrfx_pm_control,                                    \
					 &spi_##idx##_data,                                       \
					 &spi_##idx##z_config,                                    \
					 POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                   \
					 &spi_nrfx_driver_api)
#endif

#if CONFIG_NRFX_SPIM_EXT_MUTEX_CS_COUNT == 8
#define SPI_NRFX_SPIM_DEVICE(idx)                                             \
	BUILD_ASSERT(                                                             \
		!SPIM_NRFX_MISO_PULL_UP(idx) || !SPIM_NRFX_MISO_PULL_DOWN(idx),       \
		"SPIM" #idx                                                           \
		": cannot enable both pull-up and pull-down on MISO line");           \
	static int spi_##idx##_init(const struct device *dev)                     \
	{                                                                         \
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM##idx),                       \
					DT_IRQ(SPIM(idx), priority),                              \
					nrfx_isr, nrfx_spim_##idx##_irq_handler, 0);              \
                                                                              \
		int err = spi_context_mutex_init(dev);                                \
		if (err)                                                              \
		{                                                                     \
			LOG_ERR("spi_context_mutex_init, err: %d", err);                  \
			return err;                                                       \
		}                                                                     \
		spi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);          \
		return 0;                                                             \
	}                                                                         \
	static struct spi_nrfx_data spi_##idx##_data = {                          \
		SPI_CONTEXT_INIT_LOCK(spi_##idx##_data, ctx),                         \
		SPI_CONTEXT_INIT_SYNC(spi_##idx##_data, ctx),                         \
		.dev = DEVICE_DT_GET(SPIM(idx)),                                      \
		.busy = false,                                                        \
	};                                                                        \
	static const struct spi_nrfx_config spi_##idx##z_config = {               \
		.spim = NRFX_SPIM_INSTANCE(idx),                                      \
		.max_chunk_len = (1 << SPIM##idx##_EASYDMA_MAXCNT_SIZE) - 1,          \
		.max_freq = SPIM##idx##_MAX_DATARATE * 1000000,                       \
		.def_config = {                                                       \
			.sck_pin = SPIM_PROP(idx, sck_pin),                               \
			.mosi_pin = SPIM_PROP(idx, mosi_pin),                             \
			.miso_pin = SPIM_PROP(idx, miso_pin),                             \
			.ss_pin = NRFX_SPIM_PIN_NOT_USED,                                 \
			.orc = CONFIG_SPI_##idx##_NRF_ORC,                                \
			.miso_pull = SPIM_NRFX_MISO_PULL(idx),                            \
			SPI_NRFX_SPIM_EXTENDED_CONFIG(idx)},                              \
		.signal_pin_dev_label = DT_GPIO_LABEL(SPIM(idx), signal_gpios),       \
		.signal_pin_num = DT_GPIO_PIN(SPIM(idx), signal_gpios),               \
		.signal_pin_flags = DT_GPIO_FLAGS(SPIM(idx), signal_gpios),           \
		.mutex_role = DT_ENUM_IDX(SPIM(idx), mutex_role),                     \
		.cs_pin_dev_labels[0] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 0), \
		.cs_pin_nums[0] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 0),         \
		.cs_pin_flags[0] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 0),      \
		.cs_pin_dev_labels[1] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 1), \
		.cs_pin_nums[1] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 1),         \
		.cs_pin_flags[1] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 1),      \
		.cs_pin_dev_labels[2] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 2), \
		.cs_pin_nums[2] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 2),         \
		.cs_pin_flags[2] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 2),      \
		.cs_pin_dev_labels[3] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 3), \
		.cs_pin_nums[3] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 3),         \
		.cs_pin_flags[3] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 3),      \
		.cs_pin_dev_labels[4] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 4), \
		.cs_pin_nums[4] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 4),         \
		.cs_pin_flags[4] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 4),      \
		.cs_pin_dev_labels[5] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 5), \
		.cs_pin_nums[5] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 5),         \
		.cs_pin_flags[5] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 5),      \
		.cs_pin_dev_labels[6] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 6), \
		.cs_pin_nums[6] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 6),         \
		.cs_pin_flags[6] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 6),      \
		.cs_pin_dev_labels[7] = DT_GPIO_LABEL_BY_IDX(SPIM(idx), cs_gpios, 7), \
		.cs_pin_nums[7] = DT_GPIO_PIN_BY_IDX(SPIM(idx), cs_gpios, 7),         \
		.cs_pin_flags[7] = DT_GPIO_FLAGS_BY_IDX(SPIM(idx), cs_gpios, 7)};     \
	DEVICE_DT_DEFINE(SPIM(idx),                                               \
					 spi_##idx##_init,                                        \
					 spim_nrfx_pm_control,                                    \
					 &spi_##idx##_data,                                       \
					 &spi_##idx##z_config,                                    \
					 POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                   \
					 &spi_nrfx_driver_api)
#endif

#ifdef CONFIG_SPI_0_NRF_SPIM_EXT_MUTEX
SPI_NRFX_SPIM_DEVICE(0);
#endif

#ifdef CONFIG_SPI_1_NRF_SPIM_EXT_MUTEX
SPI_NRFX_SPIM_DEVICE(1);
#endif

#ifdef CONFIG_SPI_2_NRF_SPIM_EXT_MUTEX
SPI_NRFX_SPIM_DEVICE(2);
#endif

#ifdef CONFIG_SPI_3_NRF_SPIM_EXT_MUTEX
SPI_NRFX_SPIM_DEVICE(3);
#endif

#ifdef CONFIG_SPI_4_NRF_SPIM_EXT_MUTEX
SPI_NRFX_SPIM_DEVICE(4);
#endif
