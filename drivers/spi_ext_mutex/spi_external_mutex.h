

#ifndef SPI_EXT_MUTEX_H_
#define SPI_EXT_MUTEX_H_

#include <zephyr.h>

/**
 * @brief Attempt to acquire external mutex
 *
 * @param dev The SPI device who's mutex is to be acquired
 * @return int 0 if successfull
 *             -EIO on pin configuration or set error
 *             -EAGAIN if timed out while waiting for signal pin
 *                     or waiting for SCLK if mode is slave
 */
int spi_ext_mutex_acquire(const struct device *dev);

/**
 * @brief Attempt to release external mutex
 *
 * @param dev The SPI device who's mutex is to be released
 * @return int 0 if successfull
 *             -EIO on pin configuration or set error
 */
int spi_ext_mutex_release(const struct device *dev);

#endif // SPI_EXT_MUTEX_H_
