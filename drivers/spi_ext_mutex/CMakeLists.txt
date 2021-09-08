# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_NRFX_SPIM_EXT_MUTEX spi_nrfx_spim_ext_mutex.c)
