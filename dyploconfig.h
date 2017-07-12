/**
 * @file dyploconfig.h
 *
 * @ingroup DyploDriver
 *
 * @brief Used to configure the Dyplo Driver.
 *
 * You can override the defines below to set-up the DyploDriver
 * configuration.
 *
 * Add your custom DyploDriver configuration to your application's "system.h"
 * and include this file.
 *
 */

/*
 * Copyright (c) 2013-2017 Topic Embedded Products B.V.
 *
 * This file is part of DyploDriver.
 * DyploDriver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DyploDriver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 *
 * You can contact Topic by electronic mail via info@topic.nl or via
 * paper mail at the following address: Postbus 440, 5680 AK Best, The Netherlands.
 *
 */

#ifndef DYPLO_CONFIG_H
#define DYPLO_CONFIG_H

/* configuration for Dyplo driver. Include this in your system.h */

struct dyplo_driver_configuration {
  void *base;
  int irq;
  unsigned int dma_memory_size;
  unsigned int dma_default_block_size;
};

extern const struct dyplo_driver_configuration dyplo_driver_init_configuration;

#ifdef CONFIGURE_INIT

/* Either the BSP must provide Dyplo support, or the user must provide it
 * through CONFIGURE_DYPLO_BASE_ADDR and CONFIGURE_DYPLO_IRQ */

#ifndef CONFIGURE_DYPLO_BASE_ADDR
#  include <bsp/dyplo.h>
#  define CONFIGURE_DYPLO_BASE_ADDR BSP_DYPLO_BASE_ADDR
#endif

#ifndef CONFIGURE_DYPLO_IRQ
#  include <bsp/dyplo.h>
#  define CONFIGURE_DYPLO_IRQ BSP_DYPLO_IRQ
#endif

/* Defaults when no value provided by user */

#ifndef CONFIGURE_DYPLO_DMA_MEMORY_SIZE
#       define CONFIGURE_DYPLO_DMA_MEMORY_SIZE ( 256 * 1024 )
#endif

#ifndef CONFIGURE_DYPLO_DMA_DEFAULT_BLOCK_SIZE
#       define CONFIGURE_DYPLO_DMA_DEFAULT_BLOCK_SIZE ( \
    CONFIGURE_DYPLO_DMA_MEMORY_SIZE >> 2 )
#endif

const struct dyplo_driver_configuration dyplo_driver_init_configuration = {
  .base = (void *) CONFIGURE_DYPLO_BASE_ADDR,
  .irq = CONFIGURE_DYPLO_IRQ,
  .dma_memory_size = CONFIGURE_DYPLO_DMA_MEMORY_SIZE,
  .dma_default_block_size = CONFIGURE_DYPLO_DMA_DEFAULT_BLOCK_SIZE,
};

#endif /* CONFIGURE_INIT */

#endif // DYPLO_CONFIG_H
