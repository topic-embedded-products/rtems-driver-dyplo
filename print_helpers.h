/**
 * @file print_helpers.h
 *
 * @ingroup DyploDriver
 *
 * @brief Internally used in DyploDriver for printing messages to stdout/stderr.
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

#ifndef DYPLO_PRINT_HELPERS_H
#define DYPLO_PRINT_HELPERS_H

#include <rtems/bspIo.h>

#ifdef NDEBUG
// Don't do anything in release builds for the DEBUG print macro's
#define KERNEL_DEBUG_PRINT( fmt, args ... )
#define DEBUG_PRINT( fmt, args ... )
#else
#define DEBUG_PRINT( fmt, args ... ) /* printf("DEBUG: %s:%d:%s(): " fmt "\n",  __FILE__, __LINE__, __func__, ##args) */
#define KERNEL_DEBUG_PRINT( fmt, args ... ) /* printk("%s:%d:%s(): " fmt "\r\n", __FILE__, __LINE__, __func__, ##args) */
#endif

// use this for printing errors & info when the console driver has not yet been loaded  or in ISR's
#define KERNEL_INFO_PRINT( fmt, args ... ) printk( fmt "\r\n", ## args )
#define KERNEL_ERROR_PRINT( fmt, args ... ) printk( \
  "%s:%d:%s(): ERROR: " fmt "\r\n", \
  __FILE__, \
  __LINE__, \
  __func__, \
  ## args )

#define INFO_PRINT( fmt, args ... ) printf( fmt "\n", ## args )
#define ERROR_PRINT( fmt, args ... ) fprintf( stderr, \
  "%s:%d:%s(): ERROR:" fmt "\n", \
  __FILE__, \
  __LINE__, \
  __func__, \
  ## args )

#endif // DYPLO_PRINT_HELPERS_H
