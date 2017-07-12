/**
 * @file dyplo_driver.c
 *
 * @ingroup DyploDriver
 *
 * @brief RTEMS Dyplo Driver interface implementation.
 *
 * RTEMS Dyplo Driver interface, interfaces with the dyplo_core,
 * the main implementation of the driver.
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

#include <rtems/libio.h>
#include <stdio.h>
#include "dyplo_driver.h"
#include "dyplo_core.h"
#include "dyploconfig.h"
#include "print_helpers.h"
#include "dyplo.h"
#include "errno.h"
#include <assert.h>
#include <inttypes.h>

static dyplo_dev dyplo_device;

// counter used for registering subdevices with unique minor numbers
static int child_device_counter = 0;

static const char DRIVER_CONTROL_DEVICE_NAME[] = "/dev/dyploctl";
static const char DRIVER_CONFIG_DEVICE_NAME[] = "/dev/dyplocfg%d";
static const char DRIVER_FIFO_READ_DEVICE_NAME[] = "/dev/dyplor%d";
static const char DRIVER_FIFO_WRITE_DEVICE_NAME[] = "/dev/dyplow%d";
static const char DRIVER_DMA_DEVICE_NAME[] = "/dev/dyplod%d";

void dyplo_set_iop_data(
  rtems_device_major_number      major,
  rtems_device_minor_number      minor,
  rtems_libio_open_close_args_t *arg
);

bool dyplo_register_control_device( dyplo_dev *ctldev )
{
  KERNEL_DEBUG_PRINT( "Registering %s", DRIVER_CONTROL_DEVICE_NAME );
  rtems_status_code status = rtems_io_register_name(
    DRIVER_CONTROL_DEVICE_NAME,
    dyplo_device.major,
    child_device_counter++ );

  if ( status != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not register %s", DRIVER_CONTROL_DEVICE_NAME );
    rtems_fatal_error_occurred( status );
  }

  return true;
}

bool dyplo_register_config_device( dyplo_config_dev *cfgdev )
{
  static int config_device_counter = 0;

  char config_device_name[ 16 ];
  int  chars_written = snprintf( config_device_name,
    sizeof( config_device_name ),
    DRIVER_CONFIG_DEVICE_NAME,
    config_device_counter++ );

  assert( chars_written > 0 && chars_written < sizeof( config_device_name ) );
  KERNEL_DEBUG_PRINT( "Registering %s", config_device_name );
  rtems_status_code status = rtems_io_register_name( config_device_name,
    dyplo_device.major,
    child_device_counter++ );

  if ( status != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not register %s", config_device_name );
    rtems_fatal_error_occurred( status );
  }

  return true;
}

bool dyplo_register_fifo_read_device( dyplo_fifo_dev *fifodev )
{
  static int cpu_fifo_read_device_counter = 0;

  char fifo_device_name[ 16 ];
  int  chars_written = snprintf( fifo_device_name,
    sizeof( fifo_device_name ),
    DRIVER_FIFO_READ_DEVICE_NAME,
    cpu_fifo_read_device_counter++ );

  assert( chars_written > 0 && chars_written < sizeof( fifo_device_name ) );
  KERNEL_DEBUG_PRINT( "Registering %s, minor: %d",
    fifo_device_name,
    child_device_counter );
  rtems_status_code status = rtems_io_register_name( fifo_device_name,
    dyplo_device.major,
    child_device_counter++ );

  if ( status != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not register %s", fifo_device_name );
    rtems_fatal_error_occurred( status );
  }

  return true;
}

bool dyplo_register_fifo_write_device( dyplo_fifo_dev *fifodev )
{
  static int cpu_fifo_write_device_counter = 0;

  char fifo_device_name[ 16 ];
  int  chars_written = snprintf( fifo_device_name,
    sizeof( fifo_device_name ),
    DRIVER_FIFO_WRITE_DEVICE_NAME,
    cpu_fifo_write_device_counter++ );

  assert( chars_written > 0 && chars_written < sizeof( fifo_device_name ) );
  KERNEL_DEBUG_PRINT( "Registering %s, minor: %d",
    fifo_device_name,
    child_device_counter );
  rtems_status_code status = rtems_io_register_name( fifo_device_name,
    dyplo_device.major,
    child_device_counter++ );

  if ( status != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not register %s", fifo_device_name );
    rtems_fatal_error_occurred( status );
  }

  return true;
}

bool dyplo_register_dma_device( dyplo_dma_dev *dmadev )
{
  static int dma_device_counter = 0;

  char dma_device_name[ 16 ];
  int  chars_written = snprintf( dma_device_name,
    sizeof( dma_device_name ),
    DRIVER_DMA_DEVICE_NAME,
    dma_device_counter++ );

  assert( chars_written > 0 && chars_written < sizeof( dma_device_name ) );
  KERNEL_DEBUG_PRINT( "Registering %s, minor: %d",
    dma_device_name,
    child_device_counter );
  rtems_status_code status = rtems_io_register_name( dma_device_name,
    dyplo_device.major,
    child_device_counter++ );

  if ( status != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not register %s", dma_device_name );
    rtems_fatal_error_occurred( status );
  }

  return true;
}

/*
 *  Dyplo Device Driver Entry Points
 */
rtems_device_driver dyplo_initialize(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
)
{
  KERNEL_DEBUG_PRINT( "dyplo_initialize" );

  dyplo_device.major = major;
  dyplo_device.minor = minor;
  dyplo_device.count_fifo_read_devices = 0;
  dyplo_device.count_fifo_write_devices = 0;
  dyplo_device.number_of_config_devices = 0;
  dyplo_device.base = dyplo_driver_init_configuration.base;
  dyplo_device.irq = dyplo_driver_init_configuration.irq;

  // register all devices
  int res = dyplo_core_probe( &dyplo_device,
    &dyplo_register_control_device,
    &dyplo_register_config_device,
    &dyplo_register_fifo_read_device,
    &dyplo_register_fifo_write_device,
    &dyplo_register_dma_device );

  if ( res == 0 ) {
    KERNEL_INFO_PRINT( "Dyplo Driver Loaded" );

    return RTEMS_SUCCESSFUL;
  } else {
    KERNEL_ERROR_PRINT( "Initializing Dyplo Driver failed" );
  }

  return RTEMS_IO_ERROR;
}

rtems_device_driver dyplo_open(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
)
{
  DEBUG_PRINT( "dyplo_open. minor %" PRIu32, minor );
  rtems_libio_open_close_args_t *oc = (rtems_libio_open_close_args_t *) arg;

  if ( oc->iop->data1 == NULL ) {
    dyplo_set_iop_data( major, minor, oc );
  }

  int status = dyplo_device_open( oc );

  switch ( status ) {
    case 0:

      return RTEMS_SUCCESSFUL;
    case -EBUSY:

      return RTEMS_RESOURCE_IN_USE;
    case -EIO:
    case -EINVAL:

      return RTEMS_IO_ERROR;
    default:

      return RTEMS_IO_ERROR;
  }
}

rtems_device_driver dyplo_close(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
)
{
  DEBUG_PRINT( "dyplo_close, minor: %" PRIu32, minor );
  rtems_libio_open_close_args_t *oc = arg;

  int status = dyplo_device_close( oc );

  switch ( status ) {
    case 0:

      return RTEMS_SUCCESSFUL;
    default:

      return RTEMS_IO_ERROR;
  }
}

rtems_device_driver dyplo_write(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
)
{
  rtems_libio_rw_args_t *args = (rtems_libio_rw_args_t *) arg;
  int                    status = dyplo_device_write( args );

  switch ( status ) {
    case 0:

      return RTEMS_SUCCESSFUL;
    case -EBUSY:

      return RTEMS_RESOURCE_IN_USE;
    case -EAGAIN:

      return RTEMS_UNSATISFIED;
    case -EIO:
    case -EINVAL:

      return RTEMS_IO_ERROR;
    case -EINTR:

      return RTEMS_OBJECT_WAS_DELETED;
    default:

      if ( status < 0 ) {
        return RTEMS_IO_ERROR;
      }
  }

  return RTEMS_SUCCESSFUL;
}

rtems_device_driver dyplo_read(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
)
{
  rtems_libio_rw_args_t *args = (rtems_libio_rw_args_t *) arg;

  int status = dyplo_device_read( args );

  switch ( status ) {
    case 0:

      return RTEMS_SUCCESSFUL;
    case -EBUSY:

      return RTEMS_RESOURCE_IN_USE;
    case -EAGAIN:

      return RTEMS_UNSATISFIED;
    case -EIO:
    case -EINVAL:

      return RTEMS_IO_ERROR;
    case -EINTR:

      return RTEMS_OBJECT_WAS_DELETED;
    default:

      if ( status < 0 ) {
        return RTEMS_IO_ERROR;
      }
  }

  return RTEMS_SUCCESSFUL;
}

rtems_device_driver dyplo_control(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
)
{
  rtems_libio_ioctl_args_t *args = arg;

  DEBUG_PRINT( "dyplo_control. IOCTL: %d", (int) ( args->command ) );

  args->ioctl_return = dyplo_device_control( args );

  switch ( args->ioctl_return ) {
    case 0:

      return RTEMS_SUCCESSFUL;
    case -EBUSY:

      return RTEMS_RESOURCE_IN_USE;
    case -EAGAIN:

      return RTEMS_UNSATISFIED;
    case -EIO:
    case -EINVAL:

      return RTEMS_IO_ERROR;
    case -EINTR:

      return RTEMS_OBJECT_WAS_DELETED;
    case -EACCES:

      return RTEMS_IO_ERROR;
    default:

      if ( args->ioctl_return < 0 ) {
        return RTEMS_IO_ERROR;
      }
  }

  return RTEMS_SUCCESSFUL;
}

// link the IO device (iop) to the Dyplo Devices that belongs to it.
void dyplo_set_iop_data(
  rtems_device_major_number      major,
  rtems_device_minor_number      minor,
  rtems_libio_open_close_args_t *arg
)
{
  KERNEL_DEBUG_PRINT( "Set iop data for device with minor ID: %" PRIu32,
    minor );

  rtems_status_code sc = rtems_semaphore_obtain( dyplo_device.semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  if ( minor == 0 ) {
    arg->iop->data0 = DYPLO_DRIVER_TYPE_CTL;
    arg->iop->data1 = &dyplo_device;
    goto exit_ok;
  } else {
    // Find the correct device, based on the 'minor' number by looping through all the devices
    // in the order that they are registered.
    uint32_t device_counter = 1;

    for ( int cfg_index = 0;
          cfg_index < dyplo_device.number_of_config_devices;
          ++cfg_index ) {
      dyplo_config_dev *config_device =
        &( dyplo_device.config_devices[ cfg_index ] );

      if ( device_counter == minor ) {
        arg->iop->data0 = DYPLO_DRIVER_TYPE_CFG;
        arg->iop->data1 = config_device;
        goto exit_ok;
      }

      device_counter++;

      if ( config_device->node_type == DYPLO_TYPE_ID_TOPIC_CPU ) {
        dyplo_fifo_control_dev *fifo_ctl_dev = config_device->private_data;

        int fifo_idx;

        for ( fifo_idx = 0;
              fifo_idx < fifo_ctl_dev->number_of_fifo_write_devices;
              ++fifo_idx ) {
          if ( device_counter == minor ) {
            arg->iop->data0 = DYPLO_DRIVER_TYPE_FIFO_WRITE;
            arg->iop->data1 = &( fifo_ctl_dev->fifo_devices[ fifo_idx ] );
            goto exit_ok;
          }

          device_counter++;
        }

        for (;
             fifo_idx <
             fifo_ctl_dev->number_of_fifo_read_devices +
             fifo_ctl_dev->number_of_fifo_write_devices;
             ++fifo_idx ) {
          if ( device_counter == minor ) {
            arg->iop->data0 = DYPLO_DRIVER_TYPE_FIFO_READ;
            arg->iop->data1 = &( fifo_ctl_dev->fifo_devices[ fifo_idx ] );
            goto exit_ok;
          }

          device_counter++;
        }
      } else if ( config_device->node_type == DYPLO_TYPE_ID_TOPIC_DMA ) {
        if ( device_counter == minor ) {
          arg->iop->data0 = DYPLO_DRIVER_TYPE_DMA;
          arg->iop->data1 = config_device->private_data;
          goto exit_ok;
        }

        device_counter++;
      }
    }
  }

exit_ok:
  sc = rtems_semaphore_release( dyplo_device.semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  return;
}
