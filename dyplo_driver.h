/**
 * @file dyplo_driver.h
 *
 * @ingroup DyploDriver
 *
 * @brief RTEMS Dyplo Driver interface declaration.
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
#ifndef DYPLO_DRIVER_H
#define DYPLO_DRIVER_H

#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYPLO_DRIVER_TABLE_ENTRY \
  { dyplo_initialize, dyplo_open, dyplo_close, \
    dyplo_read, dyplo_write, dyplo_control }

/**
 *  @brief DYPLO Driver Initialization Entry Point
 *
 *  This method initializes the DYPLO device driver.
 *
 *  @param[in] major is the device driver major number
 *  @param[in] minor is the device driver minor number
 *  @param[in] arg is the parameters to this call
 *
 *  @return This method returns RTEMS_SUCCESSFUL when
 *          the device driver is successfully initialized.
 *          All other return values indicate an error
 *          occured.
 */
rtems_device_driver dyplo_initialize(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
);

/**
 *  @brief DYPLO Driver Open Entry Point
 *
 *  This method opens a specific device supported by the
 *  DYPLO device driver.
 *
 *  @param[in] major is the device driver major number
 *  @param[in] minor is the device driver minor number
 *  @param[in] arg is the parameters to this call
 *
 *  @return This method returns RTEMS_SUCCESSFUL when
 *          the device driver is successfully opened.
 *          All other return values indicate an error
 *          occured.
 */
rtems_device_driver dyplo_open(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
);

/**
 *  @brief DYPLO Driver Close Entry Point
 *
 *  This method closes a specific device supported by the
 *  DYPLO device driver.
 *
 *  @param[in] major is the device driver major number
 *  @param[in] minor is the device driver minor number
 *  @param[in] arg is the parameters to this call
 *
 *  @return This method returns RTEMS_SUCCESSFUL when
 *          the device is successfully closed.
 *          All other return values indicate an error
 *          occured.
 */
rtems_device_driver dyplo_close(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
);

/**
 *  @brief DYPLO Driver Read Entry Point
 *
 *  This method reads from a specific device supported by the
 *  DYPLO device driver.
 *
 *  @param[in] major is the device driver major number
 *  @param[in] minor is the device driver minor number
 *  @param[in] arg is the parameters to this call
 *
 *  @return This method returns RTEMS_SUCCESSFUL when
 *          the device is successfully read from.
 *          All other return values indicate an error
 *          occured.
 */
rtems_device_driver dyplo_read(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
);

/**
 *  @brief DYPLO Driver Write Entry Point
 *
 *  This method writes to a specific device supported by the
 *  DYPLO device driver.
 *
 *  @param[in] major is the device driver major number
 *  @param[in] minor is the device driver minor number
 *  @param[in] arg is the parameters to this call
 *
 *  @return This method returns RTEMS_SUCCESSFUL when
 *          the device is successfully written.
 *          All other return values indicate an error
 *          occured.
 */
rtems_device_driver dyplo_write(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
);

/**
 *  @brief DYPLO Driver IO Control Entry Point
 *
 *  This method performs an IO Control operation on a
 *  specific device supported by the DYPLO device driver.
 *
 *  @param[in] major is the device driver major number
 *  @param[in] minor is the device driver minor number
 *  @param[in] arg is the parameters to this call
 *
 *  @return This method returns RTEMS_SUCCESSFUL when
 *          the device driver IO control operation is
 *          successfully performed.
 *          All other return values indicate an error
 *          occured.
 */
rtems_device_driver dyplo_control(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
);

#ifdef __cplusplus
}

#endif

#endif // DYPLO_DRIVER_H
