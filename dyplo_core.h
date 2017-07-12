/**
 * @file dyplo_core.h
 *
 * @ingroup DyploDriver
 *
 * @brief Declaration of main interfaces and structs of core DyploDriver implementation.
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

#ifndef DYPLO_CORE_H
#define DYPLO_CORE_H

#include <rtems.h>
#include <rtems/libio.h>
#include <semaphore.h>
#include <rtems/score/atomic.h>
#include "dyplo-ioctl.h"

/* Use DMA coherent memory. Depending on hardware HP/ACP, this may yield
 * non-cachable memory which is particularly noticeable on logic-to-cpu
 * unless you have hardware coherency (dma-coherent in DT). */
#define DYPLO_DMA_BLOCK_FLAG_COHERENT 1
/* Use streaming instead of coherent memory. This requires cacheline
* maintenance which may cost more than actually copying the data. */
#define DYPLO_DMA_BLOCK_FLAG_STREAMING 2
/* Indicates that the memory pointers point to a shared block and should
 * not be freed. */
#define DYPLO_DMA_BLOCK_FLAG_SHAREDMEM 4

#define ICAP_NOT_AVAILABLE ( (uint8_t) -1 )

/* Maximum number of commands, i.e. the size of the command queue in
 * logic. This is mostly dynamically used, but in some places, it's
 * good to know how far we can go. */
#define DMA_MAX_NUMBER_OF_COMMANDS 8

typedef uint32_t dma_addr_t;

typedef enum {
  DYPLO_DRIVER_TYPE_CTL,
  DYPLO_DRIVER_TYPE_CFG,
  DYPLO_DRIVER_TYPE_FIFO_READ,
  DYPLO_DRIVER_TYPE_FIFO_WRITE,
  DYPLO_DRIVER_TYPE_DMA
} dyplo_device_type;

enum dma_data_direction {
  DMA_BIDIRECTIONAL = 0,
  DMA_TO_DEVICE = 1,
  DMA_FROM_DEVICE = 2,
  DMA_NONE = 3,
};

typedef struct dyplo_dma_block {
  // Kernel part
  struct dyplo_dma_dev *parent;
  dma_addr_t phys_addr;
  void *mem_addr;
  // User part
  struct dyplo_buffer_block data;
} dyplo_dma_block;

typedef struct dyplo_dma_block_set {
  struct dyplo_dma_block *blocks;
  uint32_t size;
  uint32_t count;
  uint32_t flags;
} dyplo_dma_block_set;

typedef struct dyplo_dma_to_logic_operation {
  dma_addr_t addr;
  uint32_t size;
} dyplo_dma_to_logic_operation;

typedef struct dyplo_dma_from_logic_operation {
  char *addr;
  uint32_t size;
  uint32_t next_tail;
  uint16_t user_signal;
  uint16_t short_transfer;       // Non-zero if size < blocksize
} dyplo_dma_from_logic_operation;

#define DMA_FLAG_OPEN_MODE_READ LIBIO_FLAGS_READ
#define DMA_FLAG_OPEN_MODE_WRITE LIBIO_FLAGS_WRITE
#define DMA_FLAG_OPEN_MODE_EXCL 0x8000  // For standalone mode, exclusive access to the DMA node

typedef struct dyplo_dma_dev {
  struct dyplo_config_dev *config_parent;
  mode_t open_mode;       // possible flags: DMA_FLAG_OPEN_MODE_READ | DMA_FLAG_OPEN_MODE_WRITE | DMA_FLAG_OPEN_MODE_EXCL

  struct dyplo_dma_block_set dma_to_logic_blocks;
  struct dyplo_dma_block_set dma_from_logic_blocks;

  // big blocks of memory for read/write transfers
  dma_addr_t dma_to_logic_handle;
  void *dma_to_logic_memory;
  uint32_t dma_to_logic_memory_size;
  uint32_t dma_to_logic_head;
  uint32_t dma_to_logic_tail;
  uint32_t dma_to_logic_block_size;
  rtems_id write_to_logic_wait_semaphore;
  // FIFO for the dma to logic operations that are in progress
  struct dyplo_dma_to_logic_operation dma_to_logic_commands_in_progress[ 16 ];
  int dma_to_logic_commands_idx_first;
  int dma_to_logic_commands_count;

  void *dma_from_logic_memory;
  uint32_t dma_from_logic_memory_size;
  uint32_t dma_from_logic_head;
  uint32_t dma_from_logic_tail;
  uint32_t dma_from_logic_block_size;
  rtems_id read_from_logic_wait_semaphore;
  struct dyplo_dma_from_logic_operation dma_from_logic_current_op;
  bool dma_from_logic_full;

  // Protection for reading/writing from 2 different contexts:
  rtems_id read_semaphore;
  rtems_id write_semaphore;
  volatile CPU_atomic_Flag is_closing;
} dyplo_dma_dev;

typedef struct dyplo_fifo_dev {
  struct dyplo_config_dev *config_parent;
  uint32_t index;
  rtems_id wait_semaphore;       // So the IRQ handler can notify waiting threads
  uint32_t words_transfered;
  uint32_t poll_treshold;
  uint16_t user_signal;
  bool eof;
  bool is_open;

  // Protection for reading/writing from 2 different contexts:
  rtems_id read_write_semaphore;
  volatile CPU_atomic_Flag is_closing;
} dyplo_fifo_dev;

typedef struct dyplo_fifo_control_dev {
  struct dyplo_config_dev *config_parent;
  struct dyplo_fifo_dev *fifo_devices;
  uint8_t number_of_fifo_write_devices;
  uint8_t number_of_fifo_read_devices;
} dyplo_fifo_control_dev;

typedef struct dyplo_config_dev {
  struct dyplo_dev *parent;
  uint32_t *base;
  uint32_t *control_base;
  uint8_t node_type;
  uint8_t open_mode;
  void ( *isr )(
    struct dyplo_dev        *dev,
    struct dyplo_config_dev *cfg_dev
  );   // IRQ handler, if any
  void *private_data;
} dyplo_config_dev;

typedef struct dyplo_dev {
  rtems_device_major_number major;
  rtems_device_minor_number minor;
  rtems_id semaphore;       // for protecting access to dyplo_dev and all its subdevices
  uint32_t *base;
  uint32_t number_of_config_devices;
  struct dyplo_config_dev *config_devices;
  uint8_t count_fifo_write_devices;
  uint8_t count_fifo_read_devices;
  int irq;
  uint8_t number_of_dma_devices;
  uint8_t icap_device_index;
} dyplo_dev;

typedef bool (*func_dyplo_control_device_registered) ( struct dyplo_dev * );
typedef bool (*func_dyplo_config_device_registered) ( struct dyplo_config_dev * );
typedef bool (*func_dyplo_fifo_read_device_registered) ( struct dyplo_fifo_dev
  * );
typedef bool (*func_dyplo_fifo_write_device_registered) ( struct dyplo_fifo_dev
  * );
typedef bool (*func_dyplo_dma_device_registered) ( struct dyplo_dma_dev * );

// main interfaces for RTEMS driver:

// Errors that occur during Dyplo Driver initialization are fatal and will cause RTEMS to terminate.
int dyplo_core_probe(
  struct dyplo_dev                       *dev,
  func_dyplo_control_device_registered    func_register_ctrl_dev,
  func_dyplo_config_device_registered     func_register_cfg_dev,
  func_dyplo_fifo_read_device_registered  func_register_fifo_read_dev,
  func_dyplo_fifo_write_device_registered func_register_fifo_write_dev,
  func_dyplo_dma_device_registered        func_register_dma_dev
);
int dyplo_device_read( rtems_libio_rw_args_t *args );
int dyplo_device_write( rtems_libio_rw_args_t *args );
int dyplo_device_open( rtems_libio_open_close_args_t *args );
int dyplo_device_close( rtems_libio_open_close_args_t *args );
int dyplo_device_control( rtems_libio_ioctl_args_t *args );

#ifdef __cplusplus
// defined as "extern" for unit testing purposes
extern "C" {
#endif

int dma_command_fifo_get(
  struct dyplo_dma_dev                *dma_dev,
  struct dyplo_dma_to_logic_operation *op
);
int dma_command_fifo_put(
  struct dyplo_dma_dev               *dma_dev,
  struct dyplo_dma_to_logic_operation op
);
bool dma_command_fifo_is_empty( struct dyplo_dma_dev *dma_dev );
void dma_command_fifo_reset( struct dyplo_dma_dev *dma_dev );

#ifdef __cplusplus
}
#endif

#endif // DYPLO_CORE_H
