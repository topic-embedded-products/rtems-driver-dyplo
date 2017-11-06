/**
 * @file dyplo_core.c
 *
 * @ingroup DyploDriver
 *
 * @brief Core DyploDriver implementation
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <malloc.h>
#include <inttypes.h>
#include <fcntl.h>
#include <errno.h>

#include "dyplo.h"
#include "dyplo_core.h"
#include "dyplo-ioctl.h"
#include "dyploconfig.h"
#include "print_helpers.h"

#include <rtems/irq-extension.h>
#include <rtems/bspIo.h>
#include <rtems/endian.h>
#include <bsp.h>
#include <sys/libkern.h>
#include <sys/param.h>
#include <sys/poll.h>

inline uint32_t ioread32( uint32_t *addr )
{
  return *( (volatile uint32_t *) addr );
}

inline void iowrite32(
  uint32_t *addr,
  uint32_t  val
)
{
  *( (volatile uint32_t *) addr ) = val;
}

#define BIT( n ) ( 1 << ( n ) )

#define _IOC_NR( nr ) ( ( nr ) & 0xFF )
#define _IOC_TYPE( nr ) ( ( nr >> 8 ) & 0xFF )

/* What is to be passed to rtems_cache_coherent_allocate() as alignment. The
 * Dyplo DMA engine needs everything to be 64-bit aligned. */
#define DYPLO_DMA_MEMORY_ALIGNMENT 8

/* TODO: Move to BSP */
/* The memory wraps around every 512MB, so addresses 0x40000000 and 0xA0000000
 * refer to the same physical location in RAM. The Dyplo DMA controller sees
 * DDR RAM as starting at offset "0", so mask away the high order bits to
 * translate between CPU and DMA memory addressing. */
 #define DYPLO_DMA_ADDRESS_MASK 0x1FFFFFFF

/* Derive DMA address from (physical) CPU address. */
static inline dma_addr_t dyplo_cpu_to_dma( uintptr_t addr )
{
  return (dma_addr_t) ( addr & DYPLO_DMA_ADDRESS_MASK );
}

/* Derive CPU address from DMA address, given a CPU physical "base" address. */
static inline uintptr_t dyplo_dma_to_cpu(
  uintptr_t  cpu_base,
  dma_addr_t addr
)
{
  return (uintptr_t) addr |
         ( cpu_base & ~( (uintptr_t) DYPLO_DMA_ADDRESS_MASK ) );
}

/* When device takes ownership we need to flush out data first to make sure
 * it's present in external RAM */
static inline void dyplo_dma_sync_device_to_device(
  const void *addr,
  size_t      size
)
{
  rtems_cache_flush_multiple_data_lines( addr, size );
}

/* After device has sent out data, no action needed on our side, the hardware
 * will not modify the data so any cached data is still valid. */
static inline void dyplo_dma_sync_cpu_to_device(
  const void *addr,
  size_t      size
)
{
}

/* Before starting to fill, we need to invalidate the cache to prevent stale
 * data from the cache being written in case of write-back cache. Can be
 * omitted in case of write-through cache. */
static inline void dyplo_dma_sync_device_from_device(
  const void *addr,
  size_t      size
)
{
  rtems_cache_invalidate_multiple_data_lines( addr, size );
}

/* After device has filled the memory, we need to invalidate it (again) to
 * prevent reading stale cache data. */
static inline void dyplo_dma_sync_cpu_from_device(
  const void *addr,
  size_t      size
)
{
  rtems_cache_invalidate_multiple_data_lines( addr, size );
}

static inline void dyplo_reg_write(
  uint32_t *base,
  uint32_t  reg,
  uint32_t  value
)
{
  iowrite32( ( base + ( reg >> 2 ) ), value );
}

static inline void dyplo_reg_write_index(
  uint32_t *base,
  uint32_t  reg,
  uint32_t  index,
  uint32_t  value
)
{
  iowrite32( ( base + ( reg >> 2 ) + index ), value );
}

static inline uint32_t dyplo_reg_read_by_addr( uint32_t *addr )
{
  return ioread32( addr );
}

static inline uint32_t dyplo_reg_read(
  uint32_t *base,
  uint32_t  reg
)
{
  return ioread32( base + ( reg >> 2 ) );
}

static inline uint32_t dyplo_reg_read_index(
  uint32_t *base,
  uint32_t  reg,
  uint32_t  index
)
{
  return ioread32( base + ( reg >> 2 ) + index );
}

union dyplo_route_item_u {
  unsigned int route;
  dyplo_route_item_t route_item;
};

static uint32_t dyplo_ctl_get_dyplo_version_id( const dyplo_dev *dev )
{
  return dyplo_reg_read( dev->base, DYPLO_REG_CONTROL_DYPLO_VERSION );
}

// 0-based index of the config node
static unsigned int dyplo_get_config_index( const dyplo_config_dev *cfg_dev )
{
  return ( ( (char *) cfg_dev->base - (char *) cfg_dev->parent->base ) /
           DYPLO_CONFIG_SIZE ) - 1;
}

static uint32_t dyplo_cfg_get_version_id( const dyplo_config_dev *cfg_dev )
{
  return dyplo_reg_read( cfg_dev->control_base, DYPLO_REG_VERSION_ID );
}

static uint8_t dyplo_cfg_get_node_type( const dyplo_config_dev *cfg_dev )
{
  return ( dyplo_reg_read( cfg_dev->control_base,
             DYPLO_REG_TYPE_ID ) >> 8 ) & 0xFF;
}

static uint8_t dyplo_number_of_input_queues( const dyplo_config_dev *cfg_dev )
{
  return dyplo_reg_read( cfg_dev->control_base, DYPLO_REG_NODE_INFO ) & 0x0F;
}

static uint8_t dyplo_number_of_output_queues( const dyplo_config_dev *cfg_dev )
{
  return ( dyplo_reg_read( cfg_dev->control_base,
             DYPLO_REG_NODE_INFO ) >> 4 ) & 0x0F;
}

static void dyplo_ctl_route_remove_dst(
  dyplo_dev *dev,
  uint32_t   route
)
{
  int ctl_index;
  int queue_index;

  for ( ctl_index = 0; ctl_index < dev->number_of_config_devices;
        ++ctl_index ) {
    const int number_of_fifos = dyplo_number_of_output_queues(
      &dev->config_devices[ ctl_index ] );
    uint32_t *ctl_route_base = dev->base +
                               ( DYPLO_REG_CONTROL_ROUTE_TABLE >> 2 ) +
                               ( ctl_index << DYPLO_STREAM_ID_WIDTH );

    for ( queue_index = 0; queue_index < number_of_fifos; ++queue_index ) {
      if ( dyplo_reg_read_by_addr( ctl_route_base + queue_index ) == route ) {
        DEBUG_PRINT( "removed route %d,%d->%lu,%lu",
          ctl_index, queue_index,
          ( route >> DYPLO_STREAM_ID_WIDTH ) - 1,
          route & ( ( 0x1 << DYPLO_STREAM_ID_WIDTH ) - 1 ) );
        iowrite32( ctl_route_base + queue_index, 0 );
      }
    }
  }
}

static int dyplo_ctl_route_add(
  dyplo_dev          *dev,
  dyplo_route_item_t *route
)
{
  uint32_t *dst_control_addr;
  uint32_t  dst_route;

  DEBUG_PRINT( "%d,%d->%d,%d",
    route->srcNode,
    route->srcFifo,
    route->dstNode,
    route->dstFifo );

  if ( ( route->srcNode >= dev->number_of_config_devices ) ||
       ( route->dstNode >= dev->number_of_config_devices ) ) {
    DEBUG_PRINT( "Invalid source or destination" );

    return -EINVAL;
  }

  dst_route = ( ( route->dstNode + 1 ) << DYPLO_STREAM_ID_WIDTH ) |
              route->dstFifo;
  dyplo_ctl_route_remove_dst( dev, dst_route );

  /* Setup route. The PL assumes that "0" is the control node, hence
   * the "+1" in config node indices */
  dst_control_addr = dev->base +
                     ( DYPLO_REG_CONTROL_ROUTE_TABLE >> 2 ) +
                     ( route->srcNode << DYPLO_STREAM_ID_WIDTH ) +
                     route->srcFifo;
  DEBUG_PRINT( "(%d) @ %p: %" PRIx32 "",
    route->srcNode,
    dst_control_addr,
    dst_route );

  iowrite32( dst_control_addr, dst_route );

  return 0;
}

static int dyplo_ctl_routes_add(
  dyplo_dev           *dev,
  const dyplo_route_t *routes
)
{
  int          status = 0;
  unsigned int numroutes = routes->n_routes;

  for ( int i = 0; i < numroutes; ++i ) {
    status = dyplo_ctl_route_add( dev, &( routes->proutes[ i ] ) );

    if ( status )
      break;
  }

  return status;
}

static int dyplo_ctl_route_get(
  dyplo_dev     *dev,
  dyplo_route_t *routes
)
{
  int status = 0;
  int nr = 0;
  int src_node;
  int src_fifo;

  for ( src_node = 0; src_node < dev->number_of_config_devices; ++src_node ) {
    uint32_t *ctl_route_base = dev->base +
                               ( DYPLO_REG_CONTROL_ROUTE_TABLE >> 2 ) +
                               ( src_node << DYPLO_STREAM_ID_WIDTH );
    const int number_of_fifos =
      dyplo_number_of_output_queues( &( dev->config_devices[ src_node ] ) );

    for ( src_fifo = 0; src_fifo < number_of_fifos; ++src_fifo ) {
      uint32_t route = ioread32( ctl_route_base + src_fifo );

      if ( route ) {
        int dst_node = route >> DYPLO_STREAM_ID_WIDTH;

        if ( dst_node > 0 ) {
          int dst_fifo = route & ( ( 0x1 << DYPLO_STREAM_ID_WIDTH ) - 1 );

          if ( nr >= routes->n_routes )
            return nr;                                     // No room for more, quit

          routes->proutes[ nr ].srcNode = src_node;
          routes->proutes[ nr ].srcFifo = src_fifo;
          routes->proutes[ nr ].dstNode = dst_node - 1;
          routes->proutes[ nr ].dstFifo = dst_fifo;

          DEBUG_PRINT( "cfg=%d 0x%" PRIx32 " @ %p",
            src_node,
            route,
            ctl_route_base + src_fifo );
          ++nr;
        }
      }
    }
  }

  return status ? status : nr;       // Return number of items found
}

static int dyplo_ctl_route_delete(
  dyplo_dev *dev,
  int        ctl_index_to_delete
)
{
  int       queue_index;
  int       ctl_index;
  const int match = ( ctl_index_to_delete + 1 ) << DYPLO_STREAM_ID_WIDTH;
  const int number_of_fifos = dyplo_number_of_output_queues(
    &dev->config_devices[ ctl_index_to_delete ] );
  uint32_t *ctl_route_base = dev->base +
                             ( DYPLO_REG_CONTROL_ROUTE_TABLE >> 2 ) +
                             ( ctl_index_to_delete << DYPLO_STREAM_ID_WIDTH );

  // Erase outgoing routes
  for ( queue_index = 0; queue_index < number_of_fifos; ++queue_index )
    iowrite32( ctl_route_base + queue_index, 0 );

  // Erase incoming routes
  for ( ctl_index = 0; ctl_index < ctl_index_to_delete; ++ctl_index ) {
    const int number_of_fifos =
      dyplo_number_of_output_queues( &( dev->config_devices[ ctl_index ] ) );
    ctl_route_base = dev->base +
                     ( DYPLO_REG_CONTROL_ROUTE_TABLE >> 2 ) +
                     ( ctl_index << DYPLO_STREAM_ID_WIDTH );

    for ( queue_index = 0; queue_index < number_of_fifos; ++queue_index ) {
      if ( ( ioread32( ctl_route_base + queue_index ) &
             ( 0xFFFF << DYPLO_STREAM_ID_WIDTH ) ) == match ) {
        iowrite32( ctl_route_base + queue_index, 0 );
      }
    }
  }

  for ( ctl_index = ctl_index_to_delete + 1;
        ctl_index < dev->number_of_config_devices;
        ++ctl_index ) {
    const int number_of_fifos =
      dyplo_number_of_output_queues( &( dev->config_devices[ ctl_index ] ) );
    ctl_route_base = dev->base +
                     ( DYPLO_REG_CONTROL_ROUTE_TABLE >> 2 ) +
                     ( ctl_index << DYPLO_STREAM_ID_WIDTH );

    for ( queue_index = 0; queue_index < number_of_fifos; ++queue_index ) {
      if ( ( ioread32( ctl_route_base + queue_index ) &
             ( 0xFFFF << DYPLO_STREAM_ID_WIDTH ) ) == match ) {
        iowrite32( ctl_route_base + queue_index, 0 );
      }
    }
  }

  return 0;
}

static int dyplo_ctl_route_clear( dyplo_dev *dev )
{
  int ctl_index;
  int queue_index;

  uint32_t *ctl_route_base = dev->base +
                             ( DYPLO_REG_CONTROL_ROUTE_TABLE >> 2 );

  for ( ctl_index = 0; ctl_index < dev->number_of_config_devices;
        ++ctl_index ) {
    // Remove outgoing routes
    const int number_of_fifos =
      dyplo_number_of_output_queues( &( dev->config_devices[ ctl_index ] ) );

    for ( queue_index = 0; queue_index < number_of_fifos; ++queue_index ) {
      iowrite32( ctl_route_base + queue_index, 0 );
    }

    ctl_route_base += ( 1 << DYPLO_STREAM_ID_WIDTH );
  }

  return 0;
}

static long dyplo_ctl_license_key(
  dyplo_dev      *dev,
  ioctl_command_t cmd,
  void           *arg_key
)
{
  uint32_t *key = (uint32_t *) arg_key;
  const int expected_license_key_length = sizeof( uint64_t );

  if ( IOCPARM_LEN( cmd ) != expected_license_key_length )
    return -EINVAL;

  if ( ( cmd & IOC_DIRMASK ) & IOC_IN ) {
    // Already checked memory with access_ok
    dyplo_reg_write( dev->base, DYPLO_REG_CONTROL_LICENSE_KEY0, key[ 0 ] );
    dyplo_reg_write( dev->base, DYPLO_REG_CONTROL_LICENSE_KEY1, key[ 1 ] );
  }

  if ( ( cmd & IOC_DIRMASK ) & IOC_OUT ) {
    key[ 0 ] = dyplo_reg_read( dev->base, DYPLO_REG_CONTROL_LICENSE_KEY0 );
    key[ 1 ] = dyplo_reg_read( dev->base, DYPLO_REG_CONTROL_LICENSE_KEY1 );
  }

  return 0;
}

static ssize_t dyplo_generic_read(
  uint32_t *mapped_memory,
  char     *buf,
  size_t    count,
  off_t    *f_pos
)
{
  DEBUG_PRINT( "generic read @ %p", mapped_memory );

  size_t offset;

  uint32_t *wbuf = (uint32_t *) buf;

  // EOF when past our area
  if ( *f_pos >= DYPLO_CONFIG_SIZE )
    return 0;

  if ( count == 0 )
    return 0;

  if ( count < 4 )     // Do not allow read or write below word size
    return -EINVAL;

  // add offset/fpos
  offset = ( (size_t) *f_pos ) & ~0x03;    // Align to word size
  count &= ~0x03;

  if ( ( offset + count ) > DYPLO_CONFIG_SIZE )
    count = DYPLO_CONFIG_SIZE - offset;

  mapped_memory += ( offset >> 2 );

  uint32_t words_to_transfer = count >> 2;

  for (; words_to_transfer != 0; --words_to_transfer ) {
    *wbuf = ioread32( mapped_memory );
    ++wbuf;
    ++mapped_memory;
  }

  *f_pos = offset + count;

  return (ssize_t) count;
}

static ssize_t dyplo_generic_write(
  uint32_t   *mapped_memory,
  const char *buf,
  uint32_t    count,
  off_t      *f_pos
)
{
  DEBUG_PRINT( "generic write @ %p", mapped_memory );

  size_t          offset;
  const uint32_t *wbuf = (uint32_t *) buf;

  // EOF when past our area
  if ( *f_pos >= DYPLO_CONFIG_SIZE )
    return 0;

  if ( count == 0 )
    return 0;

  if ( count < 4 )     // Do not allow read or write below word size
    return -EINVAL;

  offset = ( (size_t) *f_pos ) & ~0x03;    // Align to word size
  count &= ~0x03;

  if ( ( offset + count ) > DYPLO_CONFIG_SIZE )
    count = DYPLO_CONFIG_SIZE - offset;

  mapped_memory += ( offset >> 2 );

  uint32_t words_to_transfer = count >> 2;

  for (; words_to_transfer != 0; --words_to_transfer ) {
    iowrite32( mapped_memory, *wbuf );
    ++wbuf;
    ++mapped_memory;
  }

  *f_pos = offset + count;

  return (ssize_t) count;
}

static int dyplo_ctl_open( rtems_libio_open_close_args_t *args )
{
  return 0;
}

static int dyplo_ctl_release( rtems_libio_open_close_args_t *args )
{
  return 0;
}

// Interrupt service routine for CPU fifo node, version 2
static void dyplo_fifo_isr(
  dyplo_dev        *dev,
  dyplo_config_dev *cfg_dev
)
{
  dyplo_fifo_control_dev *fifo_ctl_dev =
    (dyplo_fifo_control_dev *) cfg_dev->private_data;
  uint32_t status_reg = dyplo_reg_read( cfg_dev->control_base,
    DYPLO_REG_FIFO_IRQ_STATUS );

  // Acknowledge interrupt to hardware
  iowrite32( cfg_dev->control_base + ( DYPLO_REG_FIFO_IRQ_CLR >> 2 ),
    status_reg );

  uint16_t read_status_reg;
  uint16_t write_status_reg;
  uint8_t  index;

  // Trigger the associated wait queues, "read" queues first. These
  // are in the upper 16 bits of the interrupt status word
  read_status_reg = status_reg >> 16;

  for ( index =
          0;
        ( read_status_reg != 0 ) &&
        ( index < fifo_ctl_dev->number_of_fifo_read_devices );
        ++index ) {
    if ( read_status_reg & 1 ) {
      rtems_semaphore_release( fifo_ctl_dev->fifo_devices[ fifo_ctl_dev->
                                                           number_of_fifo_write_devices
                                                           + index ].wait_semaphore );
    }

    read_status_reg >>= 1;
  }

  write_status_reg = status_reg & 0xFFFF;

  for ( index =
          0;
        ( write_status_reg != 0 ) &&
        ( index < fifo_ctl_dev->number_of_fifo_write_devices );
        ++index ) {
    if ( write_status_reg & 1 ) {
      rtems_semaphore_release(
        fifo_ctl_dev->fifo_devices[ index ].wait_semaphore );
    }

    write_status_reg >>= 1;
  }
}

static unsigned int dyplo_dma_get_index( const dyplo_dma_dev *dma_dev )
{
  return dyplo_get_config_index( dma_dev->config_parent );
}

static void dyplo_dma_to_logic_irq_enable( uint32_t *control_base )
{
  DEBUG_PRINT( "" );
  iowrite32( control_base + ( DYPLO_REG_FIFO_IRQ_SET >> 2 ), BIT( 0 ) );
}

static void dyplo_dma_from_logic_irq_enable( uint32_t *control_base )
{
  DEBUG_PRINT( "" );
  iowrite32( control_base + ( DYPLO_REG_FIFO_IRQ_SET >> 2 ), BIT( 16 ) );
}

static void dyplo_set_bits(
  uint32_t *reg_ptr,
  uint32_t  mask,
  bool      enable
)
{
  uint32_t value = ioread32( reg_ptr );
  uint32_t next = enable ? ( value | mask ) : ( value & ~mask );

  if ( next != value )
    iowrite32( reg_ptr, next );
}

static void dyplo_dma_from_logic_enable(
  uint32_t *control_base,
  bool      value
)
{
  dyplo_set_bits( control_base + ( DYPLO_DMA_FROMLOGIC_CONTROL >> 2 ),
   BIT( 0 ), value );
}

static void dyplo_dma_to_logic_enable(
  uint32_t *control_base,
  bool      value
)
{
  dyplo_set_bits( control_base + ( DYPLO_DMA_TOLOGIC_CONTROL >> 2 ), BIT(
      0 ), value );
}

static bool dyplo_dma_common_is_standalone_mode( dyplo_dma_dev *dma_dev )
{
  return ( dyplo_reg_read( dma_dev->config_parent->control_base,
             DYPLO_DMA_STANDALONE_CONTROL ) & BIT( 0 ) );
}

// Utilities for fifo functions
static uint32_t *dyplo_fifo_memory_location( dyplo_fifo_dev *fifo_dev )
{
  dyplo_config_dev *cfg_dev = fifo_dev->config_parent;

  return cfg_dev->base + ( fifo_dev->index * ( DYPLO_FIFO_MEMORY_SIZE >> 2 ) );
}

static bool dyplo_fifo_write_usersignal(
  dyplo_fifo_dev *fifo_dev,
  uint16_t        user_signal
)
{
  uint32_t *control_base_user_signal = fifo_dev->config_parent->control_base +
                                       ( DYPLO_REG_FIFO_WRITE_USERSIGNAL_BASE
                                         >> 2 ) +
                                       fifo_dev->index;

  iowrite32( control_base_user_signal, user_signal );

  // Test if user signals are supported by reading back the value
  return ( (uint16_t) ioread32( control_base_user_signal ) ) == user_signal;
}

static uint32_t dyplo_fifo_read_level( dyplo_fifo_dev *fifo_dev )
{
  return dyplo_reg_read_index(
    fifo_dev->config_parent->control_base,
    DYPLO_REG_FIFO_READ_LEVEL_BASE,
    fifo_dev->index );
}

static void dyplo_fifo_read_enable_interrupt(
  dyplo_fifo_dev *fifo_dev,
  uint32_t        thd
)
{
  int index = fifo_dev->index;

  uint32_t *control_base = fifo_dev->config_parent->control_base;

  if ( thd > ( DYPLO_FIFO_READ_SIZE * 2 ) / 4 ) {
    thd = ( DYPLO_FIFO_READ_SIZE * 2 ) / 4;
  } else if ( thd ) {
    --thd;             // Treshold of "15" will alert when 16 words are present in the FIFO
  }

  iowrite32( control_base + ( DYPLO_REG_FIFO_READ_THD_BASE >> 2 ) + index,
    thd );
  /* v2 uses upper 16 bits of shared IRQ registers */
  DEBUG_PRINT( "index=%d thd=%" PRIu32 " v2", index, thd );

  iowrite32( control_base + ( DYPLO_REG_FIFO_IRQ_SET >> 2 ),
    BIT( index + 16 ) );
}

int dma_command_fifo_get(
  dyplo_dma_dev                *dma_dev,
  dyplo_dma_to_logic_operation *op
)
{
  if ( dma_dev->dma_to_logic_commands_count > 0 ) {
    *op =
      dma_dev->dma_to_logic_commands_in_progress[ dma_dev->
                                                  dma_to_logic_commands_idx_first
      ];

    --dma_dev->dma_to_logic_commands_count;

    // update position
    if ( dma_dev->dma_to_logic_commands_count > 0 ) {
      int arraysize = RTEMS_ARRAY_SIZE(
        dma_dev->dma_to_logic_commands_in_progress );
      int max_idx = arraysize - 1;

      if ( dma_dev->dma_to_logic_commands_idx_first < max_idx ) {
        dma_dev->dma_to_logic_commands_idx_first++;
      } else {
        // wrap around
        dma_dev->dma_to_logic_commands_idx_first = 0;
      }
    }

    return 1;
  } else {
    return 0;
  }
}

int dma_command_fifo_put(
  dyplo_dma_dev               *dma_dev,
  dyplo_dma_to_logic_operation op
)
{
  int arraysize =
    RTEMS_ARRAY_SIZE( dma_dev->dma_to_logic_commands_in_progress );

  if ( dma_dev->dma_to_logic_commands_count < arraysize ) {
    int max_idx = arraysize - 1;

    // calculate tail
    int dma_to_logic_wip_idx_tail =
      dma_dev->dma_to_logic_commands_idx_first +
      dma_dev->dma_to_logic_commands_count;

    if ( dma_to_logic_wip_idx_tail > max_idx ) {
      // wrap around
      dma_to_logic_wip_idx_tail -= arraysize;
    }

    dma_dev->dma_to_logic_commands_in_progress[ dma_to_logic_wip_idx_tail ] =
      op;
    dma_dev->dma_to_logic_commands_count++;

    return 1;
  }

  // no more space
  return 0;
}

bool dma_command_fifo_is_empty( dyplo_dma_dev *dma_dev )
{
  return dma_dev->dma_to_logic_commands_count == 0;
}

void dma_command_fifo_reset( dyplo_dma_dev *dma_dev )
{
  dma_dev->dma_to_logic_commands_count = 0;
}

// Kills ongoing DMA transactions and resets everything.
static int dyplo_dma_to_logic_reset( dyplo_dma_dev *dma_dev )
{
  uint32_t *control_base = dma_dev->config_parent->control_base;
  uint32_t  reg;
  int       result = 0;

  reg = dyplo_reg_read( control_base, DYPLO_DMA_TOLOGIC_CONTROL );
  DEBUG_PRINT( "ctl=0x%" PRIx32, reg );

  if ( reg & BIT( 1 ) ) {
    ERROR_PRINT( "Reset already in progress" );

    return -EBUSY;
  }

  if ( !( reg & BIT( 0 ) ) ) {
    DEBUG_PRINT( "DMA hardware not running" );

    return -EINVAL;
  }

  reg |= BIT( 1 );
  // Enable reset-ready-interrupt
  iowrite32( control_base + ( DYPLO_REG_FIFO_IRQ_SET >> 2 ), BIT( 15 ) );
  // Send reset command
  iowrite32( control_base + ( DYPLO_DMA_TOLOGIC_CONTROL >> 2 ), reg );

  for (;; ) {
    if ( ( dyplo_reg_read( control_base,
             DYPLO_DMA_TOLOGIC_CONTROL ) & BIT( 1 ) ) == 0 ) {
      result = 0;
      break;
    }

    rtems_interval    ticksIn250ms = rtems_clock_get_ticks_per_second() / 4;
    rtems_status_code sc = rtems_semaphore_obtain(
      dma_dev->write_to_logic_wait_semaphore,
      RTEMS_WAIT,
      ticksIn250ms );

    if ( sc == RTEMS_TIMEOUT ) {
      DEBUG_PRINT( "TIMEOUT waiting for reset complete IRQ" );
      result = -ETIMEDOUT;
      break;
    }
  }

  // Re-enable the node
  iowrite32( control_base + ( DYPLO_DMA_TOLOGIC_CONTROL >> 2 ), BIT( 0 ) );
  dma_dev->dma_to_logic_head = 0;
  dma_dev->dma_to_logic_tail = 0;

  dma_command_fifo_reset( dma_dev );

  return result;
}

static int dyplo_dma_from_logic_reset( dyplo_dma_dev *dma_dev )
{
  uint32_t *control_base = dma_dev->config_parent->control_base;
  uint32_t  reg;
  int       result;

  reg = dyplo_reg_read( control_base, DYPLO_DMA_FROMLOGIC_CONTROL );
  DEBUG_PRINT( "ctl=0x%" PRIx32, reg );

  if ( reg & BIT( 1 ) ) {
    ERROR_PRINT( "Reset already in progress" );

    return -EBUSY;
  }

  if ( !( reg & BIT( 0 ) ) ) {
    ERROR_PRINT( "DMA hardware not running" );

    return -EINVAL;
  }

  reg |= BIT( 1 );
  // Enable reset-ready-interrupt
  iowrite32( control_base + ( DYPLO_REG_FIFO_IRQ_SET >> 2 ), BIT( 31 ) );
  // Send reset command
  iowrite32( control_base + ( DYPLO_DMA_FROMLOGIC_CONTROL >> 2 ),
    BIT( 1 ) | BIT( 0 ) );

  for (;; ) {
    if ( ( dyplo_reg_read( control_base,
             DYPLO_DMA_FROMLOGIC_CONTROL ) & BIT( 1 ) ) == 0 ) {
      result = 0;
      break;
    }

    rtems_status_code sc = rtems_semaphore_obtain(
      dma_dev->read_from_logic_wait_semaphore,
      RTEMS_WAIT,
      rtems_clock_get_ticks_per_second() );

    if ( sc == RTEMS_TIMEOUT ) {
      DEBUG_PRINT( "TIMEOUT waiting for reset complete IRQ" );
      result = -ETIMEDOUT;
      break;
    }
  }

  // Re-enable the node
  iowrite32( control_base + ( DYPLO_DMA_FROMLOGIC_CONTROL >> 2 ), BIT( 0 ) );

  dma_dev->dma_from_logic_head = 0;
  dma_dev->dma_from_logic_tail = 0;
  dma_dev->dma_from_logic_current_op.size = 0;
  dma_dev->dma_from_logic_full = false;

  return result;
}

static void dyplo_dma_common_set_standalone_mode(
  dyplo_dma_dev *dma_dev,
  bool           standalone
)
{
  uint32_t *control_base = dma_dev->config_parent->control_base;

  if ( dyplo_dma_common_is_standalone_mode( dma_dev ) == standalone )
    return;             // Already in that mode

  DEBUG_PRINT( "Switching to %s mode",
    standalone ? "standalone" : "normal" );
  dyplo_dma_to_logic_enable( control_base, false );
  dyplo_dma_from_logic_enable( control_base, false );
  iowrite32( control_base + ( DYPLO_DMA_STANDALONE_CONTROL >> 2 ),
    standalone ? BIT( 0 ) : 0 );

  if ( !standalone ) {
    dyplo_dma_to_logic_enable( control_base, true );
    dyplo_dma_to_logic_reset( dma_dev );
    dyplo_dma_from_logic_enable( control_base, true );
    dyplo_dma_from_logic_reset( dma_dev );
  }
}

static int dyplo_dma_open( rtems_libio_open_close_args_t *args )
{
  dyplo_dma_dev    *dma_dev = (dyplo_dma_dev *) args->iop->data1;
  dyplo_config_dev *cfg_dev = dma_dev->config_parent;
  dyplo_dev        *dev = cfg_dev->parent;
  int               status = 0;

  DEBUG_PRINT( "(mode=%" PRIx32 " flags=%" PRIx32 ")", args->mode,
    args->flags );

  // Must specify either read or write mode
  if ( ( args->flags & ( LIBIO_FLAGS_READ | LIBIO_FLAGS_WRITE ) ) == 0 )
    return -EINVAL;

  rtems_status_code sc = rtems_semaphore_obtain( dev->semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  if ( args->flags & LIBIO_FLAGS_WRITE ) {
    if ( dma_dev->open_mode & DMA_FLAG_OPEN_MODE_WRITE ) {
      status = -EBUSY;
      goto exit_open;
    }

    // Normally, RDWR means just write. If O_SYNC is specified, the
    // intention is to use standalone mode, which needs both ends
    if ( args->mode & O_SYNC ) {
      if ( dma_dev->open_mode & DMA_FLAG_OPEN_MODE_READ ) {
        status = -EBUSY;
        goto exit_open;
      }

      dma_dev->open_mode |=
        ( DMA_FLAG_OPEN_MODE_WRITE | DMA_FLAG_OPEN_MODE_READ |
          DMA_FLAG_OPEN_MODE_EXCL );
    } else {
	  dma_dev->open_mode |= DMA_FLAG_OPEN_MODE_WRITE; // Set in-use bits
	  args->iop->flags &= ~LIBIO_FLAGS_READ; // Set flags to non-reading. RDWR is interpreted as Write.
    }

    // Reset usersignal
    iowrite32( cfg_dev->control_base + ( DYPLO_DMA_TOLOGIC_USERBITS >> 2 ),
      DYPLO_USERSIGNAL_ZERO );
    // Default to generic size
    dma_dev->dma_to_logic_block_size =
	  dyplo_driver_init_configuration.dma_default_block_size;
  } else {
    if ( dma_dev->open_mode & DMA_FLAG_OPEN_MODE_READ ) {
      status = -EBUSY;
      goto exit_open;
    }

	dma_dev->open_mode |= DMA_FLAG_OPEN_MODE_READ;  // Set in-use bits
  }

exit_open:
  sc = rtems_semaphore_release( dev->semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  return status;
}

static void dyplo_dma_common_stop_read_write( dyplo_dma_dev *dma_dev )
{
  rtems_status_code sc;

  dma_dev->is_closing = true;

  sc = rtems_semaphore_release( dma_dev->read_from_logic_wait_semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  sc = rtems_semaphore_release( dma_dev->write_to_logic_wait_semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  // wait till read is completed
  sc = rtems_semaphore_obtain( dma_dev->read_semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );
  // wait till write is completed
  sc = rtems_semaphore_obtain( dma_dev->write_semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  // after obtaining both semaphores, it is time to release them
  sc = rtems_semaphore_release( dma_dev->read_semaphore );
  assert( sc == RTEMS_SUCCESSFUL );
  sc = rtems_semaphore_release( dma_dev->write_semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  dma_dev->is_closing = false;
}

static int dyplo_dma_release( rtems_libio_open_close_args_t *args )
{
  dyplo_dma_dev *dma_dev = (dyplo_dma_dev *) args->iop->data1;
  dyplo_dev     *dev = dma_dev->config_parent->parent;

  rtems_status_code sc = rtems_semaphore_obtain( dev->semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );

  assert( sc == RTEMS_SUCCESSFUL );

  dyplo_dma_common_stop_read_write( dma_dev );

  if ( dma_dev->open_mode & DMA_FLAG_OPEN_MODE_EXCL ) {
    dyplo_dma_common_set_standalone_mode( dma_dev, false );
    dma_dev->open_mode = 0;             // Closes both ends
  } else {
    dma_dev->open_mode &= ~( args->iop->flags ); // Clear in use bit
  }

  sc = rtems_semaphore_release( dev->semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  return 0;
}

// CPU and DMA shouldn't be accessing the same cache line simultaneously.
// Align the head pointer for DMA transfers to cache line.
static unsigned int round_up_to_cacheline( unsigned int value )
{
  size_t data_cache_line_size = rtems_cache_get_data_line_size();

  if ( data_cache_line_size == 0 ) {
    // If we cannot retrieve the data cache line size, use PAGE_SIZE

    // to align the head pointer for DMA transfers.
    return ( value + ( PAGE_SIZE - 1 ) ) & ( ~( PAGE_SIZE - 1 ) );
  } else {
    return ( value + ( data_cache_line_size - 1 ) ) &
           ( ~( data_cache_line_size - 1 ) );
  }
}

static unsigned int dyplo_dma_to_logic_avail( dyplo_dma_dev *dma_dev )
{
  uint32_t *control_base = dma_dev->config_parent->control_base;
  uint32_t  status = dyplo_reg_read( control_base, DYPLO_DMA_TOLOGIC_STATUS );
  // Status: bits 24..31: nr. of commands that returned result
  // 16..23:				nr. of commands in the command buffer
  uint8_t num_results;

  // Fetch result from queue
  dyplo_dma_to_logic_operation op;

  op.addr = 0;
  op.size = 0;

  DEBUG_PRINT( "status=0x%" PRIx32, status );

  for ( num_results = ( status >> 24 ); num_results != 0; --num_results ) {
    uint32_t addr =
      dyplo_reg_read( control_base, DYPLO_DMA_TOLOGIC_RESULT_ADDR );

    if ( !dma_command_fifo_get( dma_dev, &op ) ) {
      ERROR_PRINT(
        "Bug detected in Dyplo. Nothing in fifo of DMA node %u but still %u results",
        dyplo_dma_get_index( dma_dev ),
        num_results );
      rtems_fatal_error_occurred( RTEMS_IO_ERROR );
    }

    DEBUG_PRINT( "addr=0x%" PRIx32 " wip=0x%" PRIx32 ",%" PRIu32,
      addr,
      op.addr,
      op.size );

    if ( op.addr != addr ) {
      ERROR_PRINT(
        "Bug detected in Dyplo. Mismatch in result of DMA node %u: phys=%pa expected 0x%" PRIx32 " (size %" PRIu32 ") actual 0x%" PRIx32,
        dyplo_dma_get_index( dma_dev ),
        &dma_dev->dma_to_logic_handle,
        (uint32_t) op.addr,
        op.size,
        addr );
      ERROR_PRINT( "Additional information below:" );
      ERROR_PRINT(
        "head=0x%" PRIx32 " (%" PRIu32 ") tail=0x%" PRIx32 " (%" PRIu32 ")",
        dma_dev->dma_to_logic_head,
        dma_dev->dma_to_logic_head,
        dma_dev->dma_to_logic_tail,
        dma_dev->dma_to_logic_tail );

      for (;; ) {
        if ( !dma_command_fifo_get( dma_dev, &op ) )
          break;

        ERROR_PRINT( "Internal entry: 0x%" PRIx32 " (size %" PRIu32 ")",
          op.addr,
          op.size );
      }

      while ( num_results ) {
        addr = dyplo_reg_read( control_base, DYPLO_DMA_TOLOGIC_RESULT_ADDR );
        ERROR_PRINT( "Logic result: 0x%" PRIx32, addr );
        --num_results;
      }

      rtems_fatal_error_occurred( RTEMS_IO_ERROR );
    }

    dma_dev->dma_to_logic_tail += round_up_to_cacheline( op.size );

    if ( dma_dev->dma_to_logic_tail == dma_dev->dma_to_logic_memory_size )
      dma_dev->dma_to_logic_tail = 0;

    DEBUG_PRINT( "tail=%" PRIu32, dma_dev->dma_to_logic_tail );

    if ( dma_dev->dma_to_logic_tail > dma_dev->dma_to_logic_memory_size ) {
      ERROR_PRINT(
        "Bug detected in Dyplo. Overflow in DMA node %u: tail %" PRIu32 " size %" PRIu32,
        dyplo_dma_get_index( dma_dev ),
        dma_dev->dma_to_logic_tail,
        dma_dev->dma_to_logic_memory_size );
      rtems_fatal_error_occurred( RTEMS_IO_ERROR );
    }
  }

  // Calculate available space
  if ( dma_dev->dma_to_logic_tail > dma_dev->dma_to_logic_head ) {
    return dma_dev->dma_to_logic_tail - dma_dev->dma_to_logic_head;
  } else if ( dma_dev->dma_to_logic_tail == dma_dev->dma_to_logic_head ) {
    // Can mean "full" or "empty"
    if ( !dma_command_fifo_is_empty( dma_dev ) )
      return 0;                   // head==tail and there is work in progress
  }

  // Return available bytes until end of buffer
  return dma_dev->dma_to_logic_memory_size - dma_dev->dma_to_logic_head;
}

// Two things may block: There's no room in the ring, or there's no room
// in the command buffer.
static ssize_t dyplo_dma_write( rtems_libio_rw_args_t *args )
{
  dyplo_dma_dev *dma_dev = (dyplo_dma_dev *) args->iop->data1;

  if ( dma_dev->is_closing ) {
    return -EINTR;
  }

  // IMPORTANT: do not exit this function before the semaphore is released
  // (which is done at the end of the function).
  rtems_status_code sc = rtems_semaphore_obtain( dma_dev->write_semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  int       status = 0;
  uint32_t *control_base =
    dma_dev->config_parent->control_base;
  unsigned int                 bytes_to_copy = 0;
  unsigned int                 bytes_avail = 0;
  const char                  *buf = (const char *) args->buffer;
  uint32_t                     count = args->count;
  dyplo_dma_to_logic_operation dma_op;
  const bool                   is_blocking =
    ( args->iop->flags & LIBIO_FLAGS_NO_DELAY ) == 0;

  DEBUG_PRINT( "count: %" PRIu32 " bytes", count );

  if ( ( count % DYPLO_DMA_MEMORY_ALIGNMENT ) != 0 ) {
    ERROR_PRINT( "DMA write needs to be %d-byte aligned",
      DYPLO_DMA_MEMORY_ALIGNMENT );
    status = -EINVAL;
    goto error_exit;
  }

  if ( dma_dev->dma_to_logic_blocks.blocks ) {
    status = -EBUSY;
    goto error_exit;
  }

  while ( count && !dma_dev->is_closing ) {
    bytes_to_copy =
      min( (unsigned int) count, dma_dev->dma_to_logic_block_size );
    DEBUG_PRINT( "bytes to copy: %u", bytes_to_copy );

    while ( !dma_dev->is_closing ) {
      bytes_avail = dyplo_dma_to_logic_avail( dma_dev );
      DEBUG_PRINT( "bytes_avail=%u head=%" PRIu32 " tail=%" PRIu32,
        bytes_avail, dma_dev->dma_to_logic_head, dma_dev->dma_to_logic_tail );

      if ( bytes_avail != 0 )
        break;

      // Enable interrupt
      dyplo_dma_to_logic_irq_enable( control_base );

      if ( is_blocking ) {
        if ( !dma_dev->is_closing ) {
          rtems_status_code sc = rtems_semaphore_obtain(
            dma_dev->write_to_logic_wait_semaphore,
            RTEMS_WAIT,
            RTEMS_NO_TIMEOUT );
          assert( sc == RTEMS_SUCCESSFUL );
        }
      } else {
        if ( args->bytes_moved ) {
          goto exit_ok;        // Some data transferred
        } else {
          // No data available, tell user
          status = -EAGAIN;
          goto error_exit;
        }
      }
    }

    if ( dma_dev->is_closing ) {
      goto closing;
    }

    if ( bytes_avail < bytes_to_copy )
      bytes_to_copy = bytes_avail;

    // Copy data into DMA buffer...
    DEBUG_PRINT( "memcpy %p to %p", buf,
      ( dma_dev->dma_to_logic_memory + dma_dev->dma_to_logic_head ) );

    memcpy( dma_dev->dma_to_logic_memory + dma_dev->dma_to_logic_head,
      buf,
      bytes_to_copy );

#ifdef DYPLO_NO_COHERENT_MEMORY
    /* Transfer memory ownership to the device */
    dyplo_dma_sync_device_to_device(
      dma_dev->dma_to_logic_memory + dma_dev->dma_to_logic_head,
      bytes_to_copy );
#endif

    // Submit command to engine, wait for availability first
    dma_op.addr = dyplo_cpu_to_dma(
      (uintptr_t) dma_dev->dma_to_logic_memory + dma_dev->dma_to_logic_head );
    dma_op.size = bytes_to_copy;

    while ( !dma_dev->is_closing ) {
      if ( dyplo_reg_read( control_base,
             DYPLO_DMA_TOLOGIC_STATUS ) & 0xFF0000 )
        break;        // There is room in the command buffer

      // Enable interrupt
      dyplo_dma_to_logic_irq_enable( control_base );

      if ( is_blocking ) {
        if ( !dma_dev->is_closing ) {
          rtems_status_code sc = rtems_semaphore_obtain(
            dma_dev->write_to_logic_wait_semaphore,
            RTEMS_WAIT,
            RTEMS_NO_TIMEOUT );
          assert( sc == RTEMS_SUCCESSFUL );
        }
      } else {
        if ( args->bytes_moved ) {
          goto exit_ok;   // Some data transferred
        } else {
          // No data available, tell user
          status = -EAGAIN;
          goto error_exit;
        }
      }
    }

    if ( dma_dev->is_closing ) {
      goto closing;
    }

    DEBUG_PRINT( "sending addr=0x%" PRIx32 " size=%" PRIu32 " TO addr %p",
      dma_op.addr, dma_op.size,
      control_base + ( DYPLO_DMA_TOLOGIC_STARTADDR >> 2 ) );

    iowrite32( control_base + ( DYPLO_DMA_TOLOGIC_STARTADDR >> 2 ),
      dma_op.addr );
    iowrite32( control_base + ( DYPLO_DMA_TOLOGIC_BYTESIZE >> 2 ),
      dma_op.size );

    if ( dma_command_fifo_put( dma_dev, dma_op ) == 0 ) {
      ERROR_PRINT(
        "Bug detected in Dyplo. dma_to_logic_commands_in_progress was full, cannot put 0x%" PRIu32 " %" PRIu32,
        dma_op.addr,
        dma_op.size );
      rtems_fatal_error_occurred( RTEMS_IO_ERROR );
    }

    // Update pointers for next chunk, if any
    dma_dev->dma_to_logic_head += round_up_to_cacheline( bytes_to_copy );

    if ( dma_dev->dma_to_logic_head == dma_dev->dma_to_logic_memory_size )
      dma_dev->dma_to_logic_head = 0;

    DEBUG_PRINT( "head=%" PRIu32, dma_dev->dma_to_logic_head );
    args->bytes_moved += bytes_to_copy;
    buf += bytes_to_copy;
    count -= bytes_to_copy;
  }

exit_ok:
  status = args->bytes_moved;
closing:

  if ( dma_dev->is_closing ) {
    status = -EINTR;
  }

error_exit:
  DEBUG_PRINT( "STATUS: %d", status );
  rtems_semaphore_release( dma_dev->write_semaphore );

  return status;
}

// Adds new read commands to the queue and returns number of results
static unsigned int dyplo_dma_from_logic_pump( dyplo_dma_dev *dma_dev )
{
  uint32_t *control_base = dma_dev->config_parent->control_base;
  uint32_t  status_reg;
  uint8_t   num_free_entries;

  status_reg = dyplo_reg_read( control_base, DYPLO_DMA_FROMLOGIC_STATUS );
  DEBUG_PRINT( "status=0x%" PRIx32, status_reg );
  num_free_entries = ( status_reg >> 16 ) & 0xFF;

  DEBUG_PRINT( "num free command entries=%" PRIu8, num_free_entries );

  while ( !dma_dev->dma_from_logic_full && !dma_dev->is_closing ) {
    if ( !num_free_entries )
      break;                   // No more room for commands

    dma_addr_t start_addr =
      dyplo_cpu_to_dma(
        (uintptr_t) dma_dev->dma_from_logic_memory +
        dma_dev->dma_from_logic_head );
    DEBUG_PRINT( "sending addr=0x%" PRIx32 " size=%" PRIu32,
      start_addr, dma_dev->dma_from_logic_block_size );
    iowrite32( control_base + ( DYPLO_DMA_FROMLOGIC_STARTADDR >> 2 ),
      start_addr );
    iowrite32( control_base + ( DYPLO_DMA_FROMLOGIC_BYTESIZE >> 2 ),
      dma_dev->dma_from_logic_block_size );
    dma_dev->dma_from_logic_head += dma_dev->dma_from_logic_block_size;

    if ( dma_dev->dma_from_logic_head == dma_dev->dma_from_logic_memory_size )
      dma_dev->dma_from_logic_head = 0;

    if ( dma_dev->dma_from_logic_head == dma_dev->dma_from_logic_tail )
      dma_dev->dma_from_logic_full = true;

    --num_free_entries;
  }

  return status_reg >> 24;
}

static ssize_t dyplo_dma_read( rtems_libio_rw_args_t *args )
{
  dyplo_dma_dev *dma_dev = args->iop->data1;

  if ( dma_dev->is_closing ) {
    return -EINTR;
  }

  // IMPORTANT: do not exit this function before the semaphore is released
  // (which is done at the end of the function).
  rtems_status_code sc = rtems_semaphore_obtain( dma_dev->read_semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  uint32_t *control_base =
    dma_dev->config_parent->control_base;
  int                             status = 0;
  unsigned int                    bytes_to_copy;
  unsigned int                    results_avail = 0;
  dyplo_dma_from_logic_operation *current_op =
    &dma_dev->dma_from_logic_current_op;
  char      *buf = (char *) args->buffer;
  uint32_t   count = args->count;
  const bool is_blocking = ( args->iop->flags & LIBIO_FLAGS_NO_DELAY ) == 0;

  DEBUG_PRINT( "count=%" PRIu32, count );

  if ( ( count % DYPLO_DMA_MEMORY_ALIGNMENT ) != 0 ) {
    ERROR_PRINT( "DMA read needs to be %d-byte aligned",
      DYPLO_DMA_MEMORY_ALIGNMENT );
    status = -EINVAL;
    goto error_exit;
  }

  if ( dma_dev->dma_from_logic_blocks.blocks ) {
    status = -EBUSY;
    goto error_exit;
  }

  while ( count && !dma_dev->is_closing ) {
    while ( ( current_op->size == 0 ) && ( !dma_dev->is_closing ) ) {
      // Fetch a new operation from logic
      if ( results_avail ) {
        uintptr_t start_addr = dyplo_dma_to_cpu(
          (uintptr_t) dma_dev->dma_from_logic_memory,
          dyplo_reg_read( control_base, DYPLO_DMA_FROMLOGIC_RESULT_ADDR ) );
        DEBUG_PRINT( "Results available. start_addr: %" PRIxPTR, start_addr );

        current_op->addr = (char *) start_addr;
        current_op->user_signal = (uint16_t) dyplo_reg_read( control_base,
          DYPLO_DMA_FROMLOGIC_RESULT_USERBITS );
        current_op->size = dyplo_reg_read( control_base,
          DYPLO_DMA_FROMLOGIC_RESULT_BYTESIZE );
        current_op->short_transfer =
          ( current_op->size != dma_dev->dma_from_logic_block_size );

        DEBUG_PRINT( "Addr: %p, size: %" PRIu32 ", short transfer: %d",
          current_op->addr,
          current_op->size,
          current_op->short_transfer );

        uint32_t tail = start_addr -
                        (uintptr_t) dma_dev->dma_from_logic_memory;
        tail += dma_dev->dma_from_logic_block_size;

        if ( tail == dma_dev->dma_from_logic_memory_size )
          tail = 0;

        current_op->next_tail = tail;
        --results_avail;
        DEBUG_PRINT( "nexttail=%" PRIu32 " size=%" PRIu32 " addr=%p",
          tail, current_op->size, current_op->addr );
      } else {
        while ( !dma_dev->is_closing ) {
          results_avail = dyplo_dma_from_logic_pump( dma_dev );
          DEBUG_PRINT( "results_avail=%u head=%" PRIu32 " tail=%" PRIu32,
            results_avail,
            dma_dev->dma_from_logic_head,
            dma_dev->dma_from_logic_tail );

          if ( results_avail != 0 )
            break;

          // Enable interrupt
          dyplo_dma_from_logic_irq_enable( control_base );

          if ( is_blocking ) {
            if ( !dma_dev->is_closing ) {
              rtems_status_code sc = rtems_semaphore_obtain(
                dma_dev->read_from_logic_wait_semaphore,
                RTEMS_WAIT,
                RTEMS_NO_TIMEOUT );
              assert( sc == RTEMS_SUCCESSFUL );
            }
          } else {
            if ( args->bytes_moved ) {
              goto exit_ok;                                           // Some data transferred
            } else {
              // No data available, tell user
              status = -EAGAIN;
              goto error_exit;
            }
          }
        }
      }
    }

    if ( dma_dev->is_closing ) {
      goto closing;
    }

    // Copy any remaining data into the user's buffer
    if ( current_op->size ) {
      bytes_to_copy = current_op->size;

      if ( bytes_to_copy > count )
        bytes_to_copy = count;

      DEBUG_PRINT( "copy to user %p (%u)", current_op->addr, bytes_to_copy );

#ifdef DYPLO_NO_COHERENT_MEMORY
      /* CPU takes ownership of memory buffer */
      dyplo_dma_sync_cpu_from_device( current_op->addr, bytes_to_copy );
#endif

      memcpy( buf, current_op->addr, bytes_to_copy );

      args->bytes_moved += bytes_to_copy;
      count -= bytes_to_copy;
      buf += bytes_to_copy;
      current_op->size -= bytes_to_copy;

      if ( current_op->size != 0 ) {
        // No more room in user buffer
        current_op->addr += bytes_to_copy;
        break;
      } else {
        dma_dev->dma_from_logic_tail = current_op->next_tail;
        dma_dev->dma_from_logic_full = false;
        DEBUG_PRINT( "move tail %" PRIu32, dma_dev->dma_from_logic_tail );
        // We moved the tail up, so submit more work to logic
        results_avail = dyplo_dma_from_logic_pump( dma_dev );

        if ( current_op->short_transfer )
          break;                               // Usersignal change, return immediately
      }
    }
  }

exit_ok:
  status = args->bytes_moved;
closing:

  if ( dma_dev->is_closing ) {
    status = -EINTR;
  }

error_exit:
  rtems_semaphore_release( dma_dev->read_semaphore );

  return status;
}

rtems_interval timeval_to_rtems_interval( struct timeval *time )
{
  rtems_interval result = 0;

  if ( time != NULL ) {
	if ( time->tv_sec > 0 || time->tv_usec > 0 ) {
		rtems_interval ticks_per_second = rtems_clock_get_ticks_per_second();

		if ( time->tv_usec > 0 ) {
		  const double microseconds_in_second = 1000000.0f;
		  double       ticks_per_microsecond = ticks_per_second /
											   microseconds_in_second;
		  result += (rtems_interval) ( ticks_per_microsecond * time->tv_usec );
		}

		result += ( time->tv_sec * ticks_per_second );
		DEBUG_PRINT( "RTEMS_INTERVAL: %" PRIu32, result );
	}
  }

  return result;
}

static unsigned int dyplo_dma_to_logic_poll_get_avail( dyplo_dma_dev *dma_dev )
{
  uint32_t    *control_base = dma_dev->config_parent->control_base;
  unsigned int avail;

  if ( dma_dev->dma_to_logic_blocks.blocks ) {
    // Writable when not all blocks have been submitted, or when
    // results are available and can be dequeued
    avail = dyplo_reg_read( control_base, DYPLO_DMA_TOLOGIC_STATUS );

    if ( ( avail & 0xFF000000 ) == 0 ) {
      // No results yet, see if there are blocks available
      avail = ( ( avail >> 16 ) & 0xFF ) + dma_dev->dma_to_logic_blocks.count -
              DMA_MAX_NUMBER_OF_COMMANDS;
    }
  } else {
    avail = dyplo_dma_to_logic_avail( dma_dev );
  }

  return avail;
}

static int dyplo_dma_to_logic_poll(
  dyplo_dma_dev  *dma_dev,
  struct timeval *timeout
)
{
  if (timeout != NULL)
	DEBUG_PRINT( "timeout: %d secs", (int) timeout->tv_sec );
  else
	DEBUG_PRINT( "" );

  int          mask = 0;
  unsigned int avail = dyplo_dma_to_logic_poll_get_avail( dma_dev );
  uint32_t    *control_base = dma_dev->config_parent->control_base;

  if ( avail ) {
    mask |= ( POLLOUT | POLLWRNORM );
  } else {
	if ( timeout == NULL || ( timeout->tv_sec > 0 || timeout->tv_usec > 0 ) ) {
	  if ( !dma_dev->is_closing ) {
		dyplo_dma_to_logic_irq_enable( control_base );

		rtems_status_code sc = rtems_semaphore_obtain(
		  dma_dev->write_to_logic_wait_semaphore,
		  RTEMS_WAIT,
		  timeval_to_rtems_interval( timeout ) );
		assert( sc == RTEMS_SUCCESSFUL || sc == RTEMS_TIMEOUT );

		avail = dyplo_dma_to_logic_poll_get_avail( dma_dev );

		if ( avail ) {
		  mask |= ( POLLOUT | POLLWRNORM );
		} else {
		  if ( sc == RTEMS_SUCCESSFUL && !dma_dev->is_closing ) {
			/* apparently the semaphore was released normally, but there is no available data yet.
			  possibly the semaphore was in a state where semaphore count was at '1' (so obtaining it
			  did not block).. try again. */

			rtems_status_code sc = rtems_semaphore_obtain(
			  dma_dev->write_to_logic_wait_semaphore,
			  RTEMS_WAIT,
			  timeval_to_rtems_interval( timeout ) );
			assert( sc == RTEMS_SUCCESSFUL || sc == RTEMS_TIMEOUT );

			avail = dyplo_dma_to_logic_poll_get_avail( dma_dev );

			if ( avail ) {
			  mask |= ( POLLOUT | POLLWRNORM );
			}
		  }
		}
	  }
    }
  }

  DEBUG_PRINT( "(%#x) -> %#x\n", avail, mask );

  return mask;
}

static unsigned int dyplo_dma_from_logic_poll_get_avail( dyplo_dma_dev *dma_dev )
{
  unsigned int avail;

  uint32_t *control_base = dma_dev->config_parent->control_base;

  if ( dma_dev->dma_from_logic_blocks.blocks ) {
    avail = dyplo_reg_read( control_base, DYPLO_DMA_FROMLOGIC_STATUS );
    DEBUG_PRINT( "(status=%#x)", avail );
    avail &= 0xFF000000;
  } else {
    if ( dma_dev->dma_from_logic_current_op.size )
      avail = 1;
    else
      avail = dyplo_dma_from_logic_pump( dma_dev );
  }

  return avail;
}

static int dyplo_dma_from_logic_poll(
  dyplo_dma_dev  *dma_dev,
  struct timeval *timeout
)
{
  if ( timeout != NULL)
	DEBUG_PRINT( "timeout: %d secs", (int) timeout->tv_sec );
  else
	DEBUG_PRINT( "" );

  int          mask = 0;
  unsigned int avail = dyplo_dma_from_logic_poll_get_avail( dma_dev );
  uint32_t    *control_base = dma_dev->config_parent->control_base;

  if ( avail ) {
    mask |= ( POLLIN | POLLRDNORM );
  } else {
	if ( timeout == NULL || ( timeout->tv_sec > 0 || timeout->tv_usec > 0 ) ) {
	  if ( !dma_dev->is_closing ) {
		dyplo_dma_from_logic_irq_enable( control_base );

		rtems_status_code sc = rtems_semaphore_obtain(
		  dma_dev->read_from_logic_wait_semaphore,
		  RTEMS_WAIT,
		  timeval_to_rtems_interval( timeout ) );
		assert( sc == RTEMS_SUCCESSFUL || sc == RTEMS_TIMEOUT );

		avail = dyplo_dma_from_logic_poll_get_avail( dma_dev );

		if ( avail ) {
		  mask |= ( POLLIN | POLLRDNORM );
		} else {
		  if ( sc == RTEMS_SUCCESSFUL && !dma_dev->is_closing ) {
			/* apparently the semaphore was released normally, but there is no available space yet.
			  possibly the semaphore was in a state where semaphore count was at '1' (so obtaining it
			  did not block).. try again. */
			rtems_status_code sc = rtems_semaphore_obtain(
			  dma_dev->read_from_logic_wait_semaphore,
			  RTEMS_WAIT,
			  timeval_to_rtems_interval( timeout ) );
			assert( sc == RTEMS_SUCCESSFUL || sc == RTEMS_TIMEOUT );

			avail = dyplo_dma_from_logic_poll_get_avail( dma_dev );

			if ( avail ) {
			  mask |= ( POLLIN | POLLRDNORM );
			}
		  }
		}
	  }
    }
  }

  DEBUG_PRINT( "(%x) -> %#x\n", avail, mask );

  return mask;
}

static int dyplo_dma_add_route(
  dyplo_dma_dev *dma_dev,
  int            source,
  int            dest
)
{
  dyplo_route_item_t route;

  route.srcFifo = ( source >> 8 ) & 0xFF;
  route.srcNode = source & 0xFF;
  route.dstFifo = ( dest >> 8 ) & 0xFF;
  route.dstNode = dest & 0xFF;
  dyplo_ctl_route_add( dma_dev->config_parent->parent, &route );

  return 0;
}

static int dyplo_dma_get_route_id( dyplo_dma_dev *dma_dev )
{

  // Only one fifo, so upper 8 bits are always 0
  return dyplo_dma_get_index( dma_dev );
}

static void dyplo_dma_common_block_free_streaming(
  dyplo_dev           *dev,
  dyplo_dma_block_set *dma_block_set
)
{
  uint32_t i;

  for ( i = 0; i < dma_block_set->count; ++i ) {
    dyplo_dma_block *block = &dma_block_set->blocks[ i ];

    if ( block->mem_addr ) {
      free( block->mem_addr );
    }
  }
}

static void dyplo_dma_common_block_free_coherent(
  dyplo_dev           *dev,
  dyplo_dma_block_set *dma_block_set
)
{
  uint32_t i;

  for ( i = 0; i < dma_block_set->count; ++i ) {
    dyplo_dma_block *block = &dma_block_set->blocks[ i ];

    if ( block->mem_addr ) {
      rtems_cache_coherent_free( block->mem_addr );
    }
  }
}

static int dyplo_dma_common_block_free(
  dyplo_dma_dev       *dma_dev,
  dyplo_dma_block_set *dma_block_set
)
{
  if ( !( dma_block_set->flags & DYPLO_DMA_BLOCK_FLAG_SHAREDMEM ) ) {
    if ( dma_block_set->flags & DYPLO_DMA_BLOCK_FLAG_STREAMING )
      dyplo_dma_common_block_free_streaming( dma_dev->config_parent->parent,
        dma_block_set );
    else
      dyplo_dma_common_block_free_coherent( dma_dev->config_parent->parent,
        dma_block_set );
  }

  free( dma_block_set->blocks );
  dma_block_set->blocks = NULL;
  dma_block_set->count = 0;
  dma_block_set->size = 0;
  dma_block_set->flags = 0;

  return 0;
}

static int dyplo_dma_to_logic_block_free( dyplo_dma_dev *dma_dev )
{
  // Reset the device to release all resources
  if ( !dyplo_dma_common_is_standalone_mode( dma_dev ) )
    dyplo_dma_to_logic_reset( dma_dev );

  return dyplo_dma_common_block_free( dma_dev, &dma_dev->dma_to_logic_blocks );
}

static int dyplo_dma_from_logic_block_free( dyplo_dma_dev *dma_dev )
{
  // Reset the device to release all resources
  dyplo_dma_from_logic_reset( dma_dev );

  return dyplo_dma_common_block_free( dma_dev,
    &dma_dev->dma_from_logic_blocks );
}

static int dyplo_dma_common_block_alloc_one_coherent( dyplo_dma_block *block )
{
  block->mem_addr = rtems_cache_coherent_allocate( block->data.size,
    DYPLO_DMA_MEMORY_ALIGNMENT,
    0 );

  if ( block->mem_addr == NULL ) {
    ERROR_PRINT( "Could not allocate memory" );

    return -ENOMEM;
  }

  block->phys_addr = dyplo_cpu_to_dma( (uintptr_t) block->mem_addr );
  block->data.offset = (uintptr_t) block->mem_addr;

  return 0;
}

static int dyplo_dma_common_block_alloc_one_streaming( dyplo_dma_block *block )
{
  // TODO: rtems_cache_aligned_malloc?
  block->mem_addr = malloc( block->data.size );

  if ( block->mem_addr == NULL ) {
    ERROR_PRINT( "Could not allocate memory" );

    return -ENOMEM;
  }

  block->phys_addr = dyplo_cpu_to_dma( (uintptr_t) block->mem_addr );
  block->data.offset = (uintptr_t) block->mem_addr;

  return 0;
}

static int dyplo_dma_common_block_alloc(
  dyplo_dma_dev               *dma_dev,
  dyplo_dma_configuration_req *request,
  dyplo_dma_block_set         *dma_block_set,
  enum dma_data_direction      direction
)
{
  dyplo_dma_block *block;
  uint32_t         i;
  int              ret;

  DEBUG_PRINT( "mode=%" PRIu32 " count=%" PRIu32 " size=%" PRIu32,
    request->mode, request->count, request->size );

  if ( !request->size || !request->count )
    return -EINVAL;

  // Pointless to use more
  if ( request->count > DMA_MAX_NUMBER_OF_COMMANDS )
    request->count = DMA_MAX_NUMBER_OF_COMMANDS;

  block = calloc( request->count, sizeof( *block ) );

  if ( block == NULL ) {
    ERROR_PRINT( "Could not allocate blocks" );

    return -ENOMEM;
  }

  dma_block_set->blocks = block;
  dma_block_set->size = request->size;
  dma_block_set->count = request->count;
  dma_block_set->flags = ( request->mode == DYPLO_DMA_MODE_BLOCK_STREAMING ) ?
                         DYPLO_DMA_BLOCK_FLAG_STREAMING :
                         DYPLO_DMA_BLOCK_FLAG_COHERENT;

  if ( request->mode == DYPLO_DMA_MODE_BLOCK_COHERENT ) {
    // The pre-allocated buffers are coherent, so if the blocks fit
    // in there, we can just re-use the already allocated one
    if ( direction == DMA_FROM_DEVICE ) {
      if ( request->count * request->size <=
           dma_dev->dma_from_logic_memory_size ) {
        dma_block_set->flags |= DYPLO_DMA_BLOCK_FLAG_SHAREDMEM;

        for ( i = 0; i < request->count; ++i, ++block ) {
          block->data.id = i;
          block->data.size = request->size;
          block->mem_addr = ( (char *) dma_dev->dma_from_logic_memory ) +
                            ( i * request->size );
          block->phys_addr = dyplo_cpu_to_dma( (uintptr_t) block->mem_addr );
          // on RTEMS no virtual memory is used, so the offset field is used to store the memory address
          block->data.offset = (uintptr_t) block->mem_addr;
          DEBUG_PRINT(
            "ID: %" PRIu32 ", mem_addr: %p, offset: %" PRIx32  ", phys addr: 0x%" PRIx32,
            block->data.id,
            block->mem_addr,
            block->data.offset,
            block->phys_addr );
        }

        return 0;
      }
    } else {
      if ( request->count * request->size <=
           dma_dev->dma_to_logic_memory_size ) {
        dma_block_set->flags |= DYPLO_DMA_BLOCK_FLAG_SHAREDMEM;

        for ( i = 0; i < request->count; ++i, ++block ) {
          block->data.id = i;
          block->data.size = request->size;
          block->mem_addr = ( (char *) dma_dev->dma_to_logic_memory ) +
                            ( i * request->size );
          block->phys_addr = dyplo_cpu_to_dma( (uintptr_t) block->mem_addr );
          // on RTEMS no virtual memory is used, so the offset field is used to store the memory address
          block->data.offset = (uintptr_t) block->mem_addr;
          DEBUG_PRINT(
            "ID: %" PRIu32 ", mem_addr: %p, offset: %" PRIx32  ", phys addr: 0x%" PRIx32,
            block->data.id,
            block->mem_addr,
            block->data.offset,
            block->phys_addr );
        }

        return 0;
      }
    }
  }

  // in case the mode is DYPLO_DMA_MODE_BLOCK_STREAMING, or the buffer is not large enough,
  // allocate some memory for the buffer:
  for ( i = 0; i < request->count; ++i, ++block ) {
    block->data.id = i;
    block->data.size = request->size;
    block->data.offset = i * request->size;

    if ( dma_block_set->flags & DYPLO_DMA_BLOCK_FLAG_STREAMING )
      ret = dyplo_dma_common_block_alloc_one_streaming( block );
    else
      ret = dyplo_dma_common_block_alloc_one_coherent( block );

    if ( ret ) {
      dyplo_dma_common_block_free( dma_dev, dma_block_set );

      return ret;
    }
  }

  return 0;
}

static int dyplo_dma_standalone_block_alloc(
  dyplo_dma_dev *dma_dev,
  uint32_t       size,
  uint32_t       count
)
{
  uint32_t *control_base =
    dma_dev->config_parent->control_base;
  dyplo_dma_configuration_req request;
  int                         ret;

  if ( count > 255 )
    return -EINVAL;

  // Needs a contiguous single block.
  request.mode = DYPLO_DMA_MODE_STANDALONE;
  request.size = size * count;
  request.count = 1;
  ret = dyplo_dma_common_block_alloc( dma_dev,
    &request,
    &dma_dev->dma_to_logic_blocks,
    DMA_BIDIRECTIONAL );

  if ( ret )
    return ret;

  // Configure the DMA node
  iowrite32( control_base + ( DYPLO_DMA_STANDALONE_STARTADDR >> 2 ),
    dma_dev->dma_to_logic_blocks.blocks[ 0 ].phys_addr );
  iowrite32( control_base + ( DYPLO_DMA_STANDALONE_BLOCKSIZE >> 2 ), size );
  iowrite32( control_base + ( DYPLO_DMA_STANDALONE_CONTROL >> 2 ),
    BIT( 0 ) | ( count << 8 ) );

  return 0;
}

static int dyplo_dma_to_logic_block_query(
  dyplo_dma_dev      *dma_dev,
  dyplo_buffer_block *arg
)
{
  if ( arg == NULL )
    return -EINVAL;

  uint32_t request_id = arg->id;

  if ( request_id >= dma_dev->dma_to_logic_blocks.count )
    return -EINVAL;

  *arg = dma_dev->dma_to_logic_blocks.blocks[ request_id ].data;

  DEBUG_PRINT( "ID: %" PRIu32 ", size: %" PRIu32, arg->id, arg->size );

  return 0;
}

static int dyplo_dma_to_logic_block_enqueue(
  dyplo_dma_dev      *dma_dev,
  dyplo_buffer_block *request
)
{
  dyplo_dma_block *block;
  uint32_t        *control_base = dma_dev->config_parent->control_base;

  if ( request == NULL )
    return -EINVAL;

  if ( request->id >= dma_dev->dma_to_logic_blocks.count )
    return -EINVAL;

  if ( ( request->bytes_used % DYPLO_DMA_MEMORY_ALIGNMENT ) != 0 ) {
    ERROR_PRINT( "DMA enqueue bytes_used size needs to be %d-byte aligned",
      DYPLO_DMA_MEMORY_ALIGNMENT );

    return -EINVAL;
  }

  block = &dma_dev->dma_to_logic_blocks.blocks[ request->id ];

  if ( block->data.state )
    return -EBUSY;

  block->data.bytes_used = request->bytes_used;
  block->data.user_signal = request->user_signal;

  // This operation never blocks, unless something is wrong in HW
  if ( !( dyplo_reg_read( control_base,
            DYPLO_DMA_TOLOGIC_STATUS ) & 0xFF0000 ) )
    return -EWOULDBLOCK;

  /* Transfer memory ownership to device, make sure data is in external RAM */
  if ( dma_dev->dma_to_logic_blocks.flags & DYPLO_DMA_BLOCK_FLAG_STREAMING )
    dyplo_dma_sync_device_to_device( block->mem_addr, block->data.bytes_used );

  DEBUG_PRINT( "sending addr=0x%" PRIx32 " size=%" PRIu32 "\n",
    block->phys_addr, block->data.bytes_used );
  iowrite32( control_base + ( DYPLO_DMA_TOLOGIC_STARTADDR >> 2 ),
    block->phys_addr );
  iowrite32( control_base + ( DYPLO_DMA_TOLOGIC_USERBITS >> 2 ),
    block->data.user_signal );
  iowrite32( control_base + ( DYPLO_DMA_TOLOGIC_BYTESIZE >> 2 ),
    block->data.bytes_used );
  block->data.state = 1;

  *request = block->data;

  return 0;
}

static int dyplo_dma_to_logic_block_dequeue(
  dyplo_dma_dev      *dma_dev,
  dyplo_buffer_block *request,
  bool                is_blocking
)
{
  if ( dma_dev->is_closing ) {
    return -EINTR;
  }

  // IMPORTANT: do not exit this function before the semaphore is released
  // (which is done at the end of the function).
  rtems_status_code sc = rtems_semaphore_obtain( dma_dev->write_semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  int              retval = 0;
  dyplo_dma_block *block;
  u_int32_t       *control_base = dma_dev->config_parent->control_base;
  uint32_t         status;
  dma_addr_t       start_addr;

  if ( request == NULL ) {
    retval = -EINVAL;
    goto error_exit;
  }

  if ( request->id >= dma_dev->dma_to_logic_blocks.count ) {
    retval = -EINVAL;
    goto error_exit;
  }

  block = &dma_dev->dma_to_logic_blocks.blocks[ request->id ];

  if ( !block->data.state ) {
    retval = -EINVAL;
    goto error_exit;
  }

  if ( is_blocking ) {
    while ( !dma_dev->is_closing ) {
      status = dyplo_reg_read( control_base, DYPLO_DMA_TOLOGIC_STATUS );
      DEBUG_PRINT( "Status: %" PRIx32, status );

      if ( status & 0xFF000000 )
        break;                         // Results available, done waiting

      // Enable interrupt
      dyplo_dma_to_logic_irq_enable( control_base );

      if ( !dma_dev->is_closing ) {
        rtems_status_code sc = rtems_semaphore_obtain(
          dma_dev->write_to_logic_wait_semaphore,
          RTEMS_WAIT,
          RTEMS_NO_TIMEOUT );
        assert( sc == RTEMS_SUCCESSFUL );
      }
    }

    if ( dma_dev->is_closing ) {
      goto closing;
    }
  } else {
    status = dyplo_reg_read( control_base, DYPLO_DMA_TOLOGIC_STATUS );

    if ( ( status & 0xFF000000 ) == 0 ) {
      retval = -EAGAIN;
      goto error_exit;
    }
  }

  start_addr = dyplo_reg_read( control_base, DYPLO_DMA_TOLOGIC_RESULT_ADDR );

  if ( start_addr != block->phys_addr ) {
    ERROR_PRINT( "Expected addr 0x%" PRIx32 " result 0x%" PRIx32,
      (uint32_t) block->phys_addr,
      (uint32_t) start_addr );
    retval = -EIO;
    goto error_exit;
  }

  if ( dma_dev->dma_to_logic_blocks.flags & DYPLO_DMA_BLOCK_FLAG_STREAMING )
    dyplo_dma_sync_cpu_to_device( block->mem_addr, block->data.bytes_used );

  block->data.state = 0;

  *request = block->data;

closing:

  if ( dma_dev->is_closing ) {
    retval = -EINTR;
  }

error_exit:
  rtems_semaphore_release( dma_dev->write_semaphore );
  DEBUG_PRINT( "Retval: %d", retval );

  return retval;
}

static int dyplo_dma_to_logic_reconfigure(
  dyplo_dma_dev               *dma_dev,
  dyplo_dma_configuration_req *request
)
{
  int ret = 0;

  DEBUG_PRINT( "mode=%" PRIu32 " count=%" PRIu32 " size=%" PRIu32,
    request->mode, request->count, request->size );

  if ( request->mode != DYPLO_DMA_MODE_RINGBUFFER_BOUNCE ) {
	if ( ( request->size % DYPLO_DMA_MEMORY_ALIGNMENT ) != 0 ) {
	  ERROR_PRINT( "DMA reconfigure, block size needs to be %d-byte aligned",
		DYPLO_DMA_MEMORY_ALIGNMENT );
	  ret = -EINVAL;
	}
  }

  if ( ret == 0 ) {
    dyplo_dma_to_logic_block_free( dma_dev );

    switch ( request->mode ) {
      case DYPLO_DMA_MODE_STANDALONE:

        // Device must be open in special mode
        if ( !( dma_dev->open_mode & DMA_FLAG_OPEN_MODE_EXCL ) )
          return -EACCES;

        // Clean up the "from" side as well

        dyplo_dma_from_logic_block_free( dma_dev );
        dyplo_dma_common_set_standalone_mode( dma_dev, true );
        ret = dyplo_dma_standalone_block_alloc( dma_dev,
        request->size, request->count );
        break;
      case DYPLO_DMA_MODE_RINGBUFFER_BOUNCE:
        dyplo_dma_common_set_standalone_mode( dma_dev, false );
        request->size = dma_dev->dma_to_logic_block_size;
        request->count = dma_dev->dma_to_logic_memory_size /
                         dma_dev->dma_to_logic_block_size;
        break;
      case DYPLO_DMA_MODE_BLOCK_COHERENT:
      case DYPLO_DMA_MODE_BLOCK_STREAMING:
        dyplo_dma_common_set_standalone_mode( dma_dev, false );
        ret = dyplo_dma_common_block_alloc( dma_dev,
        request, &dma_dev->dma_to_logic_blocks, DMA_TO_DEVICE );
        break;
      default:
        ret = -EINVAL;
    }
  }

  return ret;
}

static int dyplo_dma_from_logic_block_query(
  dyplo_dma_dev      *dma_dev,
  dyplo_buffer_block *arg
)
{
  if ( arg == NULL )
    return -EINVAL;

  uint32_t request_id = arg->id;

  if ( request_id >= dma_dev->dma_from_logic_blocks.count )
    return -EINVAL;

  *arg = dma_dev->dma_from_logic_blocks.blocks[ request_id ].data;

  return 0;
}

static int dyplo_dma_from_logic_block_enqueue(
  dyplo_dma_dev      *dma_dev,
  dyplo_buffer_block *request
)
{
  dyplo_dma_block *block;
  uint32_t        *control_base = dma_dev->config_parent->control_base;
  uint32_t         status_reg;

  if ( request == NULL )
    return -EINVAL;

  if ( request->id >= dma_dev->dma_from_logic_blocks.count )
    return -EINVAL;

  block = &dma_dev->dma_from_logic_blocks.blocks[ request->id ];

  if ( block->data.state )
    return -EBUSY;

  if ( ( request->bytes_used > block->data.size ) ||
       ( request->bytes_used == 0 ) )
    return -EINVAL;

  if ( ( request->bytes_used % DYPLO_DMA_MEMORY_ALIGNMENT ) != 0 ) {
    ERROR_PRINT( "DMA enqueue bytes_used size needs to be %d-byte aligned",
      DYPLO_DMA_MEMORY_ALIGNMENT );

    return -EINVAL;
  }

  // TODO: align on cache line boundary?
  if ( dma_dev->dma_from_logic_blocks.flags & DYPLO_DMA_BLOCK_FLAG_STREAMING )
    dyplo_dma_sync_device_from_device( block->mem_addr,
      block->data.bytes_used );

  // Should not block here because we never allocate more blocks than
  // what fits in the hardware queue.
  status_reg = dyplo_reg_read( control_base, DYPLO_DMA_FROMLOGIC_STATUS );
  DEBUG_PRINT( "status=0x%" PRIx32, status_reg );

  if ( !( status_reg & 0xFF0000 ) )
    return -EWOULDBLOCK;

  // Send to logic
  DEBUG_PRINT( "sending addr=0x%" PRIx32 " size=%" PRIu32,
    block->phys_addr, block->data.size );
  iowrite32( control_base + ( DYPLO_DMA_FROMLOGIC_STARTADDR >> 2 ),
    block->phys_addr );
  iowrite32( control_base + ( DYPLO_DMA_FROMLOGIC_BYTESIZE >> 2 ),
    request->bytes_used );
  block->data.bytes_used = 0;
  block->data.state = 1;

  *request = block->data;

  return 0;
}

static int dyplo_dma_from_logic_reconfigure(
  dyplo_dma_dev               *dma_dev,
  dyplo_dma_configuration_req *request
)
{
  int ret = 0;

  DEBUG_PRINT( "mode=%" PRIu32 " count=%" PRIu32 " size=%" PRIu32,
    request->mode, request->count, request->size );

  if ( request->mode != DYPLO_DMA_MODE_RINGBUFFER_BOUNCE ) {
	if ( ( request->size % DYPLO_DMA_MEMORY_ALIGNMENT ) != 0 ) {
	  ERROR_PRINT( "DMA reconfigure, block size needs to be %d-byte aligned",
		DYPLO_DMA_MEMORY_ALIGNMENT );
	  ret = -EINVAL;
	}
  }

  if ( ret == 0 ) {
    dyplo_dma_from_logic_block_free( dma_dev );

    switch ( request->mode ) {
      case DYPLO_DMA_MODE_STANDALONE:
        ret = -EACCES;                         // Must open in read+write mode for this
        break;
      case DYPLO_DMA_MODE_RINGBUFFER_BOUNCE:
        request->size = dma_dev->dma_from_logic_block_size;
        request->count = dma_dev->dma_from_logic_memory_size /
                         dma_dev->dma_from_logic_block_size;
        ret = 0;
        break;
      case DYPLO_DMA_MODE_BLOCK_COHERENT:
      case DYPLO_DMA_MODE_BLOCK_STREAMING:
        ret = dyplo_dma_common_block_alloc( dma_dev,
        request, &dma_dev->dma_from_logic_blocks, DMA_FROM_DEVICE );
        break;
      default:
        ret = -EINVAL;
    }
  }

  return ret;
}

static int dyplo_dma_from_logic_block_dequeue(
  dyplo_dma_dev      *dma_dev,
  dyplo_buffer_block *request,
  bool                is_blocking
)
{
  if ( dma_dev->is_closing ) {
    return -EINTR;
  }

  // IMPORTANT: do not exit this function before the semaphore is released
  // (which is done at the end of the function).
  rtems_status_code sc = rtems_semaphore_obtain( dma_dev->read_semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  int       status;
  uint32_t *control_base = dma_dev->config_parent->control_base;

  dyplo_dma_block *block;
  uint32_t         status_reg;
  dma_addr_t       start_addr;

  if ( request == NULL ) {
    status = -EINVAL;
    goto exit_error;
  }

  if ( request->id >= dma_dev->dma_from_logic_blocks.count ) {
    status = -EINVAL;
    goto exit_error;
  }

  block = &dma_dev->dma_from_logic_blocks.blocks[ request->id ];

  if ( !block->data.state ) {
    status = -EINVAL;
    goto exit_error;
  }

  while ( !dma_dev->is_closing ) {
    status_reg = dyplo_reg_read( control_base, DYPLO_DMA_FROMLOGIC_STATUS );
    DEBUG_PRINT( "status=0x%" PRIx32, status_reg );

    // TODO: Blocking. Linux driver comment. Verify with mike
    if ( status_reg & 0xFF000000 )
      break;                   // Result(s) available, we're done

    // Enable interrupt
    dyplo_dma_from_logic_irq_enable( control_base );

    if ( !is_blocking ) {
      status = -EAGAIN;
      goto exit_error;
    }

    if ( !dma_dev->is_closing ) {
      rtems_status_code sc = rtems_semaphore_obtain(
        dma_dev->read_from_logic_wait_semaphore,
        RTEMS_WAIT,
        RTEMS_NO_TIMEOUT );
      assert( sc == RTEMS_SUCCESSFUL );
    }
  }

  if ( dma_dev->is_closing ) {
    goto closing;
  }

  start_addr = dyplo_reg_read( control_base, DYPLO_DMA_FROMLOGIC_RESULT_ADDR );

  if ( start_addr != block->phys_addr ) {
    ERROR_PRINT( "Expected addr 0x%" PRIx32 " result 0x%" PRIx32,
      block->phys_addr, start_addr );
    status = -EIO;
    goto exit_error;
  }

  block->data.user_signal = dyplo_reg_read( control_base,
    DYPLO_DMA_FROMLOGIC_RESULT_USERBITS );
  block->data.bytes_used = dyplo_reg_read( control_base,
    DYPLO_DMA_FROMLOGIC_RESULT_BYTESIZE );
  block->data.state = 0;

  /* Transfer memory ownership from device to cpu */
  if ( dma_dev->dma_from_logic_blocks.flags & DYPLO_DMA_BLOCK_FLAG_STREAMING )
    dyplo_dma_sync_cpu_from_device( block->mem_addr, block->data.bytes_used );

  *request = block->data;
closing:

  if ( dma_dev->is_closing ) {
    status = -EINTR;
  }

exit_error:
  rtems_semaphore_release( dma_dev->read_semaphore );

  return status;
}

static long dyplo_dma_standalone_enable(
  dyplo_dma_dev          *dma_dev,
  enum dma_data_direction direction,
  bool                    enable
)
{
  uint32_t *ctrl_reg = dma_dev->config_parent->control_base;

  ctrl_reg +=
    ( direction ==
      DMA_TO_DEVICE ) ? DYPLO_DMA_TOLOGIC_CONTROL >>
    2 : DYPLO_DMA_FROMLOGIC_CONTROL >>
    2;

  dyplo_set_bits( ctrl_reg, BIT( 0 ), enable );

  return 0;
}

static long dyplo_dma_standalone_setconfig(
  dyplo_dma_dev               *dma_dev,
  dyplo_dma_standalone_config *cfg,
  enum dma_data_direction      direction
)
{
  uint32_t *control_base = dma_dev->config_parent->control_base;
  uint32_t *cfg_ptr = (uint32_t *) cfg;
  uint8_t   i;

  switch ( direction ) {
    case DMA_TO_DEVICE:

      for ( i = 0;
            i < ( sizeof( dyplo_dma_standalone_config ) / sizeof( uint32_t ) );
            ++i ) {
        dyplo_reg_write_index( control_base,
          DYPLO_DMA_STANDALONE_TOLOGIC_BASE,
          i,
          cfg_ptr[ i ] );
      }

      break;
    case DMA_FROM_DEVICE:

      for ( i = 0;
            i < ( sizeof( dyplo_dma_standalone_config ) / sizeof( uint32_t ) );
            ++i ) {
        dyplo_reg_write_index( control_base,
          DYPLO_DMA_STANDALONE_FROMLOGIC_BASE,
          i,
          cfg_ptr[ i ] );
      }

      break;
    default:

      return -EINVAL;
  }

  return 0;
}

static long dyplo_dma_standalone_getconfig(
  dyplo_dma_dev               *dma_dev,
  dyplo_dma_standalone_config *cfg,
  enum dma_data_direction      direction
)
{
  uint32_t *control_base = dma_dev->config_parent->control_base;
  uint32_t *cfg_ptr = (uint32_t *) cfg;
  uint8_t   i;

  switch ( direction ) {
    case DMA_TO_DEVICE:

      for ( i = 0;
            i < ( sizeof( dyplo_dma_standalone_config ) / sizeof( uint32_t ) );
            ++i ) {
        uint32_t value = dyplo_reg_read_index( control_base,
          DYPLO_DMA_STANDALONE_TOLOGIC_BASE,
          i );
        cfg_ptr[ i ] = value;
      }

      break;
    case DMA_FROM_DEVICE:

      for ( i = 0;
            i <
            ( sizeof( dyplo_dma_standalone_config ) / sizeof( u_int32_t ) );
            ++i ) {
        cfg_ptr[ i ] = dyplo_reg_read_index( control_base,
          DYPLO_DMA_STANDALONE_FROMLOGIC_BASE,
          i );
      }

      break;
    default:

      return -EINVAL;
  }

  return 0;
}

static long dyplo_dma_ioctl( rtems_libio_ioctl_args_t *args )
{
  dyplo_dma_dev *dma_dev = (dyplo_dma_dev *) args->iop->data1;

  if ( dma_dev == NULL )
    return -ENODEV;

  uint32_t command = _IOC_NR( args->command );
  DEBUG_PRINT( "Command: %" PRIu32 "", command );

  if ( _IOC_TYPE( args->command ) != DYPLO_IOC_MAGIC )
    return -ENOTTY;

  typedef enum {
    READ_FROM_LOGIC,
    WRITE_TO_LOGIC
  } DMA_OPEN_MODE;

  const DMA_OPEN_MODE file_open_mode =
    ( args->iop->flags &
      LIBIO_FLAGS_WRITE ) ? WRITE_TO_LOGIC : READ_FROM_LOGIC;

  switch ( command ) {
    case DYPLO_IOC_ROUTE_QUERY_ID:

      return dyplo_dma_get_route_id( dma_dev );
      break;
    case DYPLO_IOC_ROUTE_TELL_TO_LOGIC: {
      if ( file_open_mode == WRITE_TO_LOGIC ) {
        int dest = (int) args->buffer;

        return dyplo_dma_add_route( dma_dev, dyplo_dma_get_route_id(
            dma_dev ), dest );
      } else {
        return -ENOTTY;
      }
    }
    case DYPLO_IOC_ROUTE_TELL_FROM_LOGIC:

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return -ENOTTY;                         // Cannot route to this node
      } else {
        int dest = (int) args->buffer;

        return dyplo_dma_add_route( dma_dev, dest,
          dyplo_dma_get_route_id( dma_dev ) );
      }

    case DYPLO_IOC_TRESHOLD_QUERY:

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return (int) dma_dev->dma_to_logic_block_size;
      } else {
        return (int) dma_dev->dma_from_logic_block_size;
      }

    case DYPLO_IOC_TRESHOLD_TELL: {
      unsigned int arg = (unsigned int) args->buffer;

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        if ( dma_dev->dma_to_logic_block_size == arg )
          return 0;

        if ( ( dma_dev->dma_to_logic_head != dma_dev->dma_to_logic_tail ) ||
             !dma_command_fifo_is_empty( dma_dev ) )
          return -EBUSY;

        if ( dma_dev->dma_to_logic_memory_size % arg )
          return -EINVAL;                               // Must be divisable

        dma_dev->dma_to_logic_block_size = arg;
        dma_dev->dma_to_logic_head = 0;
        dma_dev->dma_to_logic_tail = 0;

        return 0;
      } else {
        if ( dma_dev->dma_from_logic_block_size == arg )
          return 0;

        if ( ( dma_dev->dma_from_logic_head !=
               dma_dev->dma_from_logic_tail ) ||
             dma_dev->dma_from_logic_full )
          return -EBUSY;                               // Cannot change value

        if ( dma_dev->dma_from_logic_memory_size % arg )
          return -EINVAL;                              // Must be divisable

        dma_dev->dma_from_logic_block_size = arg;
        dma_dev->dma_from_logic_head = 0;
        dma_dev->dma_from_logic_tail = 0;

        return 0;
      }
    }
    case DYPLO_IOC_RESET_FIFO_WRITE:
    case DYPLO_IOC_RESET_FIFO_READ:

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return dyplo_dma_to_logic_reset( dma_dev );
      } else {
        return dyplo_dma_from_logic_reset( dma_dev );
      }

    case DYPLO_IOC_USERSIGNAL_QUERY:

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return (int) dyplo_reg_read( dma_dev->config_parent->control_base,
          DYPLO_DMA_TOLOGIC_USERBITS );
      } else {
        return dma_dev->dma_from_logic_current_op.user_signal;
      }

    case DYPLO_IOC_USERSIGNAL_TELL:

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        int user_signal = (int) args->buffer;
        iowrite32( dma_dev->config_parent->control_base +
          ( DYPLO_DMA_TOLOGIC_USERBITS >> 2 ), user_signal );
      } else {
        return -EACCES;
      }

    case DYPLO_IOC_DMA_RECONFIGURE: {
      dyplo_dma_configuration_req *req =
        (dyplo_dma_configuration_req *) args->buffer;

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return dyplo_dma_to_logic_reconfigure( dma_dev, req );
      } else {
        return dyplo_dma_from_logic_reconfigure( dma_dev, req );
      }
    }
    case DYPLO_IOC_DMABLOCK_FREE:

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return dyplo_dma_to_logic_block_free( dma_dev );
      } else {
        return dyplo_dma_from_logic_block_free( dma_dev );
      }

    case DYPLO_IOC_DMABLOCK_QUERY: {
      dyplo_buffer_block *buff = (dyplo_buffer_block *) args->buffer;

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return dyplo_dma_to_logic_block_query( dma_dev, buff );
      } else {
        return dyplo_dma_from_logic_block_query( dma_dev, buff );
      }
    }
    case DYPLO_IOC_DMABLOCK_ENQUEUE: {
      dyplo_buffer_block *block = (dyplo_buffer_block *) args->buffer;

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return dyplo_dma_to_logic_block_enqueue( dma_dev, block );
      } else {
        return dyplo_dma_from_logic_block_enqueue( dma_dev, block );
      }
    }
    case DYPLO_IOC_DMABLOCK_DEQUEUE: {
      dyplo_buffer_block *block = (dyplo_buffer_block *) args->buffer;

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return dyplo_dma_to_logic_block_dequeue( dma_dev,
          block,
          ( args->iop->flags & LIBIO_FLAGS_NO_DELAY ) == 0 );
      } else {
        return dyplo_dma_from_logic_block_dequeue( dma_dev,
          block,
          ( args->iop->flags & LIBIO_FLAGS_NO_DELAY ) == 0 );
      }
    }
    case DYPLO_IOC_DMASTANDALONE_CONFIGURE_TO_LOGIC: {
      dyplo_dma_standalone_config *cfg =
        (dyplo_dma_standalone_config *) args->buffer;

      if ( ( args->command & IOC_DIRMASK ) & IOC_IN ) {
        return dyplo_dma_standalone_setconfig( dma_dev,
          cfg,
          DMA_TO_DEVICE );
      }

      if ( ( args->command & IOC_DIRMASK ) & IOC_OUT ) {
        return dyplo_dma_standalone_getconfig( dma_dev,
          cfg,
          DMA_TO_DEVICE );
      }

      return -EINVAL;
    }
    case DYPLO_IOC_DMASTANDALONE_CONFIGURE_FROM_LOGIC: {
      dyplo_dma_standalone_config *cfg =
        (dyplo_dma_standalone_config *) args->buffer;

      if ( ( args->command & IOC_DIRMASK ) & IOC_IN ) {
        return dyplo_dma_standalone_setconfig( dma_dev,
          cfg,
          DMA_FROM_DEVICE );
      }

      if ( ( args->command & IOC_DIRMASK ) & IOC_OUT ) {
        return dyplo_dma_standalone_getconfig( dma_dev,
          cfg,
          DMA_FROM_DEVICE );
      }

      return -EINVAL;
    }
    case DYPLO_IOC_DMASTANDALONE_START_TO_LOGIC:

      return dyplo_dma_standalone_enable( dma_dev, DMA_TO_DEVICE, true );
    case DYPLO_IOC_DMASTANDALONE_START_FROM_LOGIC:

      return dyplo_dma_standalone_enable( dma_dev, DMA_FROM_DEVICE, true );
    case DYPLO_IOC_DMASTANDALONE_STOP_TO_LOGIC:

      return dyplo_dma_standalone_enable( dma_dev, DMA_TO_DEVICE, false );
    case DYPLO_IOC_DMASTANDALONE_STOP_FROM_LOGIC:

      return dyplo_dma_standalone_enable( dma_dev, DMA_FROM_DEVICE, false );
	case DYPLO_IOC_WAIT_FOR_INCOMING_DATA:
	case DYPLO_IOC_POLL_FOR_INCOMING_DATA: {
	  struct timeval *timeout = NULL;
	  if ( command == DYPLO_IOC_POLL_FOR_INCOMING_DATA ) {
		timeout = (struct timeval *) args->buffer;
	  }

      if ( file_open_mode == READ_FROM_LOGIC ) {
        return dyplo_dma_from_logic_poll( dma_dev, timeout );
      }

      break;
    }
	case DYPLO_IOC_WAIT_FOR_OUTGOING_DATA:
    case DYPLO_IOC_POLL_FOR_OUTGOING_DATA: {
	  struct timeval *timeout = NULL;
	  if ( command == DYPLO_IOC_POLL_FOR_OUTGOING_DATA ) {
		timeout = (struct timeval *) args->buffer;
	  }

      if ( file_open_mode == WRITE_TO_LOGIC ) {
        return dyplo_dma_to_logic_poll( dma_dev, timeout );
      }
    }
    default:

      return -ENOTTY;
  }

  return -ENOTTY;
}

static long dyplo_ctl_static_id(
  dyplo_dev      *dev,
  ioctl_command_t cmd,
  unsigned int   *user_id
)
{
  unsigned int data;

  data = dyplo_reg_read( dev->base, DYPLO_REG_CONTROL_STATIC_ID );
  *user_id = data;

  if ( IOCPARM_LEN( cmd ) != sizeof( uint32_t ) )
    return -EINVAL;

  if ( ( cmd & IOC_DIRMASK ) & IOC_OUT ) {
    if ( !data ) {
      // When "0" is returned, check the dyplo version to see
      // if the Dyplo version is before 2015.1.4
      data = dyplo_reg_read( dev->base, DYPLO_REG_CONTROL_DYPLO_VERSION );

      if ( data < ( ( 2015 << 16 ) | 0x0104 ) )
        return -EIO;
    }
  }

  return 0;
}

static int dyplo_get_icap_device_index( dyplo_dev *dev )
{
  uint8_t index = dev->icap_device_index;

  if ( index == ICAP_NOT_AVAILABLE )
    return -ENODEV;

  return index;
}

static long dyplo_ctl_ioctl( rtems_libio_ioctl_args_t *args )
{
  int status = -1;

  dyplo_dev *dev = (dyplo_dev *) args->iop->data1;

  uint32_t command = _IOC_NR( args->command );

  DEBUG_PRINT( "Command: %" PRIu32 "", command );

  if ( _IOC_TYPE( args->command ) != DYPLO_IOC_MAGIC )
    return -ENOTTY;

  switch ( command ) {
    case DYPLO_IOC_ROUTE_CLEAR:             // Remove all routes
      status = dyplo_ctl_route_clear( dev );
      break;
    case DYPLO_IOC_ROUTE_SET:             // Set routes.
      status = dyplo_ctl_routes_add( dev, (dyplo_route_t *) args->buffer );
      break;
    case DYPLO_IOC_ROUTE_GET:             // Get routes
    {
      status = dyplo_ctl_route_get( dev, (dyplo_route_t *) args->buffer );
      break;
    }
    case DYPLO_IOC_ROUTE_TELL:             // Tell route: Adds a single route entry
    {
      status = dyplo_ctl_route_add( dev, (dyplo_route_item_t *) args->buffer );
      break;
    }
    case DYPLO_IOC_ROUTE_DELETE:             // Remove routes to a node
    {
      int arg = (int) args->buffer;
      status = dyplo_ctl_route_delete( dev, arg );
      break;
    }
    case DYPLO_IOC_BACKPLANE_STATUS:
      status =
        dyplo_reg_read( dev->base, DYPLO_REG_BACKPLANE_ENABLE_STATUS ) >> 1;
      break;
    case DYPLO_IOC_BACKPLANE_ENABLE: {
      unsigned int arg = (unsigned int) args->buffer;
      dyplo_reg_write( dev->base, DYPLO_REG_BACKPLANE_ENABLE_SET, arg << 1 );
      status =
        dyplo_reg_read( dev->base, DYPLO_REG_BACKPLANE_ENABLE_STATUS ) >> 1;
      break;
    }
    case DYPLO_IOC_BACKPLANE_DISABLE: {
      unsigned int arg = (unsigned int) args->buffer;
      dyplo_reg_write( dev->base, DYPLO_REG_BACKPLANE_ENABLE_CLR, arg << 1 );
      status =
        dyplo_reg_read( dev->base, DYPLO_REG_BACKPLANE_ENABLE_STATUS ) >> 1;
      break;
    }
    case DYPLO_IOC_BACKPLANE_N2B_COUNTER_GET: {            // Get N2B counter
      dyplo_backplane_counter_t *counter = (dyplo_backplane_counter_t *)args->buffer;
      counter->counter = dyplo_reg_read(  dev->base, 
                                          DYPLO_REG_BACKPLANE_COUNTER_F2B_BASE + 
                                          (counter->node_id * 0x4) );
      status = 0;
      break;
    }
    case DYPLO_IOC_BACKPLANE_B2N_COUNTER_GET: {            // Get B2N counter
      dyplo_backplane_counter_t *counter = (dyplo_backplane_counter_t *)args->buffer;
      counter->counter = dyplo_reg_read(  dev->base, 
                                          DYPLO_REG_BACKPLANE_COUNTER_B2F_BASE + 
                                          (counter->node_id * 0x4) );
      status = 0;
      break;
    }       
    case DYPLO_IOC_ICAP_INDEX_QUERY:
      status = dyplo_get_icap_device_index( dev );
      break;
    case DYPLO_IOC_LICENSE_KEY:
      status = dyplo_ctl_license_key( dev, args->command, args->buffer );
      break;
    case DYPLO_IOC_STATIC_ID: {
      unsigned int *arg = (unsigned int *) args->buffer;
      status = dyplo_ctl_static_id( dev, args->command, arg );
      break;
    }
    default:
      ERROR_PRINT( "DYPLO ioctl unknown command: %" PRIu32 ".", command );
      status = -ENOTTY;
  }

  DEBUG_PRINT( "Status: %d", status );

  return status;
}

static int dyplo_cfg_open( rtems_libio_open_close_args_t *args )
{
  dyplo_config_dev *cfg_dev = (dyplo_config_dev *) args->iop->data1;
  dyplo_dev        *dev = cfg_dev->parent;
  int               status = 0;

  uint8_t open_mode =
    ( args->iop->flags & ( LIBIO_FLAGS_WRITE | LIBIO_FLAGS_READ ) );

  rtems_status_code sc = rtems_semaphore_obtain( dev->semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );

  assert( sc == RTEMS_SUCCESSFUL );

  // Allow only one RW open, or one R and one W
  if ( open_mode & cfg_dev->open_mode ) {
    status = -EBUSY;
    goto exit_open;
  }

  cfg_dev->open_mode |= open_mode;       // Set in-use bits

exit_open:
  rtems_semaphore_release( dev->semaphore );

  return status;
}

static ssize_t dyplo_cfg_read( rtems_libio_rw_args_t *args )
{
  dyplo_config_dev *cfg_dev = (dyplo_config_dev *) args->iop->data1;

  ssize_t status = dyplo_generic_read( cfg_dev->base,
    args->buffer,
    args->count,
    &args->offset );

  DEBUG_PRINT( "return status: %d", status );

  if ( status >= 0 ) {
    args->bytes_moved = status;
  }

  return status;
}

static ssize_t dyplo_cfg_write( rtems_libio_rw_args_t *args )
{
  dyplo_config_dev *cfg_dev = (dyplo_config_dev *) args->iop->data1;

  ssize_t status = dyplo_generic_write( cfg_dev->base,
    args->buffer,
    args->count,
    &args->offset );

  DEBUG_PRINT( "return status: %d", status );

  if ( status >= 0 ) {
    args->bytes_moved = status;
  }

  return status;
}

static int dyplo_cfg_release( rtems_libio_open_close_args_t *args )
{
  dyplo_config_dev *cfg_dev = (dyplo_config_dev *) args->iop->data1;
  dyplo_dev        *dev = cfg_dev->parent;

  rtems_status_code sc = rtems_semaphore_obtain( dev->semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );

  assert( sc == RTEMS_SUCCESSFUL );

  cfg_dev->open_mode &= ~( args->iop->flags );     // Clear in use bits

  rtems_semaphore_release( dev->semaphore );

  return 0;
}

static long dyplo_cfg_ioctl( rtems_libio_ioctl_args_t *args )
{
  int status = -1;

  dyplo_config_dev *cfg_dev = (dyplo_config_dev *) args->iop->data1;

  uint32_t command = _IOC_NR( args->command );

  DEBUG_PRINT( "Command: %" PRIu32 "", command );

  if ( _IOC_TYPE( args->command ) != DYPLO_IOC_MAGIC )
    return -ENOTTY;

  switch ( command ) {
    case DYPLO_IOC_ROUTE_CLEAR:
    case DYPLO_IOC_ROUTE_DELETE:             // Remove routes to this node
      status =
        dyplo_ctl_route_delete( cfg_dev->parent,
          dyplo_get_config_index( cfg_dev ) );
      break;
    case DYPLO_IOC_ROUTE_QUERY_ID:
      status = dyplo_get_config_index( cfg_dev );
      break;
    case DYPLO_IOC_BACKPLANE_STATUS: {
      int index = dyplo_get_config_index( cfg_dev );
      status = dyplo_reg_read( cfg_dev->parent->base,
        DYPLO_REG_BACKPLANE_ENABLE_STATUS ) >> 1;
      status &= ( 1 << index );
    }
    break;
    case DYPLO_IOC_BACKPLANE_ENABLE: {
      int index = dyplo_get_config_index( cfg_dev );
      dyplo_reg_write( cfg_dev->parent->base,
        DYPLO_REG_BACKPLANE_ENABLE_SET,
        1 << ( index + 1 ) );
      status = dyplo_reg_read( cfg_dev->parent->base,
        DYPLO_REG_BACKPLANE_ENABLE_STATUS ) >> 1;
    }
    break;
    case DYPLO_IOC_BACKPLANE_DISABLE: {
      int index = dyplo_get_config_index( cfg_dev );
      dyplo_reg_write( cfg_dev->parent->base,
        DYPLO_REG_BACKPLANE_ENABLE_CLR,
        1 << ( index + 1 ) );
      status = dyplo_reg_read( cfg_dev->parent->base,
        DYPLO_REG_BACKPLANE_ENABLE_STATUS ) >> 1;
    }
    break;
    case DYPLO_IOC_RESET_FIFO_WRITE: {
      unsigned int mask = (unsigned int) args->buffer;
      dyplo_reg_write( cfg_dev->control_base, DYPLO_REG_NODE_RESET_FIFOS,
        mask );
      status = 0;
      break;
    }
    case DYPLO_IOC_RESET_FIFO_READ: {
      unsigned int mask = (unsigned int) args->buffer;
      dyplo_reg_write( cfg_dev->control_base, DYPLO_REG_FIFO_RESET_READ,
        mask );
      status = 0;
      break;
    }
    default:
      ERROR_PRINT( "DYPLO ioctl unknown command: %" PRIu32 ".", command );
      status = -ENOTTY;
  }

  DEBUG_PRINT( "(IOCNR %" PRIu32 ") result=%d", command, status );

  return status;
}

static int dyplo_fifo_read_open( rtems_libio_open_close_args_t *args )
{
  int             status = 0;
  dyplo_fifo_dev *fifo_dev = (dyplo_fifo_dev *) args->iop->data1;
  dyplo_dev      *dev = fifo_dev->config_parent->parent;

  DEBUG_PRINT( "mode=%#" PRIx32 "flags=%#" PRIx32 "", args->mode,
    args->flags );

  if ( ( args->iop->flags & LIBIO_FLAGS_NO_DELAY ) ) {
    DEBUG_PRINT( "Non blocking" );
  }

  if ( ( args->iop->flags & LIBIO_FLAGS_WRITE ) ) { // read-only device
    return -EINVAL;
  }

  rtems_status_code sc = rtems_semaphore_obtain( dev->semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return -EIO;
  }

  if ( fifo_dev->is_open ) {
    status = -EBUSY;
    goto exit;
  }

  fifo_dev->user_signal = DYPLO_USERSIGNAL_ZERO;
  fifo_dev->eof = false;
  fifo_dev->is_open = true;
  fifo_dev->poll_treshold = 1;

exit:
  rtems_semaphore_release( dev->semaphore );

  return status;
}

static ssize_t dyplo_fifo_read_read( rtems_libio_rw_args_t *args )
{
  dyplo_fifo_dev *fifo_dev = (dyplo_fifo_dev *) args->iop->data1;

  if ( fifo_dev->is_closing ) {
    return -EINTR;
  }

  // IMPORTANT: do not exit this function before the semaphore is released
  // (which is done at the end of the function).
  rtems_status_code sc = rtems_semaphore_obtain(
    fifo_dev->read_write_semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  DEBUG_PRINT( "count: %" PRIu32 "", args->count );

  int status = 0;

  uint32_t *fifo_memory = dyplo_fifo_memory_location( fifo_dev );

  uint32_t *buf = (uint32_t *) args->buffer;
  uint32_t  count = args->count;

  if ( count == 0 ) {
    status = 0;
    goto error;
  }

  if ( count < 4 ) {   // Do not allow read or write below word size
    status = -EINVAL;
    goto error;
  }

  count &= ~0x03;       // Align to words

  while ( count && !( fifo_dev->is_closing ) ) {
    uint32_t words_available = 0;
    uint16_t user_signal;

    if ( args->flags & LIBIO_FLAGS_NO_DELAY ) {         // Non-blocking IO
      words_available = dyplo_fifo_read_level( fifo_dev );
      user_signal = words_available >> 16;
      words_available &= 0xFFFF;                   /* Lower 16-bits only */

      if ( !words_available ) {
        // Non-blocking IO, return what we have
        if ( args->bytes_moved )
          break;

        // nothing copied yet, notify caller
        status = -EAGAIN;
        goto error;
      }

      // user_signal is valid because words_available is non-nul
      if ( user_signal != fifo_dev->user_signal ) {
        DEBUG_PRINT( "user signal exit" );
        fifo_dev->user_signal = user_signal;
        goto exit_ok;
      }
    } else {
      while ( !fifo_dev->is_closing ) {
        words_available = dyplo_fifo_read_level( fifo_dev );
        user_signal = words_available >> 16;
        words_available &= 0xFFFF;

        if ( words_available ) {
          // usersignal is only valid when there is data
          if ( user_signal != fifo_dev->user_signal ) {
            fifo_dev->user_signal = user_signal;
            goto exit_ok;
          }

          break;                               // Done waiting
        }

        dyplo_fifo_read_enable_interrupt( fifo_dev, count >> 2 );

        if ( !fifo_dev->is_closing ) {
          rtems_status_code sc = rtems_semaphore_obtain(
            fifo_dev->wait_semaphore,
            RTEMS_WAIT,
            RTEMS_NO_TIMEOUT );
          assert( sc == RTEMS_SUCCESSFUL );
        }
      }
    }

    if ( fifo_dev->is_closing ) {
      goto closing;
    } else {
      // read loop:
      do {
        uint32_t words;
        uint32_t bytes = words_available << 2;

        if ( bytes > DYPLO_FIFO_READ_MAX_BURST_SIZE )
          bytes = DYPLO_FIFO_READ_MAX_BURST_SIZE;

        if ( count < bytes )
          bytes = count;

        words = bytes >> 2;

        for ( uint32_t i = 0; i < words; ++i ) {
          buf[ i ] = ioread32( fifo_memory );
        }

        fifo_dev->words_transfered += words;
        args->bytes_moved += bytes;
        buf += words;
        count -= bytes;

        if ( !count )
          break;

        words_available -= words;
      } while ( words_available && !( fifo_dev->is_closing ) );
    }
  }

exit_ok:
  status = args->bytes_moved;
closing:

  if ( fifo_dev->is_closing ) {
    status = -EINTR;
  }

error:
  rtems_semaphore_release( fifo_dev->read_write_semaphore );

  return status;
}

static uint32_t dyplo_fifo_write_level( dyplo_fifo_dev *fifo_dev )
{
  return dyplo_reg_read_index(
    fifo_dev->config_parent->control_base,
    DYPLO_REG_FIFO_WRITE_LEVEL_BASE,
    fifo_dev->index );
}

static void dyplo_fifo_write_enable_interrupt(
  dyplo_fifo_dev *fifo_dev,
  int             thd
)
{
  int       index = fifo_dev->index;
  uint32_t *control_base = fifo_dev->config_parent->control_base;

  if ( thd > ( DYPLO_FIFO_WRITE_SIZE * 2 ) / 3 )
    thd = ( DYPLO_FIFO_WRITE_SIZE * 2 ) / 3;
  else if ( thd )
    --thd;             // IRQ will trigger when level is above thd

  DEBUG_PRINT( "index=%d thd=%d", index, thd );

  iowrite32( control_base + ( DYPLO_REG_FIFO_WRITE_THD_BASE >> 2 ) + index,
    thd );
  iowrite32( control_base + ( DYPLO_REG_FIFO_IRQ_SET >> 2 ), BIT( index ) );
}

static int dyplo_fifo_read_poll(
  dyplo_fifo_dev *fifo_dev,
  struct timeval *timeout
)
{
  if ( timeout != NULL )
	DEBUG_PRINT( "timeout: %d secs", (int) timeout->tv_sec );
  else
	DEBUG_PRINT( "" );

  int mask = 0;

  if ( fifo_dev->eof ||
       ( ( dyplo_fifo_read_level( fifo_dev ) & 0xFFFF ) > 0 ) ) {
    mask = ( POLLIN | POLLRDNORM );           // Data available
  } else {
	if ( timeout == NULL || ( timeout->tv_sec > 0 || timeout->tv_usec > 0 ) ) {
	  if ( !fifo_dev->is_closing ) {
		// Set IRQ to occur on user-defined treshold (default=1)
		dyplo_fifo_read_enable_interrupt( fifo_dev, fifo_dev->poll_treshold );

		rtems_status_code sc = rtems_semaphore_obtain(
		  fifo_dev->wait_semaphore,
		  RTEMS_WAIT,
		  timeval_to_rtems_interval( timeout ) );
		assert( sc == RTEMS_SUCCESSFUL || sc == RTEMS_TIMEOUT );

		if ( fifo_dev->eof ||
			 ( ( dyplo_fifo_read_level( fifo_dev ) & 0xFFFF ) > 0 ) ) {
		  mask = ( POLLIN | POLLRDNORM );
		} else {
		  if ( sc == RTEMS_SUCCESSFUL && !fifo_dev->is_closing ) {
			/* apparently the semaphore was released normally, but there is no available data yet.
			  possibly the semaphore was in a state where semaphore count was at '1' (so obtaining it
			  did not block).. try again. */
			rtems_status_code sc = rtems_semaphore_obtain(
			  fifo_dev->wait_semaphore,
			  RTEMS_WAIT,
			  timeval_to_rtems_interval( timeout ) );
			assert( sc == RTEMS_SUCCESSFUL || sc == RTEMS_TIMEOUT );

			if ( fifo_dev->eof ||
				 ( ( dyplo_fifo_read_level( fifo_dev ) & 0xFFFF ) > 0 ) ) {
			  mask = ( POLLIN | POLLRDNORM );
			}
		  }
		}
	  }
    }
  }

  DEBUG_PRINT( "%#x", mask );

  return mask;
}

static int dyplo_fifo_write_poll(
  dyplo_fifo_dev *fifo_dev,
  struct timeval *timeout
)
{
  if ( timeout != NULL )
	DEBUG_PRINT( "timeout: %d secs", (int) timeout->tv_sec );
  else
	DEBUG_PRINT( "" );

  int mask = 0;

  if ( dyplo_fifo_write_level( fifo_dev ) > 0 ) {
    mask = ( POLLOUT | POLLWRNORM );
  } else {
	if ( timeout == NULL || ( timeout->tv_sec > 0 || timeout->tv_usec > 0 ) ) {
	  if ( !fifo_dev->is_closing ) {
		/* Wait for buffer crossing user-defined treshold */
		dyplo_fifo_write_enable_interrupt( fifo_dev, fifo_dev->poll_treshold );

		rtems_status_code sc = rtems_semaphore_obtain(
		  fifo_dev->wait_semaphore,
		  RTEMS_WAIT,
		  timeval_to_rtems_interval( timeout ) );
		assert( sc == RTEMS_SUCCESSFUL || sc == RTEMS_TIMEOUT );

		if ( dyplo_fifo_write_level( fifo_dev ) > 0 ) {
		  mask = ( POLLOUT | POLLWRNORM );
		} else {
		  if (sc == RTEMS_SUCCESSFUL && !fifo_dev->is_closing ) {
			/* apparently the semaphore was released normally, but there is no available space yet.
			  possibly the semaphore was in a state where semaphore count was at '1' (so obtaining it
			  did not block).. try again. */

			rtems_status_code sc = rtems_semaphore_obtain(
			  fifo_dev->wait_semaphore,
			  RTEMS_WAIT,
			  timeval_to_rtems_interval( timeout ) );
			assert( sc == RTEMS_SUCCESSFUL || sc == RTEMS_TIMEOUT );

			if ( dyplo_fifo_write_level( fifo_dev ) > 0 ) {
			  mask = ( POLLOUT | POLLWRNORM );
			}
		  }
		}
	  }
	}
  }

  DEBUG_PRINT( "%#x", mask );

  return mask;
}

static void dyplo_fifo_common_stop_read_write( dyplo_fifo_dev *fifo_dev )
{
  rtems_status_code sc;

  fifo_dev->is_closing = true;

  sc = rtems_semaphore_release( fifo_dev->wait_semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  // wait till any read or write is completed
  sc = rtems_semaphore_obtain( fifo_dev->read_write_semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  sc = rtems_semaphore_release( fifo_dev->read_write_semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  fifo_dev->is_closing = false;
}

static int dyplo_fifo_read_release( rtems_libio_open_close_args_t *args )
{
  dyplo_fifo_dev *fifo_dev = (dyplo_fifo_dev *) args->iop->data1;
  dyplo_dev      *dev = fifo_dev->config_parent->parent;

  DEBUG_PRINT( "index=%" PRIu32, fifo_dev->index );

  rtems_status_code sc = rtems_semaphore_obtain( dev->semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  dyplo_fifo_common_stop_read_write( fifo_dev );
  fifo_dev->is_open = false;

  sc = rtems_semaphore_release( dev->semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  return 0;
}

static int dyplo_fifo_rw_get_route_id( dyplo_fifo_dev *fifo_dev )
{
  return dyplo_get_config_index( fifo_dev->config_parent ) |
         ( fifo_dev->index << 8 );
}

static int dyplo_fifo_rw_add_route(
  dyplo_fifo_dev *fifo_dev,
  int             source,
  int             dest
)
{
  dyplo_route_item_t route;

  route.srcFifo = ( source >> 8 ) & 0xFF;
  route.srcNode = source & 0xFF;
  route.dstFifo = ( dest >> 8 ) & 0xFF;
  route.dstNode = dest & 0xFF;
  int status = dyplo_ctl_route_add( fifo_dev->config_parent->parent, &route );

  return status;
}

static int dyplo_fifo_write_release( rtems_libio_open_close_args_t *args )
{
  int             status = 0;
  dyplo_fifo_dev *fifo_dev = (dyplo_fifo_dev *) args->iop->data1;
  dyplo_dev      *dev = fifo_dev->config_parent->parent;

  rtems_status_code sc = rtems_semaphore_obtain( dev->semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );

  assert( sc == RTEMS_SUCCESSFUL );

  dyplo_fifo_common_stop_read_write( fifo_dev );
  fifo_dev->is_open = false;

  sc = rtems_semaphore_release( dev->semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  return status;
}

static ssize_t dyplo_fifo_write_write( rtems_libio_rw_args_t *args )
{
  dyplo_fifo_dev *fifo_dev = (dyplo_fifo_dev *) args->iop->data1;

  if ( fifo_dev->is_closing ) {
    return -EINTR;
  }

  // IMPORTANT: do not exit this function before the semaphore is released
  // (which is done at the end of the function).
  rtems_status_code sc = rtems_semaphore_obtain(
    fifo_dev->read_write_semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  int       status = 0;
  uint32_t *fifo_memory = dyplo_fifo_memory_location( fifo_dev );
  DEBUG_PRINT( " count: %" PRIu32 ". Fifo memory @ %p",
    args->count,
    fifo_memory );
  uint32_t *buf = (uint32_t *) args->buffer;
  uint32_t  count = args->count;

  if ( count == 0 ) {
    status = 0;
    goto exit;
  }

  if ( count < 4 ) {   // Do not allow read or write below word size
    status = -EINVAL;
    goto exit;
  }

  count &= ~0x03;       // Align to words

  while ( count && !( fifo_dev->is_closing ) ) {
    uint32_t words_available = 0;

    if ( args->flags & LIBIO_FLAGS_NO_DELAY ) {         // Non-blocking I/O
      words_available = dyplo_fifo_write_level( fifo_dev );

      if ( !words_available ) {
        // Non-blocking IO, return what we have
        if ( args->bytes_moved )
          break;

        // nothing copied yet, notify caller
        status = -EAGAIN;
        goto exit;
      }
    } else {
      while ( !fifo_dev->is_closing ) {
        words_available = dyplo_fifo_write_level( fifo_dev );

        if ( words_available )
          break;                               // Done waiting

        dyplo_fifo_write_enable_interrupt( fifo_dev, count >> 2 );

        if ( !fifo_dev->is_closing ) {
          rtems_status_code sc = rtems_semaphore_obtain(
            fifo_dev->wait_semaphore,
            RTEMS_WAIT,
            RTEMS_NO_TIMEOUT );
          assert( sc == RTEMS_SUCCESSFUL );
        }
      }
    }

    if ( fifo_dev->is_closing ) {
      goto closing;
    }

    // write loop
    do {
      uint32_t bytes = words_available << 2;

      if ( bytes > DYPLO_FIFO_WRITE_MAX_BURST_SIZE )
        bytes = DYPLO_FIFO_WRITE_MAX_BURST_SIZE;

      if ( count < bytes )
        bytes = count;

      uint32_t words = bytes >> 2;

      for ( uint32_t i = 0; i < words; ++i ) {
        iowrite32( fifo_memory, buf[ i ] );
      }

      fifo_dev->words_transfered += words;
      args->bytes_moved += bytes;
      buf += words;
      count -= bytes;

      if ( !count )
        break;

      words_available -= words;
    } while ( words_available && !( fifo_dev->is_closing ) );
  }

  status = args->bytes_moved;
closing:

  if ( fifo_dev->is_closing ) {
    status = -EINTR;
  }

exit:
  sc = rtems_semaphore_release( fifo_dev->read_write_semaphore );
  assert( sc == RTEMS_SUCCESSFUL );

  return status;
}

static long dyplo_fifo_rw_ioctl( rtems_libio_ioctl_args_t *args )
{
  dyplo_fifo_dev *fifo_dev = (dyplo_fifo_dev *) args->iop->data1;

  uint32_t command = _IOC_NR( args->command );

  DEBUG_PRINT( "Command: %" PRIu32 "", command );

  if ( _IOC_TYPE( args->command ) != DYPLO_IOC_MAGIC )
    return -ENOTTY;

  int status = -1;

  switch ( command ) {
    case DYPLO_IOC_ROUTE_QUERY_ID:

      return dyplo_fifo_rw_get_route_id( fifo_dev );
      break;
    case DYPLO_IOC_ROUTE_TELL_TO_LOGIC: {
      if ( ( args->iop->flags & LIBIO_FLAGS_WRITE ) == 0 )
        return -ENOTTY;                         // Cannot route from this node, because it is a read fifo

      int dest = (int) args->buffer;

      return dyplo_fifo_rw_add_route( fifo_dev,
        dyplo_fifo_rw_get_route_id( fifo_dev ), dest );
    }
    case DYPLO_IOC_ROUTE_TELL_FROM_LOGIC: {
      if ( ( args->iop->flags & LIBIO_FLAGS_READ ) == 0 )
        return -ENOTTY;                         // Cannot route to this node, because it is a write fifo

      int src = (int) args->buffer;

      return dyplo_fifo_rw_add_route( fifo_dev,
        src,
        dyplo_fifo_rw_get_route_id( fifo_dev ) );
    }
    case DYPLO_IOC_TRESHOLD_QUERY:

      return fifo_dev->poll_treshold;
    case DYPLO_IOC_TRESHOLD_TELL: {
      unsigned int arg = (unsigned int) args->buffer;

      if ( arg < 1 )
        arg = 1;
      else if ( arg > 192 )
        arg = 192;

      fifo_dev->poll_treshold = arg;
      status = arg;
      break;
    }
    // ioctl value or type does not matter, this always resets the
    // associated fifo in the hardware.
    case DYPLO_IOC_RESET_FIFO_WRITE:
    case DYPLO_IOC_RESET_FIFO_READ:

      if ( ( args->iop->flags & LIBIO_FLAGS_WRITE ) != 0 )
        dyplo_reg_write( fifo_dev->config_parent->control_base,
          DYPLO_REG_FIFO_RESET_WRITE,
          1 << fifo_dev->index );
      else
        dyplo_reg_write( fifo_dev->config_parent->control_base,
          DYPLO_REG_FIFO_RESET_READ,
          1 << fifo_dev->index );

      status = 0;
      break;
    case DYPLO_IOC_USERSIGNAL_QUERY:

      // TODO (also linux driver): Return LAST usersignal, not next
      return fifo_dev->user_signal;
    case DYPLO_IOC_USERSIGNAL_TELL: {
      if ( !( args->iop->flags & LIBIO_FLAGS_WRITE ) )
        return -EINVAL;

      int arg = (int) args->buffer;

      arg &= 0xF;                   // Only lower 4 bits

      if ( !dyplo_fifo_write_usersignal( fifo_dev, arg ) ) {
        KERNEL_ERROR_PRINT( "Failed to set usersignal" );

        return -EIO;
      }

      fifo_dev->user_signal = arg;
      status = 0;
      break;
    }
	case DYPLO_IOC_WAIT_FOR_INCOMING_DATA:
	case DYPLO_IOC_POLL_FOR_INCOMING_DATA: {
	  struct timeval *timeout = NULL;
	  if ( command == DYPLO_IOC_POLL_FOR_INCOMING_DATA ) {
		timeout = (struct timeval *) args->buffer;
	  }

	  if ( args->iop->flags & LIBIO_FLAGS_READ ) {
		return dyplo_fifo_read_poll( fifo_dev, timeout );
	  }

      break;
	}
	case DYPLO_IOC_WAIT_FOR_OUTGOING_DATA:
	case DYPLO_IOC_POLL_FOR_OUTGOING_DATA: {
	  struct timeval *timeout = NULL;
	  if ( command == DYPLO_IOC_POLL_FOR_OUTGOING_DATA ) {
		timeout = (struct timeval *) args->buffer;
	  }

      if ( args->iop->flags & LIBIO_FLAGS_WRITE ) {
        return dyplo_fifo_write_poll( fifo_dev, timeout );
      }
    }
    default:
      ERROR_PRINT( "DYPLO ioctl unknown command: %" PRIu32 ".", command );
      status = -ENOTTY;
  }

  return status;
}

static int dyplo_fifo_write_open( rtems_libio_open_close_args_t *args )
{
  int             status = 0;
  dyplo_fifo_dev *fifo_dev = (dyplo_fifo_dev *) args->iop->data1;
  dyplo_dev      *dev = fifo_dev->config_parent->parent;

  DEBUG_PRINT( "mode=%#" PRIx32 "flags=%#" PRIx32 "", args->mode,
    args->flags );

  if ( ( args->iop->flags & LIBIO_FLAGS_READ ) ) { // write-only device
    return -EINVAL;
  }

  rtems_status_code sc = rtems_semaphore_obtain( dev->semaphore,
    RTEMS_WAIT,
    RTEMS_NO_TIMEOUT );
  assert( sc == RTEMS_SUCCESSFUL );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return -EIO;
  }

  if ( fifo_dev->is_open ) {
    status = -EBUSY;
    goto exit;
  }

  fifo_dev->user_signal = DYPLO_USERSIGNAL_ZERO;
  fifo_dev->eof = false;
  fifo_dev->is_open = true;
  fifo_dev->poll_treshold = DYPLO_FIFO_WRITE_SIZE / 2;

  // Set user signal register
  if ( !dyplo_fifo_write_usersignal( fifo_dev, DYPLO_USERSIGNAL_ZERO ) ) {
    DEBUG_PRINT( "Failed to reset usersignals on w%" PRIu32, fifo_dev->index );
    status = -EIO;
    goto exit;
  }

exit:
  rtems_semaphore_release( dev->semaphore );

  return status;
}

static uint32_t dyplo_core_get_number_of_config_devices( dyplo_dev *dev )
{
  uint32_t count1 =
    dyplo_reg_read( dev->base, DYPLO_REG_CONTROL_NODE_COUNT_1 );

  DEBUG_PRINT( "0x%08" PRIx32 "", count1 );
  uint32_t count2 =
    dyplo_reg_read( dev->base, DYPLO_REG_CONTROL_NODE_COUNT_2 );
  DEBUG_PRINT( "0x%08" PRIx32 "", count2 );

  return ( ( count1 >> 24 ) & 0xFF ) +
         ( ( count1 >> 16 ) & 0xFF ) +
         ( ( count1 >> 8 ) & 0xFF ) +
         ( ( count1 ) & 0xFF ) +
         ( ( count2 >> 24 ) & 0xFF ) +
         ( ( count2 >> 16 ) & 0xFF ) +
         ( ( count2 >> 8 ) & 0xFF ) +
         ( ( count2 ) & 0xFF );
}

// Interrupt service routine for DMA node
static void dyplo_dma_isr(
  dyplo_dev        *dev,
  dyplo_config_dev *cfg_dev
)
{
  dyplo_dma_dev *dma_dev = cfg_dev->private_data;
  uint32_t       status = dyplo_reg_read( cfg_dev->control_base,
    DYPLO_REG_FIFO_IRQ_STATUS );

  KERNEL_DEBUG_PRINT( "status=0x%" PRIx32, status );

  if ( !status )
    return;

  // Acknowledge IRQ
  iowrite32( cfg_dev->control_base + ( DYPLO_REG_FIFO_IRQ_CLR >> 2 ), status );

  // Clear the reset command when done
  if ( status & BIT( 15 ) ) {
    iowrite32( cfg_dev->control_base + ( DYPLO_DMA_TOLOGIC_CONTROL >> 2 ),
      dyplo_reg_read( cfg_dev->control_base,
        DYPLO_DMA_TOLOGIC_CONTROL ) & ~BIT( 1 ) );
  }

  if ( status & BIT( 31 ) ) {
    iowrite32( cfg_dev->control_base + ( DYPLO_DMA_FROMLOGIC_CONTROL >> 2 ),
      dyplo_reg_read( cfg_dev->control_base,
        DYPLO_DMA_FROMLOGIC_CONTROL ) & ~BIT( 1 ) );
  }

  // Wake up the proper queues
  if ( status & ( BIT( 0 ) | BIT( 15 ) ) ) {
    KERNEL_DEBUG_PRINT( "ISR release sem" );
    rtems_status_code sc = rtems_semaphore_release(
      dma_dev->write_to_logic_wait_semaphore );

    if ( sc != RTEMS_SUCCESSFUL ) {
      KERNEL_ERROR_PRINT( "Release sem failed" );
    }
  }

  if ( status & ( BIT( 16 ) | BIT( 31 ) ) ) {
    KERNEL_DEBUG_PRINT( "ISR release sem" );
    rtems_status_code sc = rtems_semaphore_release(
      dma_dev->read_from_logic_wait_semaphore );

    if ( sc != RTEMS_SUCCESSFUL ) {
      KERNEL_ERROR_PRINT( "Release sem failed" );
    }
  }
}

// Interrupt service routine for generic nodes (clear RESET command)
static void dyplo_generic_isr(
  dyplo_dev        *dev,
  dyplo_config_dev *cfg_dev
)
{
  uint32_t status = dyplo_reg_read( cfg_dev->control_base,
    DYPLO_REG_FIFO_IRQ_STATUS );

  if ( !status )
    return;

  // Acknowledge IRQ
  dyplo_reg_write( cfg_dev->control_base, DYPLO_REG_FIFO_IRQ_CLR, status );

  // Clear the reset command when done
  if ( status & BIT( 0 ) ) {
    dyplo_reg_write( cfg_dev->control_base, DYPLO_REG_NODE_RESET_FIFOS, 0 );
  }
}

static void dyplo_isr( void *dev_id )
{
  dyplo_dev *dev = (dyplo_dev *) dev_id;
  uint32_t   mask = dyplo_reg_read( dev->base, DYPLO_REG_CONTROL_IRQ_MASK );

  int index = 0;

  while ( mask ) {
    mask >>= 1;             // CPU node is '0', ctl doesn't need interrupt

    if ( mask & 1 ) {
      dyplo_config_dev *cfg_dev = &( dev->config_devices[ index ] );

      if ( cfg_dev->isr ) {
        cfg_dev->isr( dev, cfg_dev );
      }
    }

    ++index;
  }

  // For edge-triggered interrupt, re-arm by writing something.
  // This will only have effect on a PCI-e Dyplo IP.
  //dyplo_reg_write(dev->base, DYPLO_REG_CONTROL_IRQ_REARM, 1);
}

// any error that occurs during create_sub_devices_cpu_fifo(..) is handled as a fatal error,
// which is the reason why proper 'cleanup code' is 'commented out':
static int create_sub_devices_cpu_fifo(
  dyplo_config_dev                       *cfg_dev,
  func_dyplo_fifo_read_device_registered  func_register_fifo_read_dev,
  func_dyplo_fifo_write_device_registered func_register_fifo_write_dev
)
{
  int                     retval = 0;
  dyplo_fifo_control_dev *fifo_ctl_dev;
  dyplo_dev              *dev = cfg_dev->parent;

  uint32_t version_id = dyplo_cfg_get_version_id( cfg_dev );

  if ( ( version_id & DYPLO_VERSION_ID_MASK_REVISION ) != 0x0100 ) {
    KERNEL_ERROR_PRINT( "Unsupported CPU FIFO node version: %#x", version_id );
    rtems_fatal_error_occurred( RTEMS_IO_ERROR );
    //return -EINVAL;
  }

  fifo_ctl_dev = calloc( sizeof( dyplo_fifo_control_dev ), 1 );

  if ( fifo_ctl_dev == NULL ) {
    KERNEL_ERROR_PRINT( "No memory for fifo ctl dev" );
    rtems_fatal_error_occurred( RTEMS_IO_ERROR );
    //return -ENOMEM;
  }

  fifo_ctl_dev->config_parent = cfg_dev;
  cfg_dev->private_data = fifo_ctl_dev;

  uint8_t number_of_write_fifos = dyplo_number_of_output_queues( cfg_dev );
  uint8_t number_of_read_fifos = dyplo_number_of_input_queues( cfg_dev );

  if ( number_of_write_fifos + number_of_read_fifos > 0 ) {
    fifo_ctl_dev->fifo_devices =
      calloc( ( number_of_read_fifos + number_of_write_fifos ) *
        sizeof( dyplo_fifo_dev ), 1 );

    if ( fifo_ctl_dev->fifo_devices == NULL ) {
      KERNEL_ERROR_PRINT( "No memory for %d fifo devices",
        number_of_write_fifos );
      rtems_fatal_error_occurred( RTEMS_IO_ERROR );
      //retval = -ENOMEM;
      //goto error_create_fifo_devices;
    }
  }

  // create all the fifo devices (both write and read)
  for ( uint8_t i = 0; i < number_of_write_fifos + number_of_read_fifos;
        ++i ) {
    dyplo_fifo_dev *fifo_dev = &fifo_ctl_dev->fifo_devices[ i ];
    fifo_dev->config_parent = cfg_dev;

    rtems_status_code sc =
      rtems_semaphore_create( rtems_build_name( 'D', 'F', 'W',
          (char) ( '0' + dev->count_fifo_read_devices +
                   dev->count_fifo_write_devices + i ) ),
        0,                                                                                                    // initially blocked
        RTEMS_SIMPLE_BINARY_SEMAPHORE,
        0,
        &fifo_dev->wait_semaphore );

    if ( sc != RTEMS_SUCCESSFUL ) {
      KERNEL_ERROR_PRINT( "Could not create semaphore. RTEMS status code: %d",
        sc );
      rtems_fatal_error_occurred( sc );
      //retval = rtems_status_code_to_errno(sc);
      //goto error_create_semaphore;
    } else {
      KERNEL_DEBUG_PRINT( "Created semaphore DFW%d",
        dev->count_fifo_read_devices + dev->count_fifo_write_devices + i );
    }

    sc =
      rtems_semaphore_create( rtems_build_name( 'D', 'F', 'I',
          (char) ( '0' + dev->count_fifo_read_devices +
                   dev->count_fifo_write_devices + i ) ),
        1,                                                                                                    // initially not blocked
        RTEMS_SIMPLE_BINARY_SEMAPHORE,
        0,
        &fifo_dev->read_write_semaphore );

    if ( sc != RTEMS_SUCCESSFUL ) {
      KERNEL_ERROR_PRINT( "Could not create semaphore. RTEMS status code: %d",
        sc );
      rtems_fatal_error_occurred( sc );
      //retval = rtems_status_code_to_errno(sc);
      //goto error_create_read_write_semaphore;
    } else {
      KERNEL_DEBUG_PRINT( "Created semaphore DFI%d",
        dev->count_fifo_read_devices + dev->count_fifo_write_devices + i );
    }

    if ( i < number_of_write_fifos ) {
      // fifo is a write fifo
      fifo_dev->index = i;

      if ( !func_register_fifo_write_dev( fifo_dev ) ) {
        KERNEL_ERROR_PRINT( "Unable to register fifo write device" );
        rtems_fatal_error_occurred( sc );
        //retval = -EIO;
        //goto failed_device_register;
      }
    } else {
      // fifo is a read fifo
      fifo_dev->index = i - number_of_write_fifos;

      if ( !func_register_fifo_read_dev( fifo_dev ) ) {
        KERNEL_ERROR_PRINT( "Unable to register fifo read device" );
        rtems_fatal_error_occurred( sc );
        //retval = -EIO;
        //goto failed_device_register;
      }
    }
  }

  fifo_ctl_dev->number_of_fifo_write_devices = number_of_write_fifos;
  dev->count_fifo_write_devices += number_of_write_fifos;

  fifo_ctl_dev->number_of_fifo_read_devices = number_of_read_fifos;
  dev->count_fifo_read_devices += number_of_read_fifos;

  cfg_dev->isr = dyplo_fifo_isr;

/* Errors that occur during driver initialization are handled as fatal errors,
   so at the moment the cleanup code below is not used:
   failed_device_register:
        // TODO: unregister all fifo devices & semaphores created so far
   error_create_semaphore:
        free(fifo_ctl_dev->fifo_devices);
   error_create_read_write_semaphore:
        // TODO: unregister all fifo devices & semaphores created so far
   error_create_fifo_devices:
        free(fifo_ctl_dev);
 */

  return retval;
}

// any error that occurs during create_sub_devices_dma_fifo(..) is handled as a fatal error,
// which is the reason why proper 'cleanup code' is 'commented out':
static int create_sub_devices_dma_fifo(
  dyplo_config_dev                *cfg_dev,
  func_dyplo_dma_device_registered func_register_dma_dev
)
{
  dyplo_dev *dev = cfg_dev->parent;

  int retval = 0;

  dyplo_dma_dev *dma_dev;
  uint32_t       version_id = dyplo_cfg_get_version_id( cfg_dev );

  if ( ( version_id & DYPLO_VERSION_ID_MASK_REVISION ) != 0x0100 ) {
    KERNEL_ERROR_PRINT( "Unsupported DMA FIFO node revision: %#x",
      version_id );
    rtems_fatal_error_occurred( RTEMS_IO_ERROR );
    //return -EINVAL;
  }

  dma_dev = calloc( sizeof( dyplo_dma_dev ), 1 );

  if ( dma_dev == NULL ) {
    KERNEL_ERROR_PRINT( "No memory for DMA device" );
    rtems_fatal_error_occurred( RTEMS_NO_MEMORY );
    //return -ENOMEM;
  }

  cfg_dev->private_data = dma_dev;
  dma_dev->config_parent = cfg_dev;

  rtems_status_code sc =
    rtems_semaphore_create( rtems_build_name( 'D', 'D', 'W',
        ( '0' + (char) dev->number_of_dma_devices ) ),
      0,                                                                                              // initially blocked
      RTEMS_SIMPLE_BINARY_SEMAPHORE,
      0,
      &dma_dev->write_to_logic_wait_semaphore );

  if ( sc != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not create semaphore. RTEMS status code: %d",
      sc );
    rtems_fatal_error_occurred( sc );
    //retval = rtems_status_code_to_errno(sc);
    //goto error_create_semaphore_write_to_logic;
  } else {
    KERNEL_DEBUG_PRINT( "Created semaphore DDW%d",
      dev->number_of_dma_devices );
  }

  sc =
    rtems_semaphore_create( rtems_build_name( 'D', 'D', 'R',
        ( '0' + (char) dev->number_of_dma_devices ) ),
      0,                                                                                             // initially blocked
      RTEMS_SIMPLE_BINARY_SEMAPHORE,
      0,
      &dma_dev->read_from_logic_wait_semaphore );

  if ( sc != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not create semaphore. RTEMS status code: %d",
      sc );
    rtems_fatal_error_occurred( sc );
    //retval = rtems_status_code_to_errno(sc);
    //goto error_create_semaphore_read_from_logic;
  } else {
    KERNEL_DEBUG_PRINT( "Created semaphore DDR%d",
      dev->number_of_dma_devices );
  }

  sc =
    rtems_semaphore_create( rtems_build_name( 'D', 'R', 'D',
        ( '0' + (char) dev->number_of_dma_devices ) ),
      1,                                                                                             // initially not blocked
      RTEMS_SIMPLE_BINARY_SEMAPHORE,
      0,
      &dma_dev->read_semaphore );

  if ( sc != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not create semaphore. RTEMS status code: %d",
      sc );
    rtems_fatal_error_occurred( sc );
    //retval = rtems_status_code_to_errno(sc);
    //goto error_create_read_semaphore;
  } else {
    KERNEL_DEBUG_PRINT( "Created semaphore DRD%d",
      dev->number_of_dma_devices );
  }

  sc =
    rtems_semaphore_create( rtems_build_name( 'D', 'W', 'R',
        ( '0' + (char) dev->number_of_dma_devices ) ),
      1,                                                                                             // initially not blocked
      RTEMS_SIMPLE_BINARY_SEMAPHORE,
      0,
      &dma_dev->write_semaphore );

  if ( sc != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not create semaphore. RTEMS status code: %d",
      sc );
    rtems_fatal_error_occurred( sc );
    //retval = rtems_status_code_to_errno(sc);
    //goto error_create_write_semaphore;
  } else {
    KERNEL_DEBUG_PRINT( "Created semaphore DRW%d",
      dev->number_of_dma_devices );
  }

  dma_dev->dma_to_logic_memory = rtems_cache_coherent_allocate(
    dyplo_driver_init_configuration.dma_memory_size,
    DYPLO_DMA_MEMORY_ALIGNMENT,
    0 );

  if ( dma_dev->dma_to_logic_memory == NULL ) {
    KERNEL_ERROR_PRINT( "Failed alloc mem for DMA device" );
    rtems_fatal_error_occurred( RTEMS_NO_MEMORY );
    //retval = -ENOMEM;
    //goto error_dma_to_logic_alloc;
  }

  KERNEL_DEBUG_PRINT( "DMA Ringbuffer @ %p till %p ",
    dma_dev->dma_to_logic_memory,
    dma_dev->dma_to_logic_memory +
    dyplo_driver_init_configuration.dma_memory_size );
  assert( ( (uintptr_t) dma_dev->dma_to_logic_memory % 8 ) == 0 );
  dma_dev->dma_to_logic_memory_size =
    dyplo_driver_init_configuration.dma_memory_size;
  dma_dev->dma_to_logic_block_size =
    dyplo_driver_init_configuration.dma_default_block_size;

  // TODO: release somewhere
  dma_dev->dma_from_logic_memory = rtems_cache_coherent_allocate(
    dyplo_driver_init_configuration.dma_memory_size,
    DYPLO_DMA_MEMORY_ALIGNMENT,
    0 );

  if ( dma_dev->dma_from_logic_memory == NULL ) {
    KERNEL_ERROR_PRINT( "Failed alloc mem for DMA device" );
    rtems_fatal_error_occurred( RTEMS_NO_MEMORY );
    //retval = -ENOMEM;
    //goto error_dma_from_logic_alloc;
  }

  assert( ( (uintptr_t) dma_dev->dma_from_logic_memory % 8 ) == 0 );
  dma_dev->dma_from_logic_memory_size =
    dyplo_driver_init_configuration.dma_memory_size;
  dma_dev->dma_from_logic_block_size =
    dyplo_driver_init_configuration.dma_default_block_size;

  if ( !func_register_dma_dev( dma_dev ) ) {
    KERNEL_ERROR_PRINT( "Unable to create DMA device %d",
      dev->number_of_dma_devices );
    rtems_fatal_error_occurred( RTEMS_IO_ERROR );
    //retval = -EIO;
    //goto failed_device_create;
  }

  dev->number_of_dma_devices++;

  // Enable the DMA controller
  iowrite32( cfg_dev->control_base + ( DYPLO_DMA_TOLOGIC_CONTROL >> 2 ), BIT(
      0 ) );
  iowrite32( cfg_dev->control_base + ( DYPLO_DMA_FROMLOGIC_CONTROL >> 2 ),
    BIT( 0 ) );
  cfg_dev->isr = dyplo_dma_isr;

/* Errors that occur during driver initialization are handled as fatal errors,
   so at the moment the cleanup code below is not used:
   failed_device_create:
        free(dma_dev->dma_from_logic_memory);
   error_dma_from_logic_alloc:
        free(dma_dev->dma_to_logic_memory);
   error_dma_to_logic_alloc:
        rtems_semaphore_release(dma_dev->write_semaphore);
   error_create_write_semaphore:
        rtems_semaphore_release(dma_dev->read_semaphore);
   error_create_read_semaphore:
        rtems_semaphore_release(dma_dev->read_from_logic_wait_semaphore);
   error_create_semaphore_read_from_logic:
        rtems_semaphore_release(dma_dev->write_to_logic_wait_semaphore);
   error_create_semaphore_write_to_logic:
        free(dma_dev); */

  return retval;
}

static int create_sub_devices_icap( dyplo_config_dev *cfg_dev )
{
  dyplo_dev   *dev = cfg_dev->parent;
  unsigned int device_index = dyplo_get_config_index( cfg_dev );

  cfg_dev->isr = dyplo_generic_isr;
  dev->icap_device_index = device_index;

  return 0;
}

static int create_sub_devices(
  dyplo_config_dev                       *cfg_dev,
  func_dyplo_fifo_read_device_registered  func_register_fifo_read_dev,
  func_dyplo_fifo_write_device_registered func_register_fifo_write_dev,
  func_dyplo_dma_device_registered        func_register_dma_dev
)
{
  switch ( dyplo_cfg_get_node_type( cfg_dev ) ) {
    case DYPLO_TYPE_ID_TOPIC_CPU:

      return create_sub_devices_cpu_fifo( cfg_dev,
      func_register_fifo_read_dev,
      func_register_fifo_write_dev );
    case DYPLO_TYPE_ID_TOPIC_DMA:

      return create_sub_devices_dma_fifo( cfg_dev,
      func_register_dma_dev );
    case DYPLO_TYPE_ID_TOPIC_ICAP:

      return create_sub_devices_icap( cfg_dev );
    default:
      cfg_dev->isr = dyplo_generic_isr;

      return 0;
  }
}

static const char *dyplo_type_names[] = {
  [ DYPLO_TYPE_ID_TOPIC_CPU ] = "CPU",
  [ DYPLO_TYPE_ID_TOPIC_IO ] = "IO",
  [ DYPLO_TYPE_ID_TOPIC_FIXED ] = "FIXED",
  [ DYPLO_TYPE_ID_TOPIC_PR ] = "PR",
  [ DYPLO_TYPE_ID_TOPIC_DMA ] = "DMA",
  [ DYPLO_TYPE_ID_TOPIC_ICAP ] = "ICAP",
};

static const char *dyplo_get_type_name( uint8_t type_id )
{
  const char *result;

  if ( type_id >= RTEMS_ARRAY_SIZE( dyplo_type_names ) )
    return "";

  result = dyplo_type_names[ type_id ];

  if ( !result )
    return "";

  return result;
}

// any error that occurs during dyplo_core_probe(..) is handled as a fatal error,
// which is the reason why proper 'cleanup code' is 'commented out':
int dyplo_core_probe(
  dyplo_dev                              *dev,
  func_dyplo_control_device_registered    func_register_ctrl_dev,
  func_dyplo_config_device_registered     func_register_cfg_dev,
  func_dyplo_fifo_read_device_registered  func_register_fifo_read_dev,
  func_dyplo_fifo_write_device_registered func_register_fifo_write_dev,
  func_dyplo_dma_device_registered        func_register_dma_dev
)
{
  KERNEL_DEBUG_PRINT( "Probing Dyplo.." );

  int               retval = 0;
  rtems_name        semaphore_name = rtems_build_name( 'D', 'Y', 'P', '1' );
  rtems_status_code sc = rtems_semaphore_create( semaphore_name,
    1,
    RTEMS_BINARY_SEMAPHORE | RTEMS_INHERIT_PRIORITY | RTEMS_PRIORITY,
    RTEMS_NO_PRIORITY,
    &dev->semaphore );

  if ( sc != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Semaphore creation failed. RTEMS statuscode: %d",
      sc );
    rtems_fatal_error_occurred( sc );
    //return rtems_status_code_to_errno(sc);
  }

  bool succes = func_register_ctrl_dev( dev );

  uint32_t version = dyplo_ctl_get_dyplo_version_id( dev );
  uint8_t  dyplo_version = version >> 8;
  uint8_t  dyplo_release = ( 0xF0 & version ) >> 4;
  uint8_t  dyplo_build = 0x0F & version;
  KERNEL_INFO_PRINT( "Dyplo Version: Version: %d.%d.%d",
    dyplo_version,
    dyplo_release,
    dyplo_build );

  dev->number_of_config_devices =
    dyplo_core_get_number_of_config_devices( dev );
  KERNEL_INFO_PRINT( "%d config devices detected",
    dev->number_of_config_devices );

  // allocate memory for config devices
  dev->config_devices =
    calloc( dev->number_of_config_devices * sizeof( dyplo_config_dev ), 1 );

  if ( dev->config_devices == NULL ) {
    KERNEL_ERROR_PRINT( "Unable to allocate memory for config devices" );
    rtems_fatal_error_occurred( RTEMS_NO_MEMORY );
    //retval = -ENOMEM;
    //goto error_config_device_creation_failed;
  } else {
    uint8_t device_index = 0;

    // register all cfg devices
    while ( device_index < dev->number_of_config_devices ) {
      dyplo_config_dev *cfg_dev = &( dev->config_devices[ device_index ] );
      cfg_dev->parent = dev;
      cfg_dev->base =
        ( dev->base + ( ( DYPLO_CONFIG_SIZE >> 2 ) * ( device_index + 1 ) ) );
      cfg_dev->control_base =
        ( dev->base +
          ( ( DYPLO_NODE_REG_SIZE >> 2 ) * ( device_index + 1 ) ) );
      cfg_dev->node_type = dyplo_cfg_get_node_type( cfg_dev );
      KERNEL_INFO_PRINT( "Node %" PRIu8 ". Node type: %d (%s)",
        device_index,
        cfg_dev->node_type,
        dyplo_get_type_name( cfg_dev->node_type ) );

      succes = func_register_cfg_dev( cfg_dev );

      if ( !succes ) {
        rtems_fatal_error_occurred( RTEMS_IO_ERROR );
        //retval = -EIO;
        //goto error_config_device_registration_failed;
      }

      ++device_index;

      int status = create_sub_devices( cfg_dev,
        func_register_fifo_read_dev,
        func_register_fifo_write_dev,
        func_register_dma_dev );

      if ( status != 0 ) {
        KERNEL_ERROR_PRINT( "Unable to create sub-devices for cfg device %d",
          device_index );
        rtems_fatal_error_occurred( RTEMS_IO_ERROR );
        //retval = -EIO;
        //goto error_config_device_registration_failed;
      }
    }
  }

  KERNEL_DEBUG_PRINT( "Install interrupt handler" );

  sc = rtems_interrupt_handler_install( dev->irq,
    NULL,
    RTEMS_INTERRUPT_UNIQUE,
    dyplo_isr,
    dev );

  if ( sc != RTEMS_SUCCESSFUL ) {
    KERNEL_ERROR_PRINT( "Could not install interrupt handler @ IRQ %d",
      dev->irq );
    rtems_fatal_error_occurred( sc );
    //retval = rtems_status_code_to_errno(sc);
    //goto error_interrupt_handler_install_failed;
  }

  // enable the backplane
  dyplo_reg_write( dev->base,
    DYPLO_REG_BACKPLANE_ENABLE_SET,
    ( 2 << dev->number_of_config_devices ) - 1 );

/* Errors that occur during driver initialization are handled as fatal errors,
   so at the moment the cleanup code below is not used:
   error_interrupt_handler_install_failed:
   error_config_device_registration_failed:
        // TODO: unregister all config devices registered so far and release their semaphores

   error_config_device_creation_failed:
        sc = rtems_semaphore_delete(semaphore_name);
 */

  return retval;
}

int dyplo_device_read( rtems_libio_rw_args_t *args )
{
  dyplo_device_type type = args->iop->data0;

  int status;

  switch ( type ) {
    case DYPLO_DRIVER_TYPE_DMA:
      status = dyplo_dma_read( args );
      break;
    case DYPLO_DRIVER_TYPE_FIFO_WRITE:
      status = -EINVAL;
      break;
    case DYPLO_DRIVER_TYPE_FIFO_READ:
      status = dyplo_fifo_read_read( args );
      break;
    case DYPLO_DRIVER_TYPE_CFG:
      status = dyplo_cfg_read( args );
      break;
    default:
      assert( false );
      status = -EINVAL;
  }

  return status;
}

int dyplo_device_write( rtems_libio_rw_args_t *args )
{
  dyplo_device_type type = args->iop->data0;

  int status;

  switch ( type ) {
    case DYPLO_DRIVER_TYPE_FIFO_READ:
      status = -EINVAL;
      break;
    case DYPLO_DRIVER_TYPE_FIFO_WRITE:
      status = dyplo_fifo_write_write( args );
      break;
    case DYPLO_DRIVER_TYPE_CFG:
      status = dyplo_cfg_write( args );
      break;
    case DYPLO_DRIVER_TYPE_DMA:
      status = dyplo_dma_write( args );
      break;
    default:
      assert( false );
      status = -EINVAL;
  }

  return status;
}

int dyplo_device_open( rtems_libio_open_close_args_t *args )
{
  dyplo_device_type type = args->iop->data0;

  int status;

  switch ( type ) {
    case DYPLO_DRIVER_TYPE_CTL:
      status = dyplo_ctl_open( args );
      break;
    case DYPLO_DRIVER_TYPE_FIFO_READ:
      status = dyplo_fifo_read_open( args );
      break;
    case DYPLO_DRIVER_TYPE_FIFO_WRITE:
      status = dyplo_fifo_write_open( args );
      break;
    case DYPLO_DRIVER_TYPE_DMA:
      status = dyplo_dma_open( args );
      break;
    case DYPLO_DRIVER_TYPE_CFG:
      status = dyplo_cfg_open( args );
      break;
    default:
      assert( false );
      status = -EINVAL;
  }

  return status;
}

int dyplo_device_close( rtems_libio_open_close_args_t *args )
{
  dyplo_device_type type = args->iop->data0;

  int status;

  switch ( type ) {
    case DYPLO_DRIVER_TYPE_CTL:
      status = dyplo_ctl_release( args );
      break;
    case DYPLO_DRIVER_TYPE_FIFO_READ:
      status = dyplo_fifo_read_release( args );
      break;
    case DYPLO_DRIVER_TYPE_FIFO_WRITE:
      status = dyplo_fifo_write_release( args );
      break;
    case DYPLO_DRIVER_TYPE_DMA:
      status = dyplo_dma_release( args );
      break;
    case DYPLO_DRIVER_TYPE_CFG:
      status = dyplo_cfg_release( args );
      break;
    default:
      assert( false );
      status = -EINVAL;
  }

  return status;
}

int dyplo_device_control( rtems_libio_ioctl_args_t *args )
{
  dyplo_device_type type = args->iop->data0;

  int status;

  switch ( type ) {
    case DYPLO_DRIVER_TYPE_CTL:
      status = dyplo_ctl_ioctl( args );
      break;
    case DYPLO_DRIVER_TYPE_DMA:
      status = dyplo_dma_ioctl( args );
      break;
    case DYPLO_DRIVER_TYPE_CFG:
      status = dyplo_cfg_ioctl( args );
      break;
    case DYPLO_DRIVER_TYPE_FIFO_READ:
    case DYPLO_DRIVER_TYPE_FIFO_WRITE:
      status = dyplo_fifo_rw_ioctl( args );
      break;
	default:
      status = -EINVAL;
  }

  return status;
}
