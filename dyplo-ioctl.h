/**
 * @file dyplo-ioctl.h
 *
 * @ingroup DyploDriver
 *
 * @brief IO controls that can be used to interact with the DyploDriver.
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

#ifndef DYPLO_IOCTL_H
#define DYPLO_IOCTL_H

#ifdef __rtems__
typedef __uint32_t __u32;
typedef __uint16_t __u16;
#endif

/* ioctl values for dyploctl device, set and get routing tables */
typedef struct dyplo_route_item_t {
  unsigned char dstFifo;       /* LSB */
  unsigned char dstNode;
  unsigned char srcFifo;
  unsigned char srcNode;       /* MSB */
} dyplo_route_item_t;

typedef struct dyplo_route_t {
  unsigned int n_routes;       /* Number of entries in proutes */
  struct dyplo_route_item_t *proutes;       /* Size must be at least n_routes */
} dyplo_route_t;

typedef struct dyplo_buffer_block_alloc_req {
  __u32 size;           /* Size of each buffer */
  __u32 count;          /* Number of buffers */
} dyplo_buffer_block_alloc_req;

typedef struct dyplo_buffer_block {
  __u32 id;             /* 0-based index of the buffer */
  __u32 offset;         /* Location of data in memory map.
                                           NOTE: On platforms where no virtual memory is used (like RTEMS), "offset"
                                           represents the memory address of the block. */
  __u32 size;           /* Size of buffer. May be larger than requested. */
  __u32 bytes_used;       /* How much actually is in use */
  __u16 user_signal;       /* User signals (framing) either way */
  __u16 state;       /* Who's owner of the buffer */
} dyplo_buffer_block;

typedef struct dyplo_dma_standalone_config {
  __u32 offset;
  __u32 burst_size;
  __u32 incr_a;
  __u32 iterations_a;
  __u32 incr_b;
  __u32 iterations_b;
  __u32 incr_c;
  __u32 iterations_c;
} dyplo_dma_standalone_config;

/* DMA not used for CPU-logic transfers at all, only for logic
 * storage. Buffer can be mmap'ed for inspection. */
#define DYPLO_DMA_MODE_STANDALONE 0
/* (default) Copies data from userspace into a kernel buffer and
 * vice versa. */
#define DYPLO_DMA_MODE_RINGBUFFER_BOUNCE 1
/* Blockwise data transfers, using coherent memory. This will result in
 * slow non-cached memory being used when hardware coherency is not
 * available, but it is the fastest mode. */
#define DYPLO_DMA_MODE_BLOCK_COHERENT 2
/* Blockwise data transfers, using  streaming DMA into cachable memory.
 * Managing the cache may cost more than actually copying the data.
 * Requires that the platform implements cache control functions. */
#define DYPLO_DMA_MODE_BLOCK_STREAMING 3

typedef struct dyplo_dma_configuration_req {
  __u32 mode;           /* One of DYPLO_DMA_MODE.. */
  __u32 size;           /* Requested minimal size of each buffer */
  __u32 count;          /* Number of buffers */
} dyplo_dma_configuration_req;

#define DYPLO_IOC_MAGIC 'd'
#define DYPLO_IOC_ROUTE_CLEAR 0x00
#define DYPLO_IOC_ROUTE_SET 0x01
#define DYPLO_IOC_ROUTE_GET 0x02
#define DYPLO_IOC_ROUTE_TELL 0x03
#define DYPLO_IOC_ROUTE_DELETE 0x04
#define DYPLO_IOC_ROUTE_TELL_TO_LOGIC 0x05
#define DYPLO_IOC_ROUTE_TELL_FROM_LOGIC 0x06
#define DYPLO_IOC_ROUTE_QUERY_ID 0x07

#define DYPLO_IOC_BACKPLANE_STATUS 0x08
#define DYPLO_IOC_BACKPLANE_DISABLE 0x09
#define DYPLO_IOC_BACKPLANE_ENABLE 0x0A

#define DYPLO_IOC_ICAP_INDEX_QUERY 0x0B

#define DYPLO_IOC_RESET_FIFO_WRITE 0x0C
#define DYPLO_IOC_RESET_FIFO_READ 0x0D

#define DYPLO_IOC_TRESHOLD_QUERY 0x10
#define DYPLO_IOC_TRESHOLD_TELL 0x11

#define DYPLO_IOC_USERSIGNAL_QUERY 0x12
#define DYPLO_IOC_USERSIGNAL_TELL 0x13

#define DYPLO_IOC_DMA_RECONFIGURE 0x1F
#define DYPLO_IOC_DMABLOCK_ALLOC 0x20        // Deprecated interface. Reserved number (IOCTL numbers may not be re-used)
#define DYPLO_IOC_DMABLOCK_FREE 0x21
#define DYPLO_IOC_DMABLOCK_QUERY 0x22
#define DYPLO_IOC_DMABLOCK_ENQUEUE 0x23
#define DYPLO_IOC_DMABLOCK_DEQUEUE 0x24
#define DYPLO_IOC_DMASTANDALONE_CONFIGURE_TO_LOGIC 0x28
#define DYPLO_IOC_DMASTANDALONE_CONFIGURE_FROM_LOGIC 0x29
#define DYPLO_IOC_DMASTANDALONE_START_TO_LOGIC 0x2A
#define DYPLO_IOC_DMASTANDALONE_START_FROM_LOGIC 0x2B
#define DYPLO_IOC_DMASTANDALONE_STOP_TO_LOGIC 0x2C
#define DYPLO_IOC_DMASTANDALONE_STOP_FROM_LOGIC 0x2D

#define DYPLO_IOC_LICENSE_KEY 0x30
#define DYPLO_IOC_STATIC_ID 0x31

#define DYPLO_IOC_POLL_FOR_INCOMING_DATA 0x40
#define DYPLO_IOC_POLL_FOR_OUTGOING_DATA 0x41
#define DYPLO_IOC_WAIT_FOR_INCOMING_DATA 0x42
#define DYPLO_IOC_WAIT_FOR_OUTGOING_DATA 0x43

/**
 *  NOTE: to see how RTEMS maps rtems_status_code to the returned errno, see:
 *  rtems/src/cpukit/rtems/src/status.c
 */

/* S means "Set" through a ptr,
 * T means "Tell", sets directly
 * G means "Get" through a ptr
 * Q means "Query", return value */

/**
 * @brief Delete all existing routes
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCROUTE_CLEAR _IO( DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_CLEAR )
/**
 * @brief Define a set of routes, to be added to the currently active set
 * @param Pointer to a struct dyplo_route_t that contains the routes to be added
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCSROUTE _IOW( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_ROUTE_SET, \
  struct dyplo_route_t )
/**
 * @brief Get the currently active routes
 * @param Pointer to a struct dyplo_route_t that will be filled with information
 *        n_routes must be set to the number of entries allocated
 * @return -1 in case of error, number of active routes on success. If the
 *         n_routes value is less, only up to n_routes entries will have been
 *         provided.
 */
#define DYPLO_IOCGROUTE _IOR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_ROUTE_GET, \
  struct dyplo_route_t )
/**
 * @brief Add a single route
 * @param a dyplo_route_item_t, re-interpret cast to integer
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCTROUTE _IO( DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_TELL )
/**
 * @brief Remove routes to a node
 * @param integer node number
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCTROUTE_DELETE _IO( DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_DELETE )
/**
 * @brief Add a route from "this" dma or cpu node to another node
 * @param integer encoded as (node number) | (fifo << 8)
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCTROUTE_TELL_TO_LOGIC _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_ROUTE_TELL_TO_LOGIC )
/* Add a route dma or cpu node. Argument
 * is an integer of source node | fifo << 8 */
/**
 * @brief Add a route from another node into "this" dma or cpu node
 * @param integer encoded as (node number) | (fifo << 8)
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCTROUTE_TELL_FROM_LOGIC _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_ROUTE_TELL_FROM_LOGIC )
/**
 * @brief Get the node number and fifo (if applicable) for this cpu or dma node
 * @return -1 in case of error, Returns an integer encoded as (node) | (fifo << 8)
 */
#define DYPLO_IOCQROUTE_QUERY_ID _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_ROUTE_QUERY_ID )

/**
 * @brief Get backplane status. When called on control node, returns a bit mask
 *        that has the corresponding bit "1" for all active nodes. When called
 *        on a config node, returns the status for only that node, 0=disabled
 *        non-zero is enabled.
 * @return -1 in case of error, value as described above otherwise
 */
#define DYPLO_IOCQBACKPLANE_STATUS _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_BACKPLANE_STATUS )
/**
 * @brief Enable backplane status for a node.
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCTBACKPLANE_ENABLE _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_BACKPLANE_ENABLE )
/**
 * @brief Disable backplane status. Disable is required when the logic
 *        is active and you want to replace a node using partial configuration.
 *        Operations are atomic.
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCTBACKPLANE_DISABLE _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_BACKPLANE_DISABLE )
/**
 * @brief Get ICAP index.
 * @return -1 in case of error, node index of the ICAP node on success.
 * @exception ENODEV if no ICAP available in logic
 */
#define DYPLO_IOCQICAP_INDEX _IO( DYPLO_IOC_MAGIC, DYPLO_IOC_ICAP_INDEX_QUERY )
/* . */
/**
 * @brief Get the threshold for "writeable" or "readable" on a CPU node fifo.
 * @return -1 in case of error, current value on success.
 */
#define DYPLO_IOCQTRESHOLD _IO( DYPLO_IOC_MAGIC, DYPLO_IOC_TRESHOLD_QUERY )
/**
 * @brief Set the thresholds for "writeable" or "readable" on a CPU node fifo.
 *        Allows tuning for low latency or reduced interrupt rate. For example,
 *        when set to "10" a poll() or select() will indicate readiness when
 *        at least 10 items are available for reading or writing without
 *        blocking. Hardware limits apply.
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCTTRESHOLD _IO( DYPLO_IOC_MAGIC, DYPLO_IOC_TRESHOLD_TELL )
/**
 * @brief Reset outgoing FIFO data (i.e. throw it away).
 * @param When applied to config nodes: bitmask for queues to reset.
 *        When applied to a CPU read/write fifo: Ignored.
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCRESET_FIFO_WRITE _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_RESET_FIFO_WRITE )
/**
 * @brief Reset incoming FIFO data (i.e. throw it away).
 * @param When applied to config nodes: bitmask for queues to reset.
 *        When applied to a CPU read/write fifo: Ignored.
 * @return -1 in case of error, 0 on success.
 */
#define DYPLO_IOCRESET_FIFO_READ _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_RESET_FIFO_READ )

/**
 * @brief Get user signal bits of cpu/dma fifo.
 *        User signal is the upper 4 bits of Dyplo data
 *        that aren't part of the actual data, but control the flow.
 * @return -1 in case of error, check errno. The user signal (int).
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCQUSERSIGNAL _IO( DYPLO_IOC_MAGIC, DYPLO_IOC_USERSIGNAL_QUERY )

/**
 * @brief Set user signal bits of cpu/dma fifo (opened in write mode).
 *        User signal is the upper 4 bits of Dyplo data
 *        that aren't part of the actual data, but control the flow.
 * @param user signal (int).
 * @return -1 in case of error, check errno. 0 in case of success.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCTUSERSIGNAL _IO( DYPLO_IOC_MAGIC, DYPLO_IOC_USERSIGNAL_TELL )

/**
 * @brief Reconfigure the DMA node with a mode, buffer size and buffer amount.
 * @param (struct dyplo_dma_configuration_req) DMA configuration request.
 *        In case of DYPLO_DMA_MODE_RINGBUFFER_BOUNCE, size and count members are ignored.
 * @return -1 in case of error, check errno.
 *         0 in case of success, in case of mode DYPLO_DMA_MODE_RINGBUFFER_BOUNCE, size and count member will be updated with the values used by the driver.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this or in case of invalid argument.
 *                           If you tried to reconfigure as DYPLO_DMA_MODE_STANDALONE, but the dma device was not opened with O_SYNC.
 *                           If the size of the blocks from param is not 64-bit aligned.
 */
#define DYPLO_IOCDMA_RECONFIGURE _IOWR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMA_RECONFIGURE, \
  struct dyplo_dma_configuration_req )

/**
 * @brief Free DMA write or read buffers.
 * @return -1 in case of error, check errno. 0 in case of success.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCDMABLOCK_FREE _IO( DYPLO_IOC_MAGIC, DYPLO_IOC_DMABLOCK_FREE )

/**
 * @brief Get information about a DMA buffer block from either a dma device opened for writing, or a dma device opened for reading.
 * @param (struct dyplo_buffer_block) buffer information. dyplo_buffer_block->id is used to determine which buffer block are you requesting information about.
 * @return -1 in case of error, check errno. 0 in case of success, the parameter will be filled with buffer information.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 *                           In case of invalid argument.
 *                           If the buffer block does not exist.
 */
#define DYPLO_IOCDMABLOCK_QUERY _IOWR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMABLOCK_QUERY, \
  struct dyplo_buffer_block )

/**
 * @brief Enqueue a DMA buffer block (transfer ownership to Dyplo Driver) to either a dma device opened for writing, or a dma device opened for reading.
 *        For more information see libdyplo.
 * @param (struct dyplo_buffer_block) buffer information. dyplo_buffer_block->bytes_used is used to communicate how much data is prepared for the DMA node (needs to be 64-byte aligned).
 * @return -1 in case of error, check errno. 0 in case of success.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 *                           In case of invalid argument.
 *                           If the buffer block does not exist.
 */
#define DYPLO_IOCDMABLOCK_ENQUEUE _IOWR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMABLOCK_ENQUEUE, \
  struct dyplo_buffer_block )

/**
 * @brief Dequeue a DMA buffer block (transfer ownership to user) from either a dma device opened for writing, or a dma device opened for reading.
 *        Will block until buffer block is available. For more information see libdyplo.
 * @param (struct dyplo_buffer_block) buffer information.
 * @return -1 in case of error, check errno. 0 in case of success, buffer information is the buffer you have dequeued.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 *                           In case of invalid argument.
 *                           If the buffer block does not exist.
 */
#define DYPLO_IOCDMABLOCK_DEQUEUE _IOWR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMABLOCK_DEQUEUE, \
  struct dyplo_buffer_block )

/**
 * @brief Configure DMA standalone mode in the direction Dyplo Driver to Dyplo Logic. Use this on a dma device opened in standalone mode.
 * @param (struct dyplo_dma_standalone_config) DMA standalone configuration.
 * @return -1 in case of error, check errno. 0 in case of success.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCSDMASTANDALONE_CONFIGURE_TO_LOGIC _IOW( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMASTANDALONE_CONFIGURE_TO_LOGIC, \
  struct dyplo_dma_standalone_config )

/**
 * @brief Get DMA standalone mode configuration in the direction Dyplo Driver to Dyplo Logic. Use this on a dma device opened in standalone mode.
 * @param (struct dyplo_dma_standalone_config) DMA standalone configuration.
 * @return -1 in case of error, check errno. 0 in case of success, parameter will be filled with the actual configuration.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCGDMASTANDALONE_CONFIGURE_TO_LOGIC _IOR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMASTANDALONE_CONFIGURE_TO_LOGIC, \
  struct dyplo_dma_standalone_config )

/**
 * @brief Configure DMA standalone mode in the direction Dyplo Logic to Dyplo Driver. Use this on a dma device opened in standalone mode.
 * @param (struct dyplo_dma_standalone_config) DMA standalone configuration.
 * @return -1 in case of error, check errno. 0 in case of success.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCSDMASTANDALONE_CONFIGURE_FROM_LOGIC _IOW( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMASTANDALONE_CONFIGURE_FROM_LOGIC, \
  struct dyplo_dma_standalone_config )

/**
 * @brief Get DMA standalone mode configuration in the direction Dyplo Logic to Dyplo Driver. Use this on a dma device opened in standalone mode.
 * @param (struct dyplo_dma_standalone_config) DMA standalone configuration.
 * @return -1 in case of error, check errno. 0 in case of success, parameter will be filled with the actual configuration.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCGDMASTANDALONE_CONFIGURE_FROM_LOGIC _IOR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMASTANDALONE_CONFIGURE_FROM_LOGIC, \
  struct dyplo_dma_standalone_config )

/**
 * @brief Start DMA standalone mode in the direction Dyplo Driver to Dyplo Logic. Use this on a dma device opened in standalone mode.
 * @return -1 in case of error, check errno. 0 in case of success.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCDMASTANDALONE_START_TO_LOGIC _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMASTANDALONE_START_TO_LOGIC )

/**
 * @brief Start DMA standalone mode in the direction Dyplo Logic to Dyplo Driver. Use this on a dma device opened in standalone mode.
 * @return -1 in case of error, check errno. 0 in case of success.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCDMASTANDALONE_START_FROM_LOGIC _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMASTANDALONE_START_FROM_LOGIC )

/**
 * @brief Stop DMA standalone mode in the direction Dyplo Driver to Dyplo Logic. Use this on a dma device opened in standalone mode.
 * @return -1 in case of error, check errno. 0 in case of success.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCDMASTANDALONE_STOP_TO_LOGIC _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMASTANDALONE_STOP_TO_LOGIC )

/**
 * @brief Stop DMA standalone mode in the direction Dyplo Logic to Dyplo Driver. Use this on a dma device opened in standalone mode.
 * @return -1 in case of error, check errno. 0 in case of success.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCDMASTANDALONE_STOP_FROM_LOGIC _IO( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_DMASTANDALONE_STOP_FROM_LOGIC )

/**
 * @brief Set 64-bit license key. Only to be used on the dyplo control device.
 * @return -1 in case of error, check errno.
 *         0 in case of success, parameter will contain the license key.
 * @param (unsigned long long) license key to set.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this or in case
 *                           of invalid argument.
 */
#define DYPLO_IOCSLICENSE_KEY _IOW( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_LICENSE_KEY, \
  unsigned long long )

/**
 * @brief Read 64-bit license key. Only to be used on the dyplo control device.
 * @return -1 in case of error, check errno.
 *         0 in case of success, parameter will contain the license key.
 * @param (unsigned long long) license key
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this or in case
 *                           of invalid argument.
 */
#define DYPLO_IOCGLICENSE_KEY _IOR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_LICENSE_KEY, \
  unsigned long long )

/**
 * @brief Retrieve static ID (to match against partials). Only to be used on the dyplo control device.
 * @return -1 in case of error, check errno.
 *         0 in case of success, parameter will contain the static ID.
 * @param (unsigned int) static id
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this or in case
 *                           of invalid argument.
 *                           If you are using a old Dyplo version (< 2015.1.4).
 */
#define DYPLO_IOCGSTATIC_ID _IOR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_STATIC_ID, \
  unsigned int )

/**
 * @brief Poll for incoming data. To be used on cpu/dma device opened for reading.
 * @return -1 in case of error, check errno. 0 in case there is no data available for reading.
 *         >0 in case data is available for reading.
 * @param (struct timeval) the maximum amount of time to wait for data to be available.
 *        In case no waiting should be performed, leave this at 0.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this or in case
 *                           of invalid argument.
 */
#define DYPLO_IOCPOLLFOR_INCOMING_DATA _IOR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_POLL_FOR_INCOMING_DATA, \
  struct timeval )

/**
 * @brief Poll for available space in the write buffer. To be used on cpu/dma device opened for writing.
 * @return -1 in case of error, check errno.
 *         0 in case data in case there is no space available for writing.
 *         >0 in case space is available for writing.
 * @param (struct timeval) the maximum amount of time to wait for space to be available.
 *        In case no waiting should be performed, leave this at 0.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this or in case
 *                           of invalid argument.
 */
#define DYPLO_IOCPOLLFOR_OUTGOING_DATA _IOR( DYPLO_IOC_MAGIC, \
  DYPLO_IOC_POLL_FOR_OUTGOING_DATA, \
  struct timeval )


/**
 * @brief Poll and wait for incoming data. To be used on cpu/dma device opened for reading.
 *        Blocks until there is data available, or the file descriptor is closed.
 * @return -1 in case of error, check errno. 0 in case there is no data available for reading.
 *         >0 in case data is available for reading.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCWAITFOR_INCOMING_DATA	_IO(DYPLO_IOC_MAGIC, DYPLO_IOC_WAIT_FOR_INCOMING_DATA)

/**
 * @brief Poll and wait for available space in the write buffer. To be used on cpu/dma device opened for writing.
 *        Blocks until there is space available, or the file descriptor is closed.
 * @return -1 in case of error, check errno.
 *         0 in case data in case there is no space available for writing.
 *         >0 in case space is available for writing.
 * @exception RTEMS_IO_ERROR If the file descriptor does not support this.
 */
#define DYPLO_IOCWAITFOR_OUTGOING_DATA	_IO(DYPLO_IOC_MAGIC, DYPLO_IOC_WAIT_FOR_OUTGOING_DATA)

#endif
