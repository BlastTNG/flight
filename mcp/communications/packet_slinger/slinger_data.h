/**
 * @file slinger_data.h
 *
 * @date Feb 18, 2011
 * @author seth
 * 
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2010 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifndef SLINGER_DATA_H_
#define SLINGER_DATA_H_
#include <time.h>
#include <stdint.h>
#include <pthread.h>
#include <lzma.h>
#include <glib.h>

#include <lookup.h>
#include <blast_packet_format.h>
#include <fifo.h>
#include <pq.h>
#include <packet_slinger.h>
#include <slinger_time.h>

#define SLINGER_MAX_SMALL_PAYLOAD_SIZE (1024) /**< SLINGER_MAX_SMALL_PAYLOAD_SIZE 2^10 Governs the maximum size of multi-packets */
#define SLINGER_MAX_SEGMENT_SIZE 255
#define SLINGER_MAX_PAYLOAD_SIZE 65535
#define SLINGER_MAX_FILE_DL_SIZE SLINGER_MAX_PAYLOAD_SIZE    /**< SLINGER_MAX_FILE_DL_SIZE defines the maximum size in bytes of a file to downlink */

#define SLINGER_MAX_PRIORITY 255
#define SLINGER_MAX_QUEUE_LEN 50
#define FILE_HASH 0

#define _SLINGER_CACHE_TYPES(x,_)    \
    _(x, BUNDLE)                    \
    _(x, STREAM)                    \
    _(x, PERIODIC)                    \
    _(x, FUNCTION)
BLAST_LOOKUP_TABLE(SLINGER_CACHE_TYPE, static);

typedef struct slinger_cache_node slinger_cache_node_t;

typedef struct slinger_dl_node
{
    uint8_t                 priority;
    uint32_t                id;
    size_t                  size;
    uint8_t                 type;
    bool                    qos;
    void                    *stream;
    blast_master_packet_t   *data;
} slinger_dl_node_t;

typedef struct slinger_periodic_downlink
{
    uint32_t                hash;           /**< hash The hash (without mods) of the periodic data name */
    slinger_time_t          time;           /**< time The BLAST timestamp value for the starting time */
    uint32_t                value;          /**< value The previous value of the node.  If the reading changes from this value, a packet is generated */
} __attribute__((packed)) slinger_periodic_downlink_t;

typedef struct slinger_periodic_node
{
    uint8_t                 priority;
    time_t                  last_sent;      /**< last_sent Last time this value was downlinked */
    uint16_t                timeout;        /**< timeout Maximum number of seconds between packet downlink, whether or not it changes */

    slinger_periodic_downlink_t    downlink;

} slinger_periodic_node_t;

/**
 * Defines a stream packet before compression
 */
typedef struct slinger_stream_downlink
{
    uint32_t        hash;       /**< hash The hash (without mods) of the stream name */
    slinger_time_t  time;       /**< time The starting timestamp of the node.  This is BLAST time */
    uint16_t        produced;   /**< produced Number of bytes in #buffer ready for compression/downlink */
    uint8_t         buffer[];   /**< buffer Holds cached data */
}__attribute__((packed)) slinger_stream_downlink_t;

typedef struct slinger_stream_node
{
    slinger_cache_node_t        *bundle;        /**< bundle Pointer to the bundle's cache node */
    uint16_t                    packetsize;     /**< packetsize Number of bytes allocated for #buffer */
    int64_t                     delta_t;        /**< delta_t == 1/rate in 1e-5 seconds */
    slinger_stream_downlink_t   downlink;
} slinger_stream_node_t;

typedef struct slinger_bundle_node
{
    uint32_t                hash;           /**< hash The hash (without mods) of the bundle name */
    uint8_t                 priority;
    time_t                  last_sent;      /**< last_sent Last time this value was downlinked */
    uint16_t                timeout;        /**< timeout Maximum number of seconds between packet downlink, whether or not it is full */
    uint16_t                num_streams;
    pthread_mutex_t         mutex;          /**< mutex Mutex controlling access to all nodes in the bundle */
    slinger_stream_node_t   **streams;
} slinger_bundle_node_t;

struct slinger_cache_node
{
    E_SLINGER_CACHE_TYPE    type;
    char                    *name;            /**< name Human-readable name (used for log files and collision detection) */
    union
    {
        void                    *output;
        slinger_stream_node_t   *output_stream;
        slinger_bundle_node_t   *output_bundle;
        slinger_periodic_node_t *output_periodic;
    };
};

typedef struct slinger_group_entry
{
    GList                   list;
    slinger_cache_node_t    *node;
} slinger_group_entry_t;

typedef struct
{
    e_slinger_link  link_enum;
    char            *device;

    bool            is_active;

    uint32_t        bps;
    int32_t         bytes_to_write;
    float           max_priority;

    pq_t            *downlink_pq;
    fifo_t          *resend_fifo;

    fifo_t          *control_fifo_queue;
    fifo_t          *downlink_fifo;

    uint32_t        fifo_pause_us;

    pthread_t       downlink_thread;
    pthread_t       monitor_thread;
} slinger_link_state_t;
extern slinger_link_state_t link_state[slinger_link_openport+1];

void initialize_slinger_tables();
void slinger_cache_stream_data_known(slinger_time_t *m_time, slinger_cache_node_t *m_node, bool m_skipbiphase, void *m_data, size_t m_len);

void slinger_downlink_file(const char *m_filename, uint32_t m_priority);
void slinger_downlink_file_data(const char *m_filename, uint32_t m_priority, bool m_qos, const void *m_data, size_t m_len);
bool slinger_cache_raw_data(uint32_t m_hash, uint32_t m_priority, uint8_t m_type,
                            bool m_skipbiphase, bool m_qos, void *m_data, size_t m_len, lzma_stream *m_stream);
bool slinger_compress_node(slinger_dl_node_t *m_node);
void slinger_add_to_cache_sl(uint32_t m_key, slinger_cache_node_t *m_value);
void set_slinger_dl_priority(const char *m_name, uint8_t m_priority);
const char *slinger_get_channel_name(uint32_t m_hash);

void slinger_free_cache_node(slinger_cache_node_t *m_node);

void slinger_free_dl_node(void *m_node);

static inline bool slinger_priority_downlink(int32_t m_priority)
{
    if (m_priority <= (int32_t)link_state[slinger_link_highrate].max_priority ||
            m_priority <= (int32_t)link_state[slinger_link_biphase].max_priority)
    {
        return true;
    }
    return false;
}
#endif /* SLINGER_DATA_H_ */
