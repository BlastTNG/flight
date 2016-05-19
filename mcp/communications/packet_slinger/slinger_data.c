/**
 * @file slinger_data.c
 *
 * @date Sep 14, 2010
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

#include <glib.h>
#include <stdint.h>
#include <stdbool.h>
#include <libxml/tree.h>
#include <lzma.h>
#include <sys/stat.h>
#include <sys/param.h>

#include <blast.h>
#include <channels.h>
#include <lookup.h>
#include <blast_packet_format.h>

#include <packet_slinger.h>
#include <slinger_data.h>

BLAST_LOOKUP_TABLE(BLAST_PORT_DEF, static);


static slinger_dl_node_t *slinger_dl_node_new(uint32_t m_hash, void *m_data, uint8_t m_priority, size_t m_len, uint8_t m_type, bool m_qos);

static void slinger_free_bundle_node(slinger_bundle_node_t *m_node)  __attribute__((unused));
static void slinger_gather_and_package_bundle(slinger_cache_node_t *m_cache, bool m_skipbiphase);

/**
 * Cache structure for downlink streams.  Unique to individual downlink channels
 */
GHashTable *slinger_cache_ht = NULL;
GHashTable *slinger_groups_ht = NULL;
extern slinger_link_state_t link_state[2];

typedef enum {
    COMPRESS_TYPE_LOG,
    COMPRESS_TYPE_PKT,
    COMPRESS_TYPE_STREAM
} e_compress_type;

static void slinger_set_compression(int m_type, size_t m_size, lzma_filter *m_filter)
{
    m_size = MAX(LZMA_DICT_SIZE_MIN, m_size);
    /**
     * Here we define some useful pre-sets for our compression algorithm (LZMA).  These are empirically tested
     * on ground data to balance speed and compression ratio.  These choices may be revisited at any point or
     * additional presets added without requiring modification to the ground station as the values are encoded
     * in the header of each packet.
     *
     * For reference, using these presets, I measure the compression of a group of 100x 8192-byte KID UDP packets
     * to be 0.377, meaning that the compressed packet occupies 308kiB instead of 819KiB.
     *
     * MCP data logs similarly compress from 8685 bytes to 1488 bytes or 18% of original size
     */
    static lzma_options_lzma opt_lzma2_text = {
                  /**
                   * The "literal context" bits is log2 of the number of possible "contexts" that can be
                   * maintained for matches.  We set this to the maximum of 4 for our text files
                   */
                  .lc = 4,
                  /**
                   * Use at least the minimum size for the dictionary
                   */
                  .dict_size = m_size,
                  /**
                   * These two control the depth of match searching (10), a length at which the search is cut
                   * off (64) and the type of match storage (hash chain)
                   */
                  .depth = 10,
                  .nice_len = 64,
                  /**
                   * The mode and match finder control how searches are performed to match substrings.  These
                   * are tuned for log files at the moment
                   */
                  .mf = LZMA_MF_HC3,
                  .mode = LZMA_MODE_FAST
    };
    static lzma_options_lzma opt_lzma2_kid_packet = {
                  /**
                   * The "literal context" bits is log2 of the number of possible "contexts" that can be
                   * maintained for matches.  We set this to the maximum of 4 for our text files
                   */
                  .lc = 3,
                  .lp = 1,
                  /**
                   * This sets preferred alignment to 2^4=16 bytes.  For KID packets, this makes sense as the
                   * I/Q pair forms a 16-byte block.
                   */
                  .pb = 4,
                  /**
                   * Optimized by testing on test data packets.  Revisit with real data.
                   */
                  .dict_size = 64 * 1024,
                  /**
                   * These two control length at which the search is cut off (8 bytes) and the
                   * type of match storage (hash chain)
                   */
                  .nice_len = 64,
                  .mf = LZMA_MF_HC3,
                  .mode = LZMA_MODE_FAST
    };
    static lzma_options_lzma opt_lzma2_kid_stream = {
                  /**
                   * The "literal context" bits is log2 of the number of possible "contexts" that can be
                   * maintained for matches.  We set this to the maximum of 4 for our text files
                   */
                  .lc = 3,
                  .lp = 1,
                  /**
                   * This sets preferred alignment to 2^2=8 bytes.  For KID streams, this makes sense as the
                   * I/Q streams are separated
                   */
                  .pb = 2,
                  .dict_size = 64 * 1024,
                  /**
                   * These two control length at which the search is cut off (8 bytes) and the
                   * type of match storage (hash chain)
                   */
                  .nice_len = 64,
                  .mf = LZMA_MF_HC3,
                  .mode = LZMA_MODE_FAST
    };
    switch (m_type) {
        case COMPRESS_TYPE_LOG:
            m_filter[0].options = &opt_lzma2_text;
            break;
        case COMPRESS_TYPE_PKT:
            m_filter[0].options = &opt_lzma2_kid_packet;
            break;
        case COMPRESS_TYPE_STREAM:
            m_filter[0].options = &opt_lzma2_kid_stream;
            break;
        default:
            m_filter[0].options = &opt_lzma2_text;
            break;
    }

}

void initialize_slinger_tables()
{

	slinger_cache_ht = ebex_sl_new(16);
	slinger_groups_ht = ebex_sl_new(10);
}

void slinger_free_cache_node(slinger_cache_node_t *m_node)
{
    if (m_node) {
        BLAST_SAFE_FREE(m_node->name);
        if ((m_node->type == SLINGER_CACHE_TYPE_BUNDLE) && (m_node->output_bundle)) {
            BLAST_SAFE_FREE(m_node->output_bundle->streams);
        }
        BLAST_SAFE_FREE(m_node->output);
        bfree(err, m_node);
    }
}

void slinger_add_to_cache_sl(uint32_t m_key, slinger_cache_node_t *m_value)
{
	slinger_cache_node_t *dup_node;

	dup_node = ebex_sl_add(slinger_cache_ht, m_key, m_value, true);
	if (dup_node)
	{
		ebex_fatal("Hash collision between %s and %s.  Rename in packetslinger.xml to resolve.",
				m_value->name, dup_node->name);
		slinger_free_cache_node(dup_node);
	}
}


/**
 * Caches data from an arbitrary stream into the format needed for a specific cache node.
 * @param m_time Timestamp value at which to cache the data
 * @param m_node Pointer to the cache node
 * @param m_skipbiphase If true, don't downlink these data over decom
 * @param m_data Pointer to the arbitrary data to cache
 * @param m_len length in bytes of the data pointed to by #m_data
 */
void slinger_cache_stream_data_known(slinger_time_t *m_time, slinger_cache_node_t *m_node,
		bool m_skipbiphase, void *m_data, size_t m_len)
{
	slinger_time_t data_time;
	int64_t delta_time;
	slinger_stream_node_t *stream = NULL;
	slinger_periodic_node_t *periodic = NULL;

	if (!slinger_link_is_active() || !m_node) return;

	slinger_time_normalize(m_time, &data_time);

	switch (m_node->type)
	{
		case SLINGER_CACHE_TYPE_PERIODIC:
		    uint32_t temp_val = 0;
			switch(m_len)
			{
				case 1:
					temp_val = (uint32_t) *((uint8_t*) m_data);
					break;
				case 2:
					temp_val = (uint32_t) *((uint16_t*)m_data);
					break;
				case 4:
					temp_val = *((uint32_t*)m_data);
					break;
				default:
					blast_err("Unknown data type %d", (int)m_len);
					return;
			}

			periodic = m_node->output_periodic;

			/// Skip downlink of values with max (255)
			if (periodic->priority == UINT8_MAX) return;

			if (periodic->downlink.value != temp_val
                    || (periodic->timeout && (periodic->last_sent + periodic->timeout < time(NULL))))
			{
				BLAST_SWAP(periodic->downlink.value, temp_val);
				periodic->downlink.time.qword = data_time.qword;

				if (slinger_cache_raw_data(periodic->downlink.hash, periodic->priority,
						BLAST_PORT_DEF_DATA, m_skipbiphase, false,
						&periodic->downlink, sizeof(periodic->downlink), NULL))
				{
					/**
					 * If we have successfully queued the packet for download, reset the time on our node
					 */
					periodic->last_sent = time(NULL);
				}
				else
				{
					/**
					 * If we cannot cache the packet for any reason, we re-set the value to its previous value.
					 * We will re-try in the next frame.  Our index will be off by the fastsamp difference but
					 * we won't miss the value change completely.
					 */
					periodic->downlink.value = temp_val;
				}
			}

			break;
		case SLINGER_CACHE_TYPE_STREAM:
			stream = m_node->output_stream;
			if ((stream->downlink.produced + m_len > stream->packetsize)
				|| (stream->bundle->output_bundle->last_sent &&
					(stream->bundle->output_bundle->last_sent + stream->bundle->output_bundle->timeout < time(NULL))))
			{
				/**
				 * Only allow one thread to bundle the output.  We are a bit cavalier about the actual stream content during
				 * bundling.  The logic being that we accept the last stream packet as potentially
				 */
				if (!pthread_mutex_trylock(&(stream->bundle->output_bundle->mutex)))
				{

					slinger_gather_and_package_bundle(stream->bundle, m_skipbiphase);
					pthread_mutex_unlock(&(stream->bundle->output_bundle->mutex));
				}
			}

			delta_time = slinger_time_diff(&data_time, &stream->downlink.time);
			if (delta_time < 0 || delta_time > stream->delta_t)
			{
				/**
				 * Set the stream index based on the first timestamp received in the group
				 */
				if (!stream->downlink.produced)
					slinger_time_normalize(m_time, &stream->downlink.time);

				switch (m_len)
				{
					case 1:
						*(uint8_t*)(stream->downlink.buffer + stream->downlink.produced) = *((uint8_t*)m_data);
						break;
					case 2:
						*(uint16_t*)(stream->downlink.buffer + stream->downlink.produced) = *((uint16_t*)m_data);
						break;
					case 4:
						*(uint32_t*)(stream->downlink.buffer + stream->downlink.produced) = *((uint32_t*)m_data);
						break;
					default:
						memcpy(stream->downlink.buffer + stream->downlink.produced, m_data, m_len);
						break;
				}

				stream->downlink.produced += m_len;
			}

			break;
		default:
			blast_err("Unknown node type %d", (int)m_node->type);
			break;
	}
}

/**
 * Handles downlinking arbitrary data into the slinger stream.  Data are compressed using lzma.  Entire compressed
 * stream is passed over to the downlink cache as a chunk.  Data will be received as a file by Packetwrangler
 *
 * @param m_filename Name give the file upon reception by PacketWrangler
 * @param m_priority Priority at which to downlink the data
 * @param m_qos If true, set QOS flag in the downlink
 */
void slinger_downlink_file_data(const char *m_filename, uint32_t m_priority, bool m_qos, const void *m_data, size_t m_len)
{

    size_t buf_size;
    uint8_t *out_buffer;
    lzma_stream *compress_stream = LZMA_STREAM_INIT;
    lzma_filter filters[] = {
        { .id = LZMA_FILTER_LZMA2, .options = NULL },
        { .id = LZMA_VLI_UNKNOWN, .options = NULL },
    };

    lzma_ret ret;

    buf_size = m_len + strlen(m_filename) + 512;
    if (!(out_buffer = balloc(fatal, buf_size))) return;

    slinger_set_compression(COMPRESS_TYPE_LOG, MIN(m_len, (UINT32_C(1) << 20)), filters);
    compress_stream->next_in = m_data;
    compress_stream->avail_in = m_len;
    compress_stream->next_out = out_buffer + strlen(m_filename) + 1;
    compress_stream->avail_out = buf_size - strlen(m_filename) - 1;
    if ((ret = lzma_stream_encoder(compress_stream, filters, LZMA_CHECK_CRC32)) != LZMA_OK) {
        const char *errmsg;
        switch (ret) {
            case LZMA_MEM_ERROR:
                errmsg = "Memory allocation error";
                break;
            case LZMA_OPTIONS_ERROR:
                errmsg = "Unsupported options to filter chain";
                break;
            case LZMA_UNSUPPORTED_CHECK:
                errmsg = "Unsupported checksum requested";
                break;
            default:
                errmsg = "Unknown error, report this to your friendly mcp developer";
                break;
        }
        blast_fatal("Error %u initializing the mcp file downlink compressor: %s", ret, errmsg);
        bfree(err, out_buffer);
        return;
    }

    ret = lzma_code(compress_stream, LZMA_RUN);
    if ( ret != LZMA_OK && ret != LZMA_STREAM_END) {
        const char *errmsg;
        switch(ret) {
            case LZMA_MEM_ERROR:
                errmsg = "Memory allocation error";
                break;
            case LZMA_DATA_ERROR:
                errmsg = "Size limits exceeded";
                break;
            default:
                errmsg = "Unknown error, report this to your friendly mcp developer";
                break;
        }
        blast_fatal("Error %u compressing data in the mcp file downlink: %s", ret, errmsg);
        bfree(err, out_buffer);
        return;
    }
    lzma_code(compress_stream, LZMA_FINISH);

    /**
     * For downlink, we want to have the filename come first (so we know what to call the decompressed file),
     * so we copy that first over to the start of the compressed data.  This is the buffer we will use for downlink
     */

    memcpy(out_buffer, m_filename, strlen(m_filename) + 1);
    uint32_t file_hash = PMurHash32(BLAST_MAGIC32, m_filename, strlen(m_filename));
    slinger_cache_raw_data(file_hash, m_priority, BLAST_PORT_DEF_FILE, false, true,
            out_buffer, (compress_stream->next_out - out_buffer), NULL);

    bfree(err, out_buffer);
    blast_info("Queued %s for download: 0x%08X", m_filename, file_hash);
}

/**
 * Handles downlinking a file into the slinger stream.  Reads the entire file into memory before attempting.
 * Limits on filesize currently handled statically.  File is compressed using ezip.  Entire compressed stream is
 * passed over to the downlink cache as a chunk.  #slinger_cache_raw_data handles breaking the chunk into
 * packets for downlink
 *
 * @param m_filename Name of the file to downlink
 * @param m_priority Priority to downlink the file as
 * @param m_bytes Number of bytes from the end of the file or 0 for full file
 */
void slinger_downlink_file(const char *m_filename, uint32_t m_priority, ssize_t m_bytes)
{
	FILE *fp = 0;
	uint8_t *file_buffer;
	struct stat statbuf;

	/// We never downlink items with max priority value
	if (m_priority == UINT8_MAX) return;

    blast_info("Received request to downlink %s", m_filename);

    if (!m_filename) {
        blast_err("Passed NULL filename");
        return;
    }

    if (stat(m_filename, &statbuf) == -1) {
        blast_err("Could not stat file %s to downlink", m_filename);
        return;
    }

    if (!(fp = fopen(m_filename, "r"))) {
        blast_err("Could not open file %s", m_filename);
        return;
    }

    /**
     * Adjust our buffer size to allow coding overhead
     */
    if (!m_bytes) m_bytes = statbuf.st_size;
    if (statbuf.st_size > m_bytes) {
        fseek(fp, statbuf.st_size - m_bytes, SEEK_SET);
    }

    if (!(file_buffer = balloc(fatal, m_bytes))) return;

    if (fread(file_buffer, 1, m_bytes, fp) != (size_t)m_bytes) {
        blast_err("Could not read %u bytes from file %s", (unsigned)m_bytes, m_filename);
    } else {
        slinger_downlink_file_data(m_filename, m_priority, 0, file_buffer, m_bytes);
    }
    fclose(fp);
    bfree(err, file_buffer);

	return;
}

/**
 * Creates a new downlink node for Packet Slinger.  This will allocate a node that has pre-determined size or leave the data pointer NULL
 * if we do not yet have data to specify.  This might be the case if we are caching data for later compression.
 * @param m_hash Hash of the downlink data stream
 * @param m_data Pointer to the prepared data
 * @param m_priority Downlink priority value (1 is the highest priority)
 * @param m_len Length in bytes of m_data.  If 0, do not allocate space for m_data
 * @param m_type Type of data downlink (specified by the port defs)
 * @param m_qos If true, the packet will be re-sent if not acknowledged
 * @return Pointer to the newly allocated node or NULL on failure
 */
static slinger_dl_node_t *slinger_dl_node_new(uint32_t m_hash, void *m_data, uint8_t m_priority, size_t m_len, uint8_t m_type, bool m_qos)
{
	slinger_dl_node_t *new_node = NULL;

	if ((new_node = balloc(err, sizeof(slinger_dl_node_t))))
	{
		memset(new_node, 0, sizeof(slinger_dl_node_t));
		new_node->id = m_hash;
		if (m_data && m_len)
		{
			if (!(new_node->data = balloc(err, m_len + sizeof(blast_master_packet_t) + 2 * sizeof(uint32_t))))
			{
				bfree(err, new_node);
				return NULL;
			}
			memset(new_node->data, 0, sizeof(blast_master_packet_t));
			memcpy(BLAST_MASTER_PACKET_PAYLOAD(new_node->data), m_data, m_len);
			new_node->data->length = m_len;
			new_node->data->type = m_type & 0b111;
			new_node->size = BLAST_MASTER_PACKET_FULL_LENGTH(new_node->data);
		}
		else
		{
			/**
			 * If we don't have data, the length is used to store the input size of the compression stream
			 */
			new_node->size = m_len;
		}

		new_node->priority = m_priority;
		new_node->type = m_type;
		new_node->qos = m_qos;
	}

	return new_node;
}

bool slinger_cache_raw_data(
		uint32_t m_hash, uint32_t m_priority, uint8_t m_type, bool m_skipbiphase, bool m_qos,
		void *m_data, size_t m_len, lzma_stream *m_stream)
{
	slinger_dl_node_t *new_node = NULL;
	bool use_stream[2];
	lzma_stream *output_stream[2] = {NULL};

	use_stream[slinger_link_highrate] = true;
	use_stream[slinger_link_biphase] = !m_skipbiphase;

	for (int i = 0; i < 2; i++)
	{
		if (!link_state[i].is_active ||
				link_state[i].max_priority < (float)m_priority)
			use_stream[i] = false;
	}

	if (m_stream)
	{
		if (use_stream[slinger_link_biphase] && use_stream[slinger_link_highrate])
		{
			output_stream[slinger_link_biphase] = m_stream;
			output_stream[slinger_link_highrate] = ezip_compress_init(m_len);
			memcpy(output_stream[slinger_link_highrate]->raw_data_buffer, m_stream->raw_data_buffer, m_len + sizeof(ezip_chunk_header32_t));
		}
		else if (use_stream[slinger_link_biphase])
		{
			output_stream[slinger_link_biphase] =  m_stream;
		}
		else if (use_stream[slinger_link_highrate])
		{
			output_stream[slinger_link_highrate] = m_stream;
		}
		else
		{
			ezip_compress_deinit(m_stream, true, true);
			return true;
		}
	}

	for (int i = 0; i < 2; i++)
	{
		if (use_stream[i])
		{
			new_node = slinger_dl_node_new(m_hash, m_data, m_priority, m_len, m_type, m_qos);
			if (!new_node)
			{
				blast_err("Could not allocate node memory");
			}
			else
			{
				new_node->stream = output_stream[i];
				enqueue_pq(link_state[i].downlink_pq, new_node, m_priority);
			}
		}
	}

	return true;
}

static void slinger_gather_and_package_bundle(slinger_cache_node_t *m_cache, bool m_skipbiphase)
{
	uint8_t *in_data_buf = NULL;
	slinger_bundle_node_t *bundle = m_cache->output_bundle;
	slinger_stream_node_t *stream = NULL;
	ezip_stream_t *compressed_stream = NULL;
	size_t total_len = 0;
	size_t i = 0;

	bundle->last_sent = time(NULL);

	/// Skip bundles with 255 priority
	if (bundle->priority == UINT8_MAX)
	{
		for (i = 0; i < bundle->num_streams; i++)
		{
			stream = bundle->streams[i];
			memset(stream->downlink.buffer, 0, stream->packetsize);
			stream->downlink.produced = 0;
		}
		return;
	}

	if (!bundle->num_streams)
	{
		blast_err("No streams defined for bundle %s (0x%08x)", m_cache->name, bundle->hash);
		return;
	}

	/**
	 * Start with the size of the hash for the bundle
	 */
	total_len = sizeof(uint32_t);
	for (i = 0; i < bundle->num_streams; i++)
	{
		/**
		 * Add the size of each stream plus the size of the hash of each stream
		 */
		if (bundle->streams[i]->downlink.produced)
			total_len += (sizeof(bundle->streams[i]->downlink) + bundle->streams[i]->downlink.produced);
	}

	if (!(compressed_stream = ezip_compress_init(total_len)))
	{
		blast_err("Could not allocate downlink buffer");
		return;
	}
	in_data_buf = compressed_stream->data_buffer;

	for (i = 0; i < bundle->num_streams; i++)
	{
		stream = bundle->streams[i];
		if (stream->downlink.produced)
		{
			memcpy(in_data_buf, &stream->downlink, sizeof(stream->downlink) + stream->downlink.produced);
			in_data_buf += sizeof(stream->downlink) + stream->downlink.produced;

			memset(stream->downlink.buffer, 0, stream->packetsize);
			stream->downlink.produced = 0;
		}
	}

	if (compressed_stream->chunk_header16->options.option_chunk_type == ezip_16bit_chunk)
	{
		compressed_stream->chunk_header16->options.option_differencing = ezip_diff_wmtf;
		compressed_stream->chunk_header16->options.option_entropy_coder = ezip_entropy_golem;
		compressed_stream->chunk_header16->options.option_lz = false;
		compressed_stream->chunk_header16->options.option_sort = true;
		compressed_stream->chunk_header16->options.option_chunk_type = ezip_16bit_chunk;
		compressed_stream->chunk_header16->frame_size = 2;
	}
	else
	{
		compressed_stream->chunk_header32->options.option_differencing = ezip_diff_wmtf;
		compressed_stream->chunk_header32->options.option_entropy_coder = ezip_entropy_golem;
		compressed_stream->chunk_header32->options.option_lz = false;
		compressed_stream->chunk_header32->options.option_sort = true;
		compressed_stream->chunk_header32->options.option_chunk_type = ezip_32bit_chunk;
		compressed_stream->chunk_header32->frame_size = 2;
	}


	slinger_cache_raw_data(bundle->hash, bundle->priority, ebex_port_def_data, m_skipbiphase,
			false, NULL, total_len, compressed_stream);

	return;

}

bool slinger_compress_node(slinger_dl_node_t *m_node)
{
	ezip_stream_t *compressed_stream;
	ssize_t compressed_len;
	uint8_t *payload;

	compressed_stream = m_node->stream;
	/// We allocate the additional uint32_t for the downlink stream.  But it is not in our data buffer
	compressed_len = ezip_compress(compressed_stream, m_node->size - sizeof(uint32_t));

	if (compressed_len <= 0)
	{
		blast_err("Failed compressing downlink data");
		return false;
	}


	m_node->data = (blast_master_packet_t*)(compressed_stream->raw_working_buffer);
	memset(m_node->data, 0, sizeof(blast_master_packet_t));
	payload = BLAST_MASTER_PACKET_PAYLOAD(m_node->data);
	/// Marking the beginning of the packet with '0' sets this to a 'bundle' type
	*(uint32_t*)payload = 0;
	memcpy(payload + (sizeof(uint32_t)), compressed_stream->raw_data_buffer, compressed_len);

	/// Clear out the (now unused) compression stream
	ezip_compress_deinit(compressed_stream, true, false);
	m_node->stream = NULL;

	m_node->data->length = compressed_len + sizeof(uint32_t);
	m_node->size = BLAST_MASTER_PACKET_FULL_LENGTH(m_node->data);

	return true;
}

static inline void slinger_adjust_node_priority(gpointer m_node, gpointer m_priority)
{
	uint8_t prev_priority;
	slinger_cache_node_t *node = (slinger_cache_node_t*)m_node;
	switch(node->type)
	{
		case SLINGER_CACHE_TYPE_BUNDLE:
			prev_priority = node->output_bundle->priority;
			node->output_bundle->priority = *(uint8_t*)m_priority;
			break;
		case SLINGER_CACHE_TYPE_PERIODIC:
			prev_priority = node->output_periodic->priority;
			node->output_periodic->priority = *(uint8_t*)m_priority;
			break;
		default:
			break;
	}
	blast_info("Priority for slinger channel %s changed from %u to %u", node->name, prev_priority, *(uint8_t*)m_priority);
}

/**
 * Changes the priority of a slinger channel downlink
 * @param m_name The name of the slinger group/bundle/periodic
 * @param m_priority The new priority value for the channel
 */
void set_slinger_dl_priority(const char *m_name, uint8_t m_priority)
{
	slinger_cache_node_t *cache_node;
	GList *group_list;
	slinger_group_entry_t *group_entry;

	if ((group_list = g_hash_table_lookup(slinger_groups_ht, m_name)))
	{
		g_list_foreach(group_entry, slinger_adjust_node_priority, &m_priority);

	}

	if ((cache_node = (slinger_cache_node_t*)g_hash_table_lookup(slinger_cache_ht, m_name)))
	{
		slinger_adjust_node_priority(cache_node, m_priority);
	}

}

const char *slinger_get_channel_name(uint32_t m_hash)
{
	slinger_cache_node_t *cache_node;
	const char *name = "NOT_FOUND";

	if ((cache_node = (slinger_cache_node_t *)g_hash_table_lookup(slinger_cache_ht, m_hash)))
	{
		if (cache_node->name)
			name = cache_node->name;
		else name = "UNNAMED_CHANNEL";
	}
	return name;
}

void slinger_free_dl_node(void *m_node)
{
	slinger_dl_node_t *free_node = (slinger_dl_node_t*)m_node;

	if (free_node)
	{
		ezip_compress_deinit(free_node->stream, true, true);
		free_node->stream = NULL;
		BLAST_SAFE_FREE(free_node->data);
		BLAST_SAFE_FREE(free_node);
	}
}
