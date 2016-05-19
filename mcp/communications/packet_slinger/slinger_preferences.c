/**
 * @file slinger_preferences.c
 *
 * @date Apr 5, 2011
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

#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <sys/types.h>
#include <libxml/tree.h>

#include <blast.h>
#include <blast_common.h>
#include <blast_hash.h>
#include <blast_packet_format.h>
#include <blast_sl.h>
#include <blast_lookup.h>
#include <blast_DAN.h>

#include <slinger_frame.h>
#include <slinger_data.h>
#include <packet_slinger.h>
#include <slinger_preferences.h>

extern blast_sl_head_t *slinger_cache_sl;
extern blast_sl_head_t *slinger_groups_sl;

static ssize_t slinger_xml_process_raw_node(xmlNodePtr m_node);
static void slinger_xml_add_missing_channels(void);

/**
 * Opens and validates the specified XML file.  Once validated, it passes the root document pointer to the generic
 * parser to handle
 * @param m_name Name of the XML slinger channels definition file
 * @return true on success, false on failure
 */
bool slinger_xml_load_preferences(const char *m_name)
{
	xmlDocPtr 		slinger_doc = NULL;
	bool			retval = false;

	if (m_name == NULL)
	{
		blast_err("Passed NULL pointer for a filename");
		return false;
	}

	initialize_slinger_slow_packets();

	slinger_doc = xmlReadFile(m_name, NULL, XML_PARSE_NOCDATA|XML_PARSE_COMPACT|XML_PARSE_DTDLOAD|XML_PARSE_DTDATTR|XML_PARSE_NOENT);
	if (slinger_doc == NULL)
	{
		blast_fatal("Could not load slinger preferences file: %s", m_name);

		return false;
	}

	/**
	 * Parse and assign the top-level nodes to their respective pointers
	 */
	if (slinger_xml_process_raw_node(xmlDocGetRootElement(slinger_doc)) == -1)
	{
		blast_err("Could not parse %s", m_name);
	}
	else
	{
		blast_xml_dbg("Successfully parsed %s", m_name);
		retval = true;
	}

	xmlFreeDoc(slinger_doc);

	slinger_xml_add_missing_channels();

	return retval;

}

/**
 * Processes a generic node associated with the packet slinger XML.  All nodes currently defined have
 * associated call-back functions that will do the unique processing required.  This is similar
 * to a SAX-based implementation but (I believe) cleaner and more adaptable for EBEX's needs.
 *
 * Look in slinger_data.h for the currently recognized node names.  The callback functions
 * are given at the end of this file.
 *
 * @param m_node Pointer to the XML node to be processed
 * @return Number of nodes processed.
 */
static ssize_t slinger_xml_process_raw_node(xmlNodePtr m_node)
{
	xmlNodePtr child_node;
	const SLINGER_XML_TAG_LOOKUP_T *node_tag;
	void *func_return = NULL;
	ssize_t retval = 0;

	if (!m_node )
	{
		blast_err("Passed NULL pointer!");
		return -1;
	}

	for (child_node = m_node->children; child_node != NULL; child_node=child_node->next)
	{
		if (child_node->type != XML_ELEMENT_NODE)
			continue;

		blast_xml_dbg("processing <%s>", (xmlChar*) child_node->name);

		for (node_tag = SLINGER_XML_TAG_LOOKUP_TABLE; node_tag->position != SLINGER_XML_TAG_END; node_tag++)
		{
			if (xmlStrcasecmp(child_node->name, (xmlChar*)node_tag->text) == 0)
			{
				func_return = node_tag->function((void*)child_node);
				if (!func_return) return -1;

				retval++;
			}
		}
	}
	return retval;
}

static inline void
slinger_xml_parse_groups (xmlNodePtr m_xml_procedure, slinger_cache_node_t *m_node)
{
	xmlChar *tempstr = NULL;
	char *dupstr = NULL;
	char *token = NULL;
	char *saveptr = NULL;
	const char *delim = " ,";
	uint32_t hash = 0;

	slinger_group_entry_t *entry;
	slinger_group_entry_t *extant_entry;

	if(!(tempstr = xmlGetProp(m_xml_procedure, (const xmlChar*)"groups")))
		return;

	blast_tmp_sprintf(dupstr, "%s", (char*)tempstr);
	token = strtok_r(dupstr, delim, &saveptr);
	while(token)
	{
		hash = PMurHash32(BLAST_MAGIC32, token, strlen(token));
		entry = balloc(fatal, sizeof(slinger_group_entry_t));
		INIT_LIST_HEAD(&(entry->list));
		entry->node = m_node;
		if ((extant_entry = blast_sl_add(slinger_groups_sl, hash, entry, false)))
		{
			list_add_tail(&entry->list, &extant_entry->list);
		}
		token = strtok_r(NULL, delim, &saveptr);
	}
	xmlFree(tempstr);
}

void *slinger_xml_tag_dfmux_slow_callback(void *m_node)
{
	xmlNodePtr stream_xml = (xmlNodePtr)m_node;
	xmlNodePtr bundle_xml = stream_xml->parent;

	xmlChar *bundle_name;

	xmlChar *channel_name = xmlGetProp(stream_xml, (xmlChar*)"field");
	xmlChar *stream_packetsize = NULL;
	xmlChar *stream_rate = NULL;

	slinger_stream_node_t *new_stream = NULL;
	slinger_cache_node_t  *cache_node = NULL;

	uint32_t bundle_hash;
	uint32_t stream_hash;
	uint32_t packetsize = 1024;
	double rate = 1.0;

	if (!channel_name)
	{
		blast_fatal("Missing dfmux_slow 'field' property at line %ld", xmlGetLineNo(stream_xml));
		return NULL;
	}
	if (!(bundle_name = xmlGetProp(bundle_xml, (xmlChar*)"name")))
	{
		blast_fatal("Missing bundle 'name' property at line %ld", xmlGetLineNo(bundle_xml));
		blast_XML_SAFE_FREE(channel_name);
		return NULL;
	}

	bundle_hash = blast_hash_key(bundle_name, xmlStrlen(bundle_name));
	stream_hash = blast_hash_key(channel_name, xmlStrlen(channel_name));

	if ((stream_packetsize = xmlGetProp(stream_xml, (xmlChar*)"packetsize")))
	{
		packetsize = (uint32_t)strtoul((char*)stream_packetsize,NULL,10);
		blast_XML_SAFE_FREE(stream_packetsize);
	}
	if ((stream_rate = xmlGetProp(stream_xml, (xmlChar*)"rate")))
	{
		if (xmlStrcasecmp(stream_rate, (const xmlChar*)"full"))
			rate = strtod((char*)stream_rate,NULL);
		blast_XML_SAFE_FREE(stream_rate);
	}

	/**
	 * Find the bundle first and then assign the new stream
	 */
	if ((cache_node = blast_sl_find(slinger_cache_sl, bundle_hash)))
	{
		if (cache_node->type != SLINGER_CACHE_TYPE_BUNDLE)
		{
			blast_err("Name and/or hash collision for %s (with %s) at line %ld.  Expected bundle.",
					bundle_name, cache_node->name, xmlGetLineNo(bundle_xml));

			blast_fatal("Hash collisions are fatal errors.  Rename either value to resolve.  Exiting");
			return NULL;
		}
		if (cache_node->name && strcmp((char*)bundle_name, cache_node->name))
		{
			blast_fatal("Hash collision for bundle %s with stream %s. Rename either value to resolve.  Exiting",
					(char*)bundle_name, cache_node->name);
			return NULL;
		}

		if (!cache_node->output_bundle->num_streams++)
			cache_node->output_bundle->streams = balloc(err, sizeof(slinger_stream_node_t*));
		else
			cache_node->output_bundle->streams = reballoc(err, cache_node->output_bundle->streams, cache_node->output_bundle->num_streams * sizeof(slinger_stream_node_t*));

		new_stream = balloc(err, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);
		cache_node->output_bundle->streams[cache_node->output_bundle->num_streams - 1] = new_stream;

		memset(new_stream, 0, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);
		new_stream->downlink.hash = stream_hash;
		new_stream->packetsize = packetsize;
		new_stream->bundle = cache_node;
		new_stream->delta_t = (int64_t) (100000.0 / rate);

		/**
		 * After setting up the extant bundle, we create a new node in the cache list for the stream.
		 */
		cache_node = balloc(err, sizeof(slinger_cache_node_t));
		blast_ZERO_P(cache_node);
		cache_node->type = SLINGER_CACHE_TYPE_STREAM;
		cache_node->name = (char*)channel_name;
		cache_node->output_stream = new_stream;

		if (!slinger_add_slow_packet_field((char*)channel_name, cache_node))
		{
			blast_fatal("Slow packet stream %s unknown", (const char*)channel_name);
			return NULL;
		}

		slinger_add_to_cache_sl(stream_hash, cache_node);
	}
	else
	{
		blast_fatal("Malformed slinger XML file.  This will likely not work");
		m_node = NULL;
	}

	blast_XML_SAFE_FREE(bundle_name);
	return m_node;
}

void *slinger_xml_tag_hwp_callback(void *m_node)
{
	xmlNodePtr stream_xml = (xmlNodePtr)m_node;
	xmlNodePtr bundle_xml = stream_xml->parent;

	xmlChar *bundle_name;

	xmlChar *board_num = xmlGetProp(stream_xml, (xmlChar*)"board");
	xmlChar *stream_packetsize = NULL;
	xmlChar *stream_rate = NULL;

	slinger_stream_node_t *new_stream = NULL;
	slinger_cache_node_t  *cache_node = NULL;

	uint32_t bundle_hash;
	uint32_t stream_hash;
	uint32_t packetsize = 32768;
	double rate = 3051.0;

	if (!board_num)
	{
		blast_fatal("Missing channel 'name' property at line %ld", xmlGetLineNo(stream_xml));
		return NULL;
	}
	if (!(bundle_name = xmlGetProp(bundle_xml, (xmlChar*)"name")))
	{
		blast_fatal("Missing bundle 'name' property at line %ld", xmlGetLineNo(bundle_xml));
		blast_XML_SAFE_FREE(board_num);
		return NULL;
	}

	bundle_hash = blast_hash_key(bundle_name, xmlStrlen(bundle_name));
	stream_hash = ((uint32_t)strtoul((char*)board_num, NULL, 10)) << 8;
	blast_XML_SAFE_FREE(board_num);

	if ((stream_packetsize = xmlGetProp(stream_xml, (xmlChar*)"packetsize")))
	{
		packetsize = (uint32_t)strtoul((char*)stream_packetsize,NULL,10);
		blast_XML_SAFE_FREE(stream_packetsize);
	}
	if ((stream_rate = xmlGetProp(stream_xml, (xmlChar*)"rate")))
	{
		if (xmlStrcasecmp(stream_rate, (const xmlChar*)"full"))
			rate = strtod((char*)stream_rate,NULL);
		blast_XML_SAFE_FREE(stream_rate);
	}

	/**
	 * Find the bundle first and then assign the new stream
	 */
	if ((cache_node = blast_sl_find(slinger_cache_sl, bundle_hash)))
	{
		if (cache_node->type != SLINGER_CACHE_TYPE_BUNDLE)
		{
			blast_err("Name and/or hash collision for %s (with %s) at line %ld.  Expected bundle.",
					bundle_name, cache_node->name, xmlGetLineNo(bundle_xml));

			blast_fatal("Hash collisions are fatal errors.  Rename either value to resolve.  Exiting");
			return NULL;
		}
		if (cache_node->name && strcmp((char*)bundle_name, cache_node->name))
		{
			blast_fatal("Hash collision for bundle %s with stream %s. Rename either value to resolve.  Exiting",
					(char*)bundle_name, cache_node->name);
			return NULL;
		}

		if (!cache_node->output_bundle->num_streams++)
			cache_node->output_bundle->streams = balloc(err, sizeof(slinger_stream_node_t*));
		else
			cache_node->output_bundle->streams =
			        reballoc(err, cache_node->output_bundle->streams,
			                 cache_node->output_bundle->num_streams * sizeof(slinger_stream_node_t*));

		new_stream = balloc(err, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);
		cache_node->output_bundle->streams[cache_node->output_bundle->num_streams - 1] = new_stream;

		memset(new_stream, 0, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);
		new_stream->downlink.hash = stream_hash;
		new_stream->packetsize = packetsize;
		new_stream->bundle = cache_node;
		new_stream->delta_t = (int64_t)(100000.0 / rate);

		/**
		 * After setting up the extant bundle, we create a new node in the cache list for the stream.
		 */
		cache_node = balloc(err, sizeof(slinger_cache_node_t));
		blast_ZERO_P(cache_node);
		cache_node->type = SLINGER_CACHE_TYPE_STREAM;
		if (asprintf(&cache_node->name, "hwp_board%02" PRIu32, stream_hash >> 8) < 0)
		{
			bfree(fatal, cache_node);
			blast_fatal("Could not allocate string memory");
			blast_XML_SAFE_FREE(bundle_name);
			return NULL;
		}
		cache_node->output_stream = new_stream;

		slinger_add_bolo_channel(stream_hash>>8, 0,cache_node);
		slinger_add_to_cache_sl(stream_hash, cache_node);
	}
	else
	{
		blast_fatal("Malformed slinger XML file.  This will likely not work");
		m_node = NULL;
	}

	blast_XML_SAFE_FREE(bundle_name);
	return m_node;
}

void *slinger_xml_tag_channel_callback(void *m_node)
{
	xmlNodePtr stream_xml = (xmlNodePtr)m_node;
	xmlNodePtr bundle_xml = stream_xml->parent;

	xmlChar *bundle_name;

	xmlChar *channel_name = xmlGetProp(stream_xml, (xmlChar*)"name");
	xmlChar *stream_packetsize = NULL;
	xmlChar *stream_rate = NULL;

	slinger_stream_node_t *new_stream = NULL;
	slinger_cache_node_t  *cache_node = NULL;

	uint32_t bundle_hash;
	uint32_t stream_hash;
	uint32_t packetsize = 1024;
	double rate = 5.0;

	if (!channel_name)
	{
		blast_fatal("Missing channel 'name' property at line %ld", xmlGetLineNo(stream_xml));
		return NULL;
	}
	if (!(bundle_name = xmlGetProp(bundle_xml, (xmlChar*)"name")))
	{
		blast_fatal("Missing bundle 'name' property at line %ld", xmlGetLineNo(bundle_xml));
		blast_XML_SAFE_FREE(channel_name);
		return NULL;
	}

	bundle_hash = blast_hash_key(bundle_name, xmlStrlen(bundle_name));
	stream_hash = blast_hash_key(channel_name, xmlStrlen(channel_name));

	if ((stream_packetsize = xmlGetProp(stream_xml, (xmlChar*)"packetsize")))
	{
		packetsize = (uint32_t)strtoul((char*)stream_packetsize,NULL,10);
		blast_XML_SAFE_FREE(stream_packetsize);
	}
	if ((stream_rate = xmlGetProp(stream_xml, (xmlChar*)"rate")))
	{
		if (xmlStrcasecmp(stream_rate, (const xmlChar*)"full"))
			rate = strtod((char*)stream_rate,NULL);
		else
			rate = (double)slinger_get_channel_rate((char*)channel_name);
		blast_XML_SAFE_FREE(stream_rate);

	}
	else
	{
		rate = slinger_get_channel_rate((char*)channel_name);
	}


	/**
	 * Find the bundle first and then assign the new stream
	 */
	if ((cache_node = blast_sl_find(slinger_cache_sl, bundle_hash)))
	{
		if (cache_node->type != SLINGER_CACHE_TYPE_BUNDLE)
		{
			blast_err("Name and/or hash collision for %s (with %s) at line %ld.  Expected bundle.",
					bundle_name, cache_node->name, xmlGetLineNo(bundle_xml));

			blast_fatal("Hash collisions are fatal errors.  Rename either value to resolve.  Exiting");
			return NULL;
		}
		if (cache_node->name && strcmp((char*)bundle_name, cache_node->name))
		{
			blast_fatal("Hash collision for bundle %s with stream %s. Rename either value to resolve.  Exiting",
					(char*)bundle_name, cache_node->name);
			return NULL;
		}

		if (!cache_node->output_bundle->num_streams++)
			cache_node->output_bundle->streams = balloc(err, sizeof(slinger_stream_node_t*));
		else
			cache_node->output_bundle->streams = reballoc(err, cache_node->output_bundle->streams, cache_node->output_bundle->num_streams * sizeof(slinger_stream_node_t*));

		new_stream = balloc(err, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);
		cache_node->output_bundle->streams[cache_node->output_bundle->num_streams - 1] = new_stream;

		memset(new_stream, 0, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);
		new_stream->downlink.hash = stream_hash;
		new_stream->packetsize = packetsize;
		new_stream->bundle = cache_node;
		new_stream->delta_t = (int64_t)(100000.0 / rate);

		/**
		 * After setting up the extant bundle, we create a new node in the cache list for the stream.
		 */
		cache_node = balloc(err, sizeof(slinger_cache_node_t));
		blast_ZERO_P(cache_node);
		cache_node->type = SLINGER_CACHE_TYPE_STREAM;
		cache_node->name = (char*)channel_name;
		cache_node->output_stream = new_stream;

		slinger_add_frame_channel(cache_node->name, cache_node);
		slinger_add_to_cache_sl(stream_hash, cache_node);
	}
	else
	{
		blast_fatal("Malformed slinger XML file.  This will likely not work");
		m_node = NULL;
	}

	blast_XML_SAFE_FREE(bundle_name);
	return m_node;
}

void *slinger_xml_tag_kid_callback(void *m_node)
{
	xmlNodePtr stream_xml = (xmlNodePtr)m_node;
	xmlNodePtr bundle_xml = stream_xml->parent;

	xmlChar *bundle_name;

	xmlChar *board_name = xmlGetProp(stream_xml, (xmlChar*)"board");
	xmlChar *wire_name = xmlGetProp(stream_xml, (xmlChar*)"wire");
	xmlChar *channel_name = xmlGetProp(stream_xml, (xmlChar*)"channel");
	uint32_t board_val;
	uint32_t wire_val;
	uint32_t channel_val;

	xmlChar *stream_packetsize = NULL;
	xmlChar *stream_rate = NULL;

	slinger_stream_node_t *new_stream = NULL;
	slinger_cache_node_t  *cache_node = NULL;

	uint32_t bundle_hash;
	uint32_t stream_hash;
	uint32_t packetsize = 1024;
	double rate = 190.0;

	if (!board_name || !wire_name || !channel_name)
	{
		if (!board_name) {
		    blast_err("Missing bolo 'board' property at line %ld", xmlGetLineNo(stream_xml));
		} else {
		    xmlFree(board_name);
		}
		if (!channel_name) {
		    blast_err("Missing bolo 'channel' property at line %ld", xmlGetLineNo(stream_xml));
		} else {
		    xmlFree(channel_name);
		}
		return NULL;
	}

	board_val = strtoul((const char*)board_name, NULL, 10);
	channel_val = strtoul((const char*)channel_name, NULL, 10);
	blast_XML_SAFE_FREE(board_name);
	blast_XML_SAFE_FREE(channel_name);

    if (board_val > UINT8_MAX || !board_val)
        blast_err ("Bad board number %" PRIu32 " at line %ld", board_val, xmlGetLineNo(stream_xml));
    return NULL;

	if (!(bundle_name = xmlGetProp(bundle_xml, (xmlChar*)"name")))
	{
		blast_fatal("Missing bundle 'name' property at line %ld", xmlGetLineNo(bundle_xml));
		return NULL;
	}

	bundle_hash = blast_hash_key(bundle_name, xmlStrlen(bundle_name));
	stream_hash = (board_val << 8) | ((wire_val - 1) << 4) | (channel_val - 1);


	if ((stream_packetsize = xmlGetProp(stream_xml, (xmlChar*)"packetsize")))
	{
		packetsize = (uint32_t)strtoul((char*)stream_packetsize,NULL,10);
		xmlFree(stream_packetsize);
	}
	if ((stream_rate = xmlGetProp(stream_xml, (xmlChar*)"rate")))
	{
		if (xmlStrcasecmp(stream_rate, (const xmlChar*)"full"))
				rate = strtod((char*)stream_rate,NULL);
		xmlFree(stream_rate);
	}

	/**
	 * Find the bundle first and then assign the new stream
	 */
	if ((cache_node = blast_sl_find(slinger_cache_sl, bundle_hash)))
	{
		if (cache_node->type != SLINGER_CACHE_TYPE_BUNDLE)
		{
			blast_err("Name and/or hash collision for %s (with %s) at line %ld.  Expected bundle.",
					bundle_name, cache_node->name, xmlGetLineNo(bundle_xml));

			blast_fatal("Hash collisions are fatal errors.  Rename either value to resolve.  Exiting");
			return NULL;
		}
		if (cache_node->name && strcmp((char*)bundle_name, cache_node->name))
		{
			blast_fatal("Hash collision for bundle %s with stream %s. Rename either value to resolve.  Exiting",
					(char*)bundle_name, cache_node->name);
			return NULL;
		}

		if (!cache_node->output_bundle->num_streams++)
			cache_node->output_bundle->streams = balloc(err, sizeof(slinger_stream_node_t*));
		else
			cache_node->output_bundle->streams = reballoc(err, cache_node->output_bundle->streams, cache_node->output_bundle->num_streams * sizeof(slinger_stream_node_t*));

		new_stream = balloc(err, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);
		cache_node->output_bundle->streams[cache_node->output_bundle->num_streams - 1] = new_stream;

		memset(new_stream, 0, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);
		new_stream->downlink.hash = stream_hash;
		new_stream->packetsize = packetsize;
		new_stream->bundle = cache_node;
		new_stream->delta_t = (int64_t)(100000.0 / rate);

		/**
		 * After setting up the extant bundle, we create a new node in the cache list for the stream.
		 */
		cache_node = balloc(err, sizeof(slinger_cache_node_t));
		blast_ZERO_P(cache_node);
		cache_node->type = SLINGER_CACHE_TYPE_STREAM;
		if (asprintf(&cache_node->name, "board%03" PRIu32 "_wire%" PRIu32 "_ch%02" PRIu32, board_val, wire_val, channel_val) < 0)
		{
			blast_fatal("Could not allocate string memory!");
			slinger_free_cache_node(cache_node);
			BLAST_SAFE_FREE(bundle_name);
			return NULL;
		}
		cache_node->output_stream = new_stream;

		slinger_add_to_cache_sl(stream_hash, cache_node);

		slinger_add_bolo_channel(stream_hash >> 8, stream_hash & 0xff, cache_node);
	}
	else
	{
		blast_fatal("Malformed slinger XML file.  This will likely not work");
		m_node = NULL;
	}

	blast_XML_SAFE_FREE(bundle_name);
	return m_node;
}

slinger_cache_node_t *slinger_xml_add_bundle (char *m_name, uint32_t m_timeout, uint8_t m_priority)
{
	slinger_cache_node_t *cache_node;
	uint32_t name_hash;

	name_hash = blast_hash_key(m_name, strlen(m_name));

	if ((cache_node = blast_sl_find(slinger_cache_sl, name_hash)))
	{
		blast_fatal("Collision for %s (duplicate values?)", m_name);
		return NULL;
	}
	cache_node = balloc(err, sizeof(slinger_cache_node_t));
	blast_ZERO_P(cache_node);
	cache_node->type = SLINGER_CACHE_TYPE_BUNDLE;
	cache_node->name = (char*)m_name;

	cache_node->output_bundle = balloc(err, sizeof(slinger_bundle_node_t));
	blast_ZERO_P(cache_node->output_bundle);

	cache_node->output_bundle->hash = name_hash;
	cache_node->output_bundle->timeout = m_timeout;
	pthread_mutex_init(&cache_node->output_bundle->mutex, NULL);
	cache_node->output_bundle->last_sent = time(NULL);
	cache_node->output_bundle->priority = m_priority;

	slinger_add_to_cache_sl(name_hash, cache_node);

	return cache_node;
}

void *slinger_xml_tag_bundle_callback(void *m_node)
{
	xmlNodePtr node = (xmlNodePtr)m_node;
	xmlChar *priority = NULL;
	xmlChar *timeout = NULL;
	xmlChar *name = NULL;

	uint8_t priority_val = UINT8_MAX;
	uint32_t timeout_val = UINT16_MAX;
	slinger_cache_node_t *cache_node;

	if (!(name = xmlGetProp(node, (xmlChar*)"name")))
	{
		blast_fatal("Missing 'name' attribute at line %ld", xmlGetLineNo(node));
		return NULL;
	}
	if ((timeout = xmlGetProp(node, (xmlChar*)"timeout")))
	{
		timeout_val = (uint32_t)strtoul((char*)timeout,NULL,0);
		blast_XML_SAFE_FREE(timeout);
	}
	if ((priority = xmlGetProp(node, (xmlChar*)"priority")))
	{
		priority_val = (uint8_t)min(UINT8_MAX, strtoul((char*)priority,NULL,0));
		blast_XML_SAFE_FREE(priority);
	}

	if (!timeout_val || timeout_val > UINT16_MAX) timeout_val = UINT16_MAX;

	if (!(cache_node = slinger_xml_add_bundle((char*)name, timeout_val, priority_val)))
	{
		blast_XML_SAFE_FREE(name);
		return NULL;
	}

	if (slinger_xml_process_raw_node(node) <= 0)
	{
		blast_err("Could not process bundle %s", name);
		blast_sl_remove(slinger_cache_sl, cache_node->output_bundle->hash);
		slinger_free_cache_node(cache_node);
		return NULL;
	}

	slinger_xml_parse_groups(node, cache_node);

	return m_node;
}

void *slinger_xml_tag_periodic_callback(void *m_node)
{
	xmlNodePtr periodic_xml = (xmlNodePtr) m_node;

	xmlChar *periodic_name_tmp;
	char *periodic_name;

	xmlChar *periodic_type_tmp;
	char *periodic_type;

	xmlChar *periodic_priority_tmp;
	char *periodic_priority;

	xmlChar *periodic_timeout_tmp;
	char *periodic_timeout;

	slinger_cache_node_t  *cache_node = NULL;
	uint32_t timeout_val = UINT16_MAX;
	uint32_t priority_val = UINT8_MAX;

	uint32_t periodic_hash;

#define GET_STACK_VAR(_varname) \
    ({                                                                                      \
        if (!(periodic_ ## _varname ## _tmp = xmlGetProp(periodic_xml, (xmlChar*)"name")))  \
         {                                                                                  \
             blast_fatal("Missing periodic '" #_varname                                     \
	                     "' property at line %ld", xmlGetLineNo(periodic_xml));             \
             return NULL;                                                                   \
         }                                                                                  \
         blast_tmp_sprintf(periodic_ ## _varname, "%s", periodic_##_varname##_tmp);         \
         xmlFree(periodic_ ## _varname ## _tmp);                                            \
    })

	GET_STACK_VAR(name);
	GET_STACK_VAR(type);
	GET_STACK_VAR(priority);
	GET_STACK_VAR(timeout);

	periodic_hash = blast_hash_key(periodic_name, strlen(periodic_name));

    timeout_val = (uint32_t) strtoul(periodic_timeout, NULL, 10);
	if (!timeout_val || timeout_val > UINT16_MAX) timeout_val = UINT16_MAX;

	priority_val = (uint8_t) min(UINT8_MAX, strtoul(periodic_priority, NULL, 10));

	cache_node = balloc(fatal, sizeof(slinger_cache_node_t));
	BLAST_ZERO_P(cache_node);
	cache_node->type = SLINGER_CACHE_TYPE_PERIODIC;
	cache_node->name = (char*)periodic_name;
	cache_node->output_periodic = balloc(fatal, sizeof(slinger_periodic_node_t));
	BLAST_ZERO_P(cache_node->output_periodic);

	cache_node->output_periodic->downlink.hash = periodic_hash;
	cache_node->output_periodic->priority =
	cache_node->output_periodic->last_sent = time(NULL);
	cache_node->output_periodic->timeout = timeout_val;

	if (!xmlStrcasecmp(periodic_type, (const xmlChar*)"channel"))
	{
		if (!slinger_add_frame_channel((const char*)periodic_name, cache_node))
		{
			blast_fatal("Frame channel %s unknown", (const char*)periodic_name);
			slinger_free_cache_node(cache_node);
			return NULL;
		}
	}
	else if (!xmlStrcasecmp(periodic_type, (const xmlChar*)"dfmux_slow"))
	{
		if (!slinger_add_slow_packet_field((const char*)periodic_name, cache_node))
		{
			blast_fatal("Slow packet stream %s unknown", (const char*)periodic_name);
			slinger_free_cache_node(cache_node);
			return NULL;
		}
	}
	else
	{
		blast_fatal("Unknown periodic type %s", (const char*)periodic_name);
		slinger_free_cache_node(cache_node);
		return NULL;
	}

	slinger_add_to_cache_sl(periodic_hash, cache_node);
	slinger_xml_parse_groups(periodic_xml, cache_node);

	return m_node;
}

static void slinger_xml_add_missing_channels(void)
{
	char *bundle_name;
	slinger_cache_node_t *cache_node;
	slinger_stream_node_t *new_stream;
	size_t packetsize = 2048;
	uint32_t name_hash;

	for (int i = 0; i < ccTotal; ++i)
	{
		name_hash = blast_hash_key(NiosLookup[i].field, strlen(NiosLookup[i].field));
		cache_node = blast_sl_find(slinger_cache_sl, name_hash);
		if (cache_node) continue;

		asprintf(&bundle_name, "%s_bundle", NiosLookup[i].field);
		cache_node = slinger_xml_add_bundle(bundle_name, UINT16_MAX, UINT8_MAX);
		if (!cache_node)
		{
			blast_err("Could not add %s", bundle_name);
			BLAST_SAFE_FREE(bundle_name);
			continue;
		}

		cache_node->output_bundle->streams = balloc(fatal, sizeof(slinger_stream_node_t*));
		cache_node->output_bundle->num_streams = 1;

		new_stream = balloc(fatal, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);
		memset(new_stream, 0, sizeof(slinger_stream_node_t) + packetsize + SLINGER_DOWNLINK_BUFFER_OVERHEAD);

		cache_node->output_bundle->streams[0] = new_stream;
		new_stream->downlink.hash = name_hash;
		new_stream->packetsize = packetsize;
		new_stream->bundle = cache_node;
		new_stream->delta_t = (int64_t) (100000.0 /  NiosLookup[i].fast?100.0:5.0);

		/**
		 * After setting up the extant bundle, we create a new node in the cache list for the stream.
		 */
		cache_node = balloc(fatal, sizeof(slinger_cache_node_t));
		blast_ZERO_P(cache_node);
		cache_node->type = SLINGER_CACHE_TYPE_STREAM;
		cache_node->name = bstrdup(err, NiosLookup[i].field);
		cache_node->output_stream = new_stream;

		slinger_add_frame_channel(cache_node->name, cache_node);
		name_hash = blast_hash_key(cache_node->name, strlen(cache_node->name));
		slinger_add_to_cache_sl(name_hash, cache_node);
	}

}
