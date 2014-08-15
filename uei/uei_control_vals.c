/* 
 * uei_control_vals.c: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Aug 13, 2014 by seth
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <gdsl.h>
#include <PMurHash.h>

#include "uei_control_vals.h"

gdsl_hash_t uei_vals_ht;

ulong uei_vals_hash(const char* m_data)
{
    varentry_t *data = (varentry_t*)m_data;

    return PMurHash32(0x00, data->name, strnlen(data->name, 32));

}

const char* uei_vals_key (gdsl_element_t m_data)
{
    varentry_t *data = (varentry_t*)m_data;
    return data->name;
}

int uei_vals_initialize(void)
{
    uei_vals_ht = gdsl_hash_alloc("UEI Vals", NULL, NULL, uei_vals_key, uei_vals_hash, 11);

    return 0;
}

int uei_vals_add(e_vartypes m_type, const char *m_name, void *m_data)
{
    void *ret;

    varentry_t *new_entry = NULL;

    new_entry = malloc(sizeof(varentry_t));
    memset(new_entry, 0, sizeof(varentry_t));

    new_entry->type = m_type;
    new_entry->data = m_data;
    strncpy(new_entry->name, m_name, sizeof(new_entry->name) - 1);

    ret = gdsl_hash_insert(uei_vals_ht,new_entry);
    if (ret == NULL) free(new_entry);

    return (ret != NULL);
}

void uei_vals_set(const char *m_name, void *m_data)
{
    varentry_t *entry = NULL;

    entry = (varentry_t*)gdsl_hash_search(uei_vals_ht, m_name);
    if (entry) {
        switch (entry->type) {
            case TYPE_UINT8:
                *(uint8_t*)(entry->data) = *(uint8_t*)entry->data;
                break;
            case TYPE_UINT16:
                *(uint16_t*)(entry->data) = *(uint16_t*)entry->data;
                break;
            case TYPE_UINT32:
                *(uint32_t*)(entry->data) = *(uint32_t*)entry->data;
                break;
            case TYPE_UINT64:
                *(uint64_t*)(entry->data) = *(uint64_t*)entry->data;
                break;
            case TYPE_INT8:
                *(int8_t*)(entry->data) = *(int8_t*)entry->data;
                break;
            case TYPE_INT16:
                *(int16_t*)(entry->data) = *(int16_t*)entry->data;
                break;
            case TYPE_INT32:
                *(int32_t*)(entry->data) = *(int32_t*)entry->data;
                break;
            case TYPE_INT64:
                *(int64_t*)(entry->data) = *(int64_t*)entry->data;
                break;
            case TYPE_FLOAT:
                *(float*)(entry->data) = *(float*)entry->data;
                break;
            case TYPE_DOUBLE:
                *(double*)(entry->data) = *(double*)entry->data;
                break;
            default:
                printf("Tried to set invalid data type %d for %s\n", entry->type, entry->name);
                return;
        }
        printf("Update value for %s\n", entry->name);
    }
}
