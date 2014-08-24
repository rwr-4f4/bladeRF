/*
 * Copyright (C) 2014 Nuand LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#ifndef METADATA_H_
#define METADATA_H_

/* Components of the metadata header */
#define METADATA_RESV_SIZE      (sizeof(uint32_t))
#define METADATA_TIMESTAMP_SIZE (sizeof(uint64_t))
#define METADATA_FLAGS_SIZE     (sizeof(uint32_t))

#define METADATA_RESV_OFFSET        0
#define METADATA_TIMESTAMP_OFFSET   (METADATA_RESV_SIZE)
#define METADATA_FLAGS_OFFSET       (METADATA_TIMESTAMP_OFFSET + \
                                     METADATA_TIMESTAMP_SIZE)

#define METADATA_HEADER_SIZE        (METADATA_FLAGS_OFFSET + \
                                     METADATA_FLAGS_SIZE)



static inline uint64_t metadata_get_timestamp(const uint8_t *header)
{
   uint64_t ret;
   assert(sizeof(ret) == METADATA_TIMESTAMP_SIZE);
   memcpy(&ret, &header[METADATA_TIMESTAMP_OFFSET], METADATA_TIMESTAMP_SIZE);
   return ret;
}

static inline uint64_t metadata_get_flags(const uint8_t *header)
{
   uint32_t ret;
   assert(sizeof(ret) == METADATA_FLAGS_SIZE);
   memcpy(&ret, &header[METADATA_FLAGS_OFFSET], METADATA_FLAGS_SIZE);
   return ret;
}

#endif
