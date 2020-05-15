/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef __BLI_HEAP_H__
#define __BLI_HEAP_H__

/** \file
 * \ingroup bli
 * \brief A min-heap / priority queue ADT
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "BLI_math.h"

struct Heap;
struct HeapNode;
typedef struct Heap Heap;
typedef struct HeapNode HeapNode;

typedef void (*HeapFreeFP)(void *ptr);
typedef short (*HeapComparatorFP)(void *dataA, void *dataB);

Heap *BLI_heap_cmp_new_ex(unsigned int tot_reserve) ATTR_WARN_UNUSED_RESULT;
Heap *BLI_heap_cmp_new(void) ATTR_WARN_UNUSED_RESULT;
void BLI_heap_cmp_clear(Heap *heap, HeapFreeFP ptrfreefp) ATTR_NONNULL(1);
void BLI_heap_cmp_free(Heap *heap, HeapFreeFP ptrfreefp) ATTR_NONNULL(1);
HeapNode *BLI_heap_cmp_insert(Heap *heap, HeapComparatorFP cmp, void *ptr) ATTR_NONNULL(1);
void BLI_heap_cmp_insert_or_update(Heap *heap, HeapNode **node_p, HeapComparatorFP cmp, void *ptr)
    ATTR_NONNULL(1, 2);
void BLI_heap_cmp_remove(Heap *heap, HeapComparatorFP cmp, HeapNode *node) ATTR_NONNULL(1, 3);
bool BLI_heap_cmp_is_empty(const Heap *heap) ATTR_NONNULL(1);
unsigned int BLI_heap_cmp_len(const Heap *heap) ATTR_WARN_UNUSED_RESULT ATTR_NONNULL(1);
HeapNode *BLI_heap_cmp_top(const Heap *heap) ATTR_WARN_UNUSED_RESULT ATTR_NONNULL(1);
void *BLI_heap_cmp_top_ptr(const Heap *heap) ATTR_WARN_UNUSED_RESULT ATTR_NONNULL(1);
void *BLI_heap_cmp_pop_min(Heap *heap, HeapComparatorFP cmp) ATTR_NONNULL(1, 2);
void BLI_heap_cmp_node_value_update_ptr(Heap *heap,
                                        HeapNode *node,
                                        HeapComparatorFP cmp,
                                        void *ptr) ATTR_NONNULL(1, 2);

void *BLI_heap_cmp_node_ptr(const HeapNode *heap) ATTR_WARN_UNUSED_RESULT ATTR_NONNULL(1);
/* only for gtest */
bool BLI_heap_cmp_is_valid(const Heap *heap, HeapComparatorFP cmp);

#ifdef __cplusplus
}
#endif

#endif /* __BLI_HEAP_H__ */
