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

/** \file
 * \ingroup bli
 *
 * A WAVL-tree (weak AVL-tree), balanced binary search tree.
 */

#include <stdlib.h>
#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_wavlTree.h"
#include "BLI_strict_flags.h"

BLI_INLINE bool binary(WAVLT_Node *node) 
{
  return (node->left != NULL) && (node->right != NULL);
}

BLI_INLINE bool unary(WAVLT_Node *node) 
{
  return (node->left != NULL) ^ (node->right != NULL);
}

BLI_INLINE bool leaf(WAVLT_Node *node) 
{
  return (node->left == NULL) && (node->right == NULL);
}

static WAVLT_Node *new_wavl_node_with_data(void *data)
{
  WAVLT_Node *new_node = MEM_callocN(sizeof(WAVLT_Node), __func__);
  new_node->parent = new_node->left = new_node->right = NULL;
  new_node->succ = new_node->pred = NULL;
  new_node->size = 1;
  new_node->rank = 0;
  new_node->data = data;

  return data;
}

/* Helper: performs a right rotation on the given node */
static void right_rotate(WAVLT_Tree *tree, WAVLT_Node *node)
{
  if (node->left == NULL) {
    return;
  }

  WAVLT_Node *x = node->left;
  WAVLT_Node *b = x->right;
  WAVLT_Node *p = node->parent;

  node->left = b;
  x->right = node;
  node->parent = x;
  x->parent = p;

  if (node == tree->root) {
    tree->root = x;
  }
}

/* Helper: performs a left rotation on the given node */
static void left_rotate(WAVLT_Tree *tree, WAVLT_Node *node)
{
  if (node->right == NULL) {
    return;
  }

  WAVLT_Node *x = node->right;
  WAVLT_Node *b = x->left;
  WAVLT_Node *p = node->parent;

  node->right = b;
  x->left = node;
  node->parent = x;
  x->parent = p;

  if (node == tree->root) {
    tree->root = x;
  }
}

BLI_INLINE void demote(WAVLT_Node *node)
{
  node->rank--;
}

BLI_INLINE void promote(WAVLT_Node *node)
{
  node->rank++;
}

/** \} */

/** \name Public WAVl-tree API
 * \{ */

WAVLT_Tree *BLI_wavlTree_new()
{
  WAVLT_Tree *new_tree = MEM_mallocN(sizeof(WAVLT_Tree), __func__);
  WAVLT_Node *external_node = MEM_mallocN(sizeof(WAVLT_Node), __func__);
  new_tree->root = new_tree->min_node = new_tree->max_node = NULL;

  return new_tree;
}

void BLI_wavlTree_free(WAVLT_Tree *tree)
{
  if (tree == NULL) {
    return;
  }

  MEM_freeN(tree);
}

bool BLI_wavlTree_empty(const WAVLT_Tree *tree)
{
  return tree->root == NULL;
}

size_t BLI_wavlTree_size(const WAVLT_Tree *tree)
{
  return (tree->root != NULL) ? tree->root->size : 0;
}

WAVLT_Node *BLI_wavlTree_search(const WAVLT_Tree *tree, WAVLT_comparator_FP cmp, void *search_data)
{
  WAVLT_Node *current = tree->root;
  while (current != NULL) {
    short c = cmp(current->data, search_data);
    if (c == 0) {
      return current;
    }
    else if (c == -1) {
      current = current->left;
    }
    else {
      current = current->right;
    }
  }
  return NULL;
}

void BLI_wavlTree_insert(WAVLT_Tree *tree, WAVLT_comparator_FP cmp, void *data)
{
  /* if tree is empty, make root */
  if (tree->root == NULL) {
    WAVLT_Node *new_node = new_wavl_node_with_data(data);
    tree->root = new_node;
    tree->max_node = new_node;
    tree->max_node = new_node;
  }

  bool insert_left;
  WAVLT_Node *current = tree->root;
  while (current != NULL) {
    /* compare data */
    short c = cmp(current->data, data);
    /* node already exists */
    if (c == 0) {
      return;
    }
    else if (c == -1) {
      /* insert to the left of current */
      if (current->left == NULL) {
        insert_left = true;
        break;
      }
      current = current->left;
    }
    else {
      /* insert to the right of current */
      if (current->right == NULL) {
        insert_left = false;
        break;
      }
      current = current->right;
    }
  }

  /* create a new node to insert */
  WAVLT_Node *new_node = new_wavl_node_with_data(data);
  if (insert_left) {
    if (tree->min_node == current) {
      tree->min_node = new_node;
    }
  }
  else {
    if (tree->max_node == current) {
      tree->max_node = new_node;
    }
  }
}