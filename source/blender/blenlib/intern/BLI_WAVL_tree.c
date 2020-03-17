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

/** \name Internal Functions
 * \{ */

BLI_INLINE bool unary(WAVLT_Node *node) 
{
  return (node->left != NULL) ^ (node->right != NULL);
}

BLI_INLINE bool leaf(WAVLT_Node *node) 
{
  return (node->left == NULL) && (node->right == NULL);
}

BLI_INLINE void demote(WAVLT_Node *node)
{
  node->rank--;
}

BLI_INLINE void promote(WAVLT_Node *node)
{
  node->rank++;
}

/* Helper: check if the node is an i-child (rank diff with parent is i) */
BLI_INLINE bool is_i_child(int i, WAVLT_Node *node)
{
  int diff = (node->parent != NULL) ? node->parent->rank - node->rank : -1;
  if (diff == i) {
    return true;
  }
  return false;
}

/* Helper: check if the left child has rank diff i */
BLI_INLINE bool left_child_is_i(int i, WAVLT_Node *p)
{
  int diff = (p->left != NULL) ? p->rank - p->left->rank : p->rank + 1;
  if (diff == i) {
    return true;
  }
  return false;
}

/* Helper: check if the right child has rank diff i */
BLI_INLINE bool right_child_is_i(int i, WAVLT_Node *p)
{
  int diff = (p->right != NULL) ? p->rank - p->right->rank : p->rank + 1;
  if (diff == i) {
    return true;
  }
  return false;
}

/* Helper: check if the node is of type l,r (rank diff of l/r-child is l/r) */
BLI_INLINE bool is_type(int l, int r, WAVLT_Node *node)
{
  /* note: if a child is NULL (external) it's rank is -1 
   * so the diff is node->rank - (-1) = node->rank + 1*/
  int l_diff = (node->left != NULL) ? node->rank - node->left->rank : node->rank + 1;
  int r_diff = (node->right != NULL) ? node->rank - node->right->rank : node->rank + 1;
  if ((l == l_diff) && (r == r_diff)) {
    return true;
  }
  return false;
}

BLI_INLINE WAVLT_Node *sibling(WAVLT_Node *node)
{
  if (node->parent != NULL) {
    if (node->parent->left == node) {
      return node->parent->right;
    }
    else {
      return node->parent->left;
    }
  }
  return NULL;
}

BLI_INLINE void update_size(WAVLT_Node *node)
{
  uint new_size = 1;
  if (node->left != NULL) {
    new_size += node->left->size;
  }
  if (node->right != NULL) {
    new_size += node->right->size;
  }
  node->size = new_size;
}

/* returns true if the deletion of the given node does 
 * not cause a violation of the rank rule */
BLI_INLINE bool is_optimal_for_deletion(WAVLT_Node *node)
{
  if (leaf(node)) {
    if (is_i_child(1, node) && unary(node->parent)) {
      return false;
    }
  }
  else {
    if (is_i_child(2, node)) {
      return false;
    }
  }
  return true;
}

static WAVLT_Node *new_wavl_node_with_data(void *data)
{
  WAVLT_Node *new_node = MEM_mallocN(sizeof(WAVLT_Node), __func__);
  new_node->parent = new_node->left = new_node->right = NULL;
  new_node->succ = new_node->pred = NULL;
  new_node->size = 1;
  new_node->rank = 0;
  new_node->data = data;

  return new_node;
}

static void free_wavl_node(WAVLT_Node *node, WAVLT_free_data_FP free_data)
{
  if (node == NULL) {
    return;
  }

  if (free_data != NULL) {
    free_data(node->data);
  }
  MEM_freeN(node);
}

/* Helper: performs a right rotation on the child of the given node */
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

  if (b != NULL) {
    b->parent = node;
  }
  
  if (p != NULL) {
    if (p->left == node) {
      p->left = x;
    }
    else {
      p->right = x;
    }
  }
  else {
    tree->root = x;
  }

  update_size(node);
  update_size(x);
}

/* Helper: performs a left rotation on the child of the given node */
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

  if (b != NULL) {
    b->parent = node;
  }

  if (p != NULL) {
    if (p->left == node) {
      p->left = x;
    }
    else {
      p->right = x;
    }
  }
  else {
    tree->root = x;
  }

  update_size(node);
  update_size(x);
}

/* Helper: Rebalance the tree after a insertion. The WAVLT_Node 'node' is
 * the parent of the inserted node */
static void rebalance_insert(WAVLT_Tree *tree, WAVLT_Node *node)
{
  if (node == NULL) {
    return;
  }
  /* repeat case 1: simple promotion */
  while (is_type(0, 1, node) || is_type(1, 0, node)) {
    promote(node);
    update_size(node);
    if (node->parent == NULL) {
      break;
    }
    node = node->parent;
  }

  /* case 2: tree rotations */
  if (is_type(0, 2, node)) {
    /* rotate right */
    if (is_type(1, 2, node->left)) {
      right_rotate(tree, node);
      demote(node);
    }
    /* double rotate left */
    else if (is_type(2, 1, node->left)) {
      left_rotate(tree, node->left);
      right_rotate(tree, node);
      demote(node);
      demote(node->parent->left);
      promote(node->parent);
    }
  }
  else if (is_type(2, 0, node)){
    /* rotate left */
    if (is_type(2, 1, node->right)) {
      left_rotate(tree, node);
      demote(node);
    }
    /* double rotate right */
    else if (is_type(1, 2, node->right)) {
      right_rotate(tree, node->right);
      left_rotate(tree, node);
      demote(node);
      demote(node->parent->right);
      promote(node->parent);
    }
  }

  /* propagate the new size to the top */
  while (node != NULL) {
    update_size(node);
    node = node->parent;
  }
}

/* Helper: delete leaf node and return node from where to rebalance the tree, if it exists */
static WAVLT_Node *delete_leaf_node(WAVLT_Tree *tree, WAVLT_free_data_FP free_data, WAVLT_Node *node)
{
  WAVLT_Node *parent = NULL;
  if (tree->root == node) {
    tree->root = NULL;
  }
  else {
      /* update parent */
    parent = node->parent;
    if (parent->left == node) {
      parent->left = NULL;
    }
    else {
      parent->right = NULL;
    }
  }

  /* update min and max nodes */
  if (tree->min_node == node) {
    tree->min_node = node->succ;
  }
  else if (tree->max_node == node) {
    tree->max_node = node->pred;
  }

  /* update successor and predecessor */
  if (node->pred != NULL) {
    node->pred->succ = node->succ;
  }
  if (node->succ != NULL) {
    node->succ->pred = node->pred;
  }

  free_wavl_node(node, free_data);
  return parent;
}

/* Helper: delete unary node and return node from where to rebalance the tree, if it exists */
static WAVLT_Node *delete_unary_node(WAVLT_Tree *tree, WAVLT_free_data_FP free_data, WAVLT_Node *node)
{
  WAVLT_Node *parent = NULL;
  WAVLT_Node *child = (node->left != NULL) ? node->left : node->right; //note: this cannot be null, as node is unary
  if (tree->root == node) {
    tree->root = child;
    child->parent = NULL;
  }
  else {
    /* update parent */
    parent = node->parent;
    if (parent->left == node) {
      parent->left = child;
    }
    else {
      parent->right = child;
    }
    child->parent = parent;
  }

  /* update min and max nodes */
  if (tree->min_node == node) {
    tree->min_node = node->succ;
  }
  else if (tree->max_node == node) {
    tree->max_node = node->pred;
  }

  /* update successor and predecessor */
  if (node->pred != NULL) {
    node->pred->succ = node->succ;
  }
  if (node->succ != NULL) {
    node->succ->pred = node->pred;
  }

  free_wavl_node(node, free_data);
  return parent;
}

/* Helper: delete binary node and return node from where to rebalance the tree, if it exists */
static WAVLT_Node *delete_binary_node(WAVLT_Tree *tree, WAVLT_free_data_FP free_data, WAVLT_Node *node)
{
  /* note that node->pred and node->succ can never be NULL because node is binary */
  WAVLT_Node *replace_node = NULL;
  if(is_optimal_for_deletion(node->pred)) {
    replace_node = node->pred;
  }
  else {
    replace_node = node->succ;
  }

  /* replace node data */
  if (free_data != NULL) {
    free_data(node->data);
  }
  node->data = replace_node->data;
  
  /* replace_node must be leaf or unary */
  if (leaf(replace_node)) {
    /* dont free data as it is used by 'node' now. */
    replace_node = delete_leaf_node(tree, NULL, replace_node);
  }
  else {
    replace_node = delete_unary_node(tree, NULL, replace_node);
  }

  return replace_node;
}

/* Helper: Rebalance the tree after a deletion. 'node' is leaf or unary */
static void rebalance_delete(WAVLT_Tree *tree, WAVLT_Node *node)
{
  if (node == NULL) {
    return;
  }
  /* case 1: node is leaf and type 2,2 */
  if (leaf(node) && is_type(2, 2, node)) {
    demote(node);
    update_size(node);
    node = node->parent;
  }

  /* case 2: node has a 3-child */
  while (node != NULL) {
    if (right_child_is_i(3, node)) {
      if (left_child_is_i(2, node)) {
        demote(node);
      }
      else if (is_type(2, 2, node->left)) {
        demote(node->left);
        demote(node);
      }
      else {
        update_size(node);
        break;
      }
      update_size(node);
      node = node->parent;
    }
    else if (left_child_is_i(3, node)) {
      if (right_child_is_i(2, node)) {
        demote(node);
      }
      else if (is_type(2, 2, node->right)) {
        demote(node->right);
        demote(node);
      }
      else {
        update_size(node);
        break;
      }
      update_size(node);
      node = node->parent;
    }
    else {
      update_size(node);
      break;
    }
  }

  if (node == NULL) {
    return;
  }

  /* case 3: rotations */
  if (is_type(1, 3, node)) {
    WAVLT_Node *sib = node->left;
    if (left_child_is_i(1, sib)) {
      right_rotate(tree, node);
      promote(sib);
      demote(node);
      if (leaf(node)) {
        demote(node);
      }
    }
    else if (is_type(2, 1, sib)) {
      WAVLT_Node *v = sib->right;
      left_rotate(tree, sib);
      right_rotate(tree, node);
      promote(v);
      promote(v);
      demote(sib);
      demote(node);
      demote(node);
    }
  }
  else if (is_type(3, 1, node)) {
    WAVLT_Node *sib = node->right;
    if (right_child_is_i(1, sib)) {
      left_rotate(tree, node);
      promote(sib);
      demote(node);
      if (leaf(node)) {
        demote(node);
      }
    }
    else if (is_type(1, 2, sib)) {
      WAVLT_Node *v = sib->left;
      right_rotate(tree, sib);
      left_rotate(tree, node);
      promote(v);
      promote(v);
      demote(sib);
      demote(node);
      demote(node);
    }
  }

  /* propagate the new size to the top */
  while (node != NULL) {
    update_size(node);
    node = node->parent;
  }
}

/** \} */

/** \name Public WAVL-tree API
 * \{ */

/**
 * Creates a new WAVL tree.
 */
WAVLT_Tree *BLI_wavlTree_new()
{
  WAVLT_Tree *new_tree = MEM_mallocN(sizeof(WAVLT_Tree), __func__);
  new_tree->root = new_tree->min_node = new_tree->max_node = NULL;

  return new_tree;
}

/**
 * Frees the tree. If the WAVLT_free_data_FP function is not NULL,
 * it will free all of the data in the tree using this function for each
 * element.
 */
void BLI_wavlTree_free(WAVLT_Tree *tree, WAVLT_free_data_FP free_data)
{
  if (tree == NULL) {
    return;
  }

  WAVLT_Node *succ;
  if (free_data != NULL) {
    for (WAVLT_Node *curr = tree->min_node; curr != NULL; curr = succ) {
      succ = curr->succ;
      free_data(curr->data);
      MEM_freeN(curr);
    }
  }
  else {
    for (WAVLT_Node *curr = tree->min_node; curr != NULL; curr = succ) {
      succ = curr->succ;
      MEM_freeN(curr);
    }
  }

  MEM_freeN(tree);
}

/**
 * Returns true or false depending on wether the tree is empty or not.
 */
bool BLI_wavlTree_empty(const WAVLT_Tree *tree)
{
  return (tree != NULL) ? tree->root == NULL : true;
}

/**
 * Returns the number of nodes contained in the tree.
 */
uint BLI_wavlTree_size(const WAVLT_Tree *tree)
{
  if (tree != NULL) {
    return (tree->root != NULL) ? tree->root->size : 0;
  }
  return 0;
}

/**
 * Node search:
 * Searches for a particular data node (search_data) using the WAVLT_comparator_FP function.
 * Returns the WAVLT_Node if it was found, NULL otherwise.
 */
WAVLT_Node *BLI_wavlTree_search(const WAVLT_Tree *tree, WAVLT_comparator_FP cmp, void *search_data)
{
  WAVLT_Node *current = tree->root;
  while (current != NULL) {
    short c = cmp(search_data, current->data);
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

WAVLT_Node *BLI_wavlTree_successor_ex(const WAVLT_Node *node)
{
  return node->succ;
}

WAVLT_Node *BLI_wavlTree_successor(const WAVLT_Tree *tree, WAVLT_comparator_FP cmp, void *data)
{
  WAVLT_Node *node = BLI_wavlTree_search(tree, cmp, data);
  if (node != NULL) {
    return BLI_wavlTree_successor_ex(node);
  }
  return NULL;
}

WAVLT_Node *BLI_wavlTree_predecessor_ex(const WAVLT_Node *node)
{
  return node->pred;
}

WAVLT_Node *BLI_wavlTree_predecessor(const WAVLT_Tree *tree, WAVLT_comparator_FP cmp, void *data)
{
  WAVLT_Node *node = BLI_wavlTree_search(tree, cmp, data);
  if (node != NULL) {
    return BLI_wavlTree_predecessor_ex(node);
  }
  return NULL;
}

WAVLT_Node *BLI_wavlTree_min(const WAVLT_Tree *tree)
{
  return tree->min_node;
}

WAVLT_Node *BLI_wavlTree_max(const WAVLT_Tree *tree)
{
  return tree->max_node;
}

void *BLI_wavlTree_min_data(const WAVLT_Tree *tree)
{
  return tree->min_node->data;
}

void *BLI_wavlTree_max_data(const WAVLT_Tree *tree)
{
  return tree->max_node->data;
}

/**
 * Node insert:
 * Inserts a new node (data) into the tree using the WAVLT_comparator_FP function
 * to compare the nodes. If the data to be inserted has the same key as a node
 * that is already in the tree, the node will not be inserted and the function
 * returns NULL. If the node does not exists it will be inserted and a pointer
 * to the node returned.
 */
WAVLT_Node *BLI_wavlTree_insert(WAVLT_Tree *tree, WAVLT_comparator_FP cmp, void *data)
{
  WAVLT_Node *new_wavl_node = NULL;
  /* if tree is empty, make root */
  if (tree->root == NULL) {
    new_wavl_node = new_wavl_node_with_data(data);
    tree->root = new_wavl_node;
    tree->min_node = new_wavl_node;
    tree->max_node = new_wavl_node;
    return new_wavl_node;
  }

  /* create a new node to insert */
  new_wavl_node = new_wavl_node_with_data(data);

  /* search for the leaf node to insert */
  bool insert_left;
  WAVLT_Node *insert_node = tree->root;
  while (insert_node != NULL) {
    /* compare data */
    short c = cmp(data, insert_node->data);
    /* node already exists */
    if (c == 0) {
      return NULL;
    }
    else if (c == -1) {
      /* insert to the left of insert_node */
      if (insert_node->left == NULL) {
        insert_left = true;
        break;
      }
      insert_node = insert_node->left;
    }
    else {
      /* insert to the right of insert_node */
      if (insert_node->right == NULL) {
        insert_left = false;
        break;
      }
      insert_node = insert_node->right;
    }
  }

  if (insert_left) {
    /* update tree links */
    insert_node->left = new_wavl_node;
    new_wavl_node->parent = insert_node;

    /* update successor and predecessor */
    WAVLT_Node *curr_pred = insert_node->pred;
    if (curr_pred != NULL) {
      curr_pred->succ = new_wavl_node;
    }
    insert_node->pred = new_wavl_node;
    new_wavl_node->pred = curr_pred;
    new_wavl_node->succ = insert_node;

    /* if insert before min, update min */
    if (tree->min_node == insert_node) {
      tree->min_node = new_wavl_node;
    }
  }
  else {
    /* update tree links */
    insert_node->right = new_wavl_node;
    new_wavl_node->parent = insert_node;

    /* update successor and predecessor */
    WAVLT_Node *curr_succ = insert_node->succ;
    if (curr_succ != NULL) {
      curr_succ->pred = new_wavl_node;
    }
    insert_node->succ = new_wavl_node;
    new_wavl_node->succ = curr_succ;
    new_wavl_node->pred = insert_node;

    /* if insert after max, update max */
    if (tree->max_node == insert_node) {
      tree->max_node = new_wavl_node;
    }
  }

  rebalance_insert(tree, insert_node);
  return new_wavl_node;
}

/**
 * Update node:
 * Deletes the node while keeping the data, then inserts a new node with the saved data.
 * Used to update a node if the data inside has changed.
 * Returns a pointer to the updated node.
 */
WAVLT_Node *BLI_wavlTree_update_node(WAVLT_Tree *tree, WAVLT_comparator_FP cmp, WAVLT_Node *node)
{
  if (node == NULL) {
    return NULL;
  }
  /* save pointer to data */
  void *data = node->data;
  /* delete node, but don't free data */
  BLI_wavlTree_delete_node(tree, NULL, node);
  WAVLT_Node *new_node = BLI_wavlTree_insert(tree, cmp, data);
  return new_node;
}

void BLI_wavlTree_delete_node(WAVLT_Tree *tree, WAVLT_free_data_FP free_data, WAVLT_Node *node)
{
  WAVLT_Node *replace_node = NULL;
  if (leaf(node)) {
    replace_node = delete_leaf_node(tree, free_data, node);
  }
  else if (unary(node)) {
    replace_node = delete_unary_node(tree, free_data, node);
  }
  else {
    replace_node = delete_binary_node(tree, free_data, node);
  }
  
  /* rebalance the tree if there exists a replace_node node */
  if (replace_node != NULL) {
    rebalance_delete(tree, replace_node);
  }
}

/**
 * Delete node:
 * Finds the node containing "data" and removes it from the tree.
 */
void BLI_wavlTree_delete(WAVLT_Tree *tree, WAVLT_comparator_FP cmp, WAVLT_free_data_FP free_data, void *data)
{
  WAVLT_Node *delete_node = BLI_wavlTree_search(tree, cmp, data);
  if (delete_node == NULL) {
    return;
  }
  BLI_wavlTree_delete_node(tree, free_data, delete_node);
}
/** \} */
