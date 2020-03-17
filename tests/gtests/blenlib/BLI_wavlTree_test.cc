/* Apache License, Version 2.0 */

#include "testing/testing.h"
#include <string.h>
#include <math.h>

#include "MEM_guardedalloc.h"

extern "C" {
#include "BLI_compiler_attrs.h"
#include "BLI_wavlTree.h"
#include "BLI_utildefines.h"
#include "BLI_rand.h"
};

/* ************************************************************************ */
/* TEST HELPER FUNCTIONS */

typedef struct SampleData {
  int my_data;
} SampleData;

static SampleData *create_sample_node(int data)
{
  SampleData *sample_data = (SampleData *)MEM_callocN(sizeof(SampleData), __func__);
  sample_data->my_data = data;
  return sample_data;
}

static void free_sample_data(void *data)
{
  SampleData *s = (SampleData *)data;
  MEM_freeN(s);
}

static short cmp_sample_data(void *data_a, void *data_b)
{
  SampleData *s1 = (SampleData *)data_a;
  SampleData *s2 = (SampleData *)data_b;
  if (s1->my_data > s2->my_data) {
    return 1;
  }
  else if (s1->my_data < s2->my_data) {
    return -1;
  }
  else {
    return 0;
  }
}

static SampleData **create_sample_range(int num_items)
{
  SampleData **range = (SampleData **)MEM_callocN(sizeof(SampleData *) * num_items, __func__);
  for (int i = 0; i < num_items; i++) {
    range[i] = create_sample_node(i);
  }
  return range;
}

static void free_sample_range(SampleData **range, int num_items)
{
  for (int i = 0; i < num_items; i++) {
    free_sample_data(range[i]);
  }
  MEM_freeN(range);
}

static int tree_height(WAVLT_Node *root)
{
  if (root != NULL) {
    int lh = tree_height(root->left);
    int rh = tree_height(root->right);
    return lh > rh ? lh + 1: rh + 1;
  }
  return 0;
}

static void debug_print_tree_rec(WAVLT_Node *root, int space)
{
  if (root == NULL) {
    return;
  }
  space += 8;
  debug_print_tree_rec(root->right, space);
  for (int i = 8; i < space; i++) {
    std::cout << " ";
  }
  int diff = (root->parent != NULL) ? root->parent->rank - root->rank : -1;
  std::cout << "(" << diff << ") " << ((SampleData *)(root->data))->my_data;
  std::cout << " r:" << root->rank << "\n";
  debug_print_tree_rec(root->left, space);
}

static void debug_print_tree(WAVLT_Tree *tree)
{
  if (tree->root == NULL) {
    return;
  }
  std::cout << "Tree: " << "\n";
  debug_print_tree_rec(tree->root, 2);
  std::cout << "\n";
}

static bool tree_ordered(WAVLT_Tree *tree)
{
  if (BLI_wavlTree_empty(tree)) {
    return true;
  }

  int min = ((SampleData *)BLI_wavlTree_min_data(tree))->my_data;
  WAVLTREE_INORDER(SampleData *, curr, tree) {
    if (curr->my_data < min) {
      return false;
    }
    else {
      min = curr->my_data;
    }
  }
  return true;
}

static bool check_height_rule(WAVLT_Tree *tree)
{
  if (BLI_wavlTree_empty(tree)) {
    return true;
  }

  int num_nodes = BLI_wavlTree_size(tree);
  int h = tree_height(tree->root);
  // WAVL trees height should be less than log_(golden ratio)(n)
  // This can be simplified
  int limit = (int)ceil(2.078087f * log(num_nodes));
  //std::cout << "Height: " << h << ", nodes: " << num_nodes << ", limit: " << limit << "\n";
  return h < limit;
}

static bool check_rank_rule(WAVLT_Tree *tree)
{
  for (WAVLT_Node *curr = tree->min_node; curr != NULL; curr = curr->succ) {
    if (curr != tree->root) {
      int rank_diff = curr->parent->rank - curr->rank;
      // All rank differences should be 1 or 2
      if (rank_diff != 1 && rank_diff != 2) {
        return false;
      }
    }

    if ((curr->left == NULL) && (curr->right == NULL) && curr->rank != 0) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************ */
/* TEST CASES */

TEST(wavlTree, newTree) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  EXPECT_EQ(0, BLI_wavlTree_size(tree));
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
}

TEST(wavlTree, freeEmptyTree) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, insertSingle) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(1, BLI_wavlTree_size(tree));
  EXPECT_EQ(s1, tree->root->data);
  EXPECT_EQ(s1, BLI_wavlTree_min_data(tree));
  EXPECT_EQ(s1, BLI_wavlTree_max_data(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, insertSame) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  SampleData *s2 = create_sample_node(10);
  /* inserting elem with same key is not allowed */
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2); // this should just return

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(1, BLI_wavlTree_size(tree));
  EXPECT_EQ(s1, tree->root->data);
  EXPECT_EQ(s1, BLI_wavlTree_min_data(tree));
  EXPECT_EQ(s1, BLI_wavlTree_max_data(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, twoInsertLeft) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  SampleData *s2 = create_sample_node(5);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(2, BLI_wavlTree_size(tree));
  EXPECT_EQ(s1, tree->root->data);
  EXPECT_EQ(s2, BLI_wavlTree_min_data(tree));
  EXPECT_EQ(s1, BLI_wavlTree_max_data(tree));

  WAVLT_Node *s1n = BLI_wavlTree_search(tree, cmp_sample_data, s1);
  WAVLT_Node *s2n = BLI_wavlTree_search(tree, cmp_sample_data, s2);

  EXPECT_TRUE(s1n != NULL);
  EXPECT_TRUE(s2n != NULL);

  EXPECT_EQ(s1, s1n->data);
  EXPECT_EQ(s2, s2n->data);

  EXPECT_EQ(tree->root, s1n);
  EXPECT_EQ(s1n->left, s2n);

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, twoInsertRight) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  SampleData *s2 = create_sample_node(15);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(2, BLI_wavlTree_size(tree));
  EXPECT_EQ(s1, tree->root->data);
  EXPECT_EQ(s1, BLI_wavlTree_min_data(tree));
  EXPECT_EQ(s2, BLI_wavlTree_max_data(tree));

  WAVLT_Node *s1n = BLI_wavlTree_search(tree, cmp_sample_data, s1);
  WAVLT_Node *s2n = BLI_wavlTree_search(tree, cmp_sample_data, s2);

  EXPECT_TRUE(s1n != NULL);
  EXPECT_TRUE(s2n != NULL);

  EXPECT_EQ(s1, s1n->data);
  EXPECT_EQ(s2, s2n->data);

  EXPECT_EQ(tree->root, s1n);
  EXPECT_EQ(s1n->right, s2n);

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, leftRotate) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  SampleData *s2 = create_sample_node(15);
  SampleData *s3 = create_sample_node(25);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s3);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(3, BLI_wavlTree_size(tree));

  WAVLT_Node *s1n = BLI_wavlTree_search(tree, cmp_sample_data, s1);
  WAVLT_Node *s2n = BLI_wavlTree_search(tree, cmp_sample_data, s2);
  WAVLT_Node *s3n = BLI_wavlTree_search(tree, cmp_sample_data, s3);

  //debug_print_tree(tree);

  EXPECT_EQ(tree->root, s2n);
  EXPECT_EQ(s2n->left, s1n);
  EXPECT_EQ(s2n->right, s3n);
  EXPECT_EQ(s1n->parent, s2n);
  EXPECT_EQ(s3n->parent, s2n);

  EXPECT_EQ(s2n->size, 3);
  EXPECT_EQ(s1n->size, 1);
  EXPECT_EQ(s3n->size, 1);

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, rightRotate) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  SampleData *s2 = create_sample_node(5);
  SampleData *s3 = create_sample_node(3);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s3);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(3, BLI_wavlTree_size(tree));

  WAVLT_Node *s1n = BLI_wavlTree_search(tree, cmp_sample_data, s1);
  WAVLT_Node *s2n = BLI_wavlTree_search(tree, cmp_sample_data, s2);
  WAVLT_Node *s3n = BLI_wavlTree_search(tree, cmp_sample_data, s3);

  //debug_print_tree(tree);

  EXPECT_EQ(tree->root, s2n);
  EXPECT_EQ(s2n->left, s3n);
  EXPECT_EQ(s2n->right, s1n);
  EXPECT_EQ(s1n->parent, s2n);
  EXPECT_EQ(s3n->parent, s2n);

  EXPECT_EQ(s2n->size, 3);
  EXPECT_EQ(s1n->size, 1);
  EXPECT_EQ(s3n->size, 1);

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, doubleLeftRotate) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(5);
  SampleData *s2 = create_sample_node(6);
  SampleData *s3 = create_sample_node(4);
  SampleData *s4 = create_sample_node(1);
  SampleData *s5 = create_sample_node(2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s3);
  BLI_wavlTree_insert(tree, cmp_sample_data, s4);
  BLI_wavlTree_insert(tree, cmp_sample_data, s5);

  //debug_print_tree(tree);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(5, BLI_wavlTree_size(tree));

  WAVLT_Node *s1n = BLI_wavlTree_search(tree, cmp_sample_data, s1);
  WAVLT_Node *s2n = BLI_wavlTree_search(tree, cmp_sample_data, s2);
  WAVLT_Node *s3n = BLI_wavlTree_search(tree, cmp_sample_data, s3);
  WAVLT_Node *s4n = BLI_wavlTree_search(tree, cmp_sample_data, s4);
  WAVLT_Node *s5n = BLI_wavlTree_search(tree, cmp_sample_data, s5);

  EXPECT_EQ(tree->root, s1n);
  EXPECT_EQ(s2n, s1n->right);
  EXPECT_EQ(s5n, s1n->left);
  EXPECT_EQ(s3n, s5n->right);
  EXPECT_EQ(s4n, s5n->left);

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, doubleRightRotate) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(5);
  SampleData *s2 = create_sample_node(1);
  SampleData *s3 = create_sample_node(6);
  SampleData *s4 = create_sample_node(9);
  SampleData *s5 = create_sample_node(8);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s3);
  BLI_wavlTree_insert(tree, cmp_sample_data, s4);
  BLI_wavlTree_insert(tree, cmp_sample_data, s5);

  //debug_print_tree(tree);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(5, BLI_wavlTree_size(tree));

  WAVLT_Node *s1n = BLI_wavlTree_search(tree, cmp_sample_data, s1);
  WAVLT_Node *s2n = BLI_wavlTree_search(tree, cmp_sample_data, s2);
  WAVLT_Node *s3n = BLI_wavlTree_search(tree, cmp_sample_data, s3);
  WAVLT_Node *s4n = BLI_wavlTree_search(tree, cmp_sample_data, s4);
  WAVLT_Node *s5n = BLI_wavlTree_search(tree, cmp_sample_data, s5);

  EXPECT_EQ(tree->root, s1n);
  EXPECT_EQ(s2n, s1n->left);
  EXPECT_EQ(s5n, s1n->right);
  EXPECT_EQ(s3n, s5n->left);
  EXPECT_EQ(s4n, s5n->right);

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, doubleRightRotate2) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(2);
  SampleData *s2 = create_sample_node(1);
  SampleData *s3 = create_sample_node(6);
  SampleData *s4 = create_sample_node(4);
  SampleData *s5 = create_sample_node(5);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s3);
  BLI_wavlTree_insert(tree, cmp_sample_data, s4);
  BLI_wavlTree_insert(tree, cmp_sample_data, s5);

  //debug_print_tree(tree);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(5, BLI_wavlTree_size(tree));

  WAVLT_Node *s1n = BLI_wavlTree_search(tree, cmp_sample_data, s1);
  WAVLT_Node *s2n = BLI_wavlTree_search(tree, cmp_sample_data, s2);
  WAVLT_Node *s3n = BLI_wavlTree_search(tree, cmp_sample_data, s3);
  WAVLT_Node *s4n = BLI_wavlTree_search(tree, cmp_sample_data, s4);
  WAVLT_Node *s5n = BLI_wavlTree_search(tree, cmp_sample_data, s5);

  EXPECT_EQ(tree->root, s1n);
  EXPECT_EQ(s2n, s1n->left);
  EXPECT_EQ(s5n, s1n->right);
  EXPECT_EQ(s3n, s5n->right);
  EXPECT_EQ(s4n, s5n->left);

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

static void insert_array_into_tree(WAVLT_Tree *tree, int *array, int length)
{
  for (int i = 0; i < length; i++) {
    int elem = array[i];
    SampleData *s = create_sample_node(elem);
    BLI_wavlTree_insert(tree, cmp_sample_data, s);
    //debug_print_tree(tree);
  }
}

TEST(wavlTree, doubleRightRotate3) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  int elems[7] = {2, 9, 8, 7, 1, 6, 4};
  insert_array_into_tree(tree, elems, 7);
  //debug_print_tree(tree);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(7, BLI_wavlTree_size(tree));

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

static void random_insert_helper(int num_items, int rng_seed)
{
  WAVLT_Tree *tree = BLI_wavlTree_new();
  SampleData **range = create_sample_range(num_items);
  BLI_array_randomize(range, sizeof(SampleData *), num_items, rng_seed);
  for (int i = 0; i < num_items; i++) {
    SampleData *data = range[i];
    BLI_wavlTree_insert(tree, cmp_sample_data, data);
    EXPECT_TRUE(tree_ordered(tree));
    EXPECT_TRUE(check_rank_rule(tree));
  }

  EXPECT_TRUE(check_height_rule(tree));

  BLI_wavlTree_free(tree, NULL);
  free_sample_range(range, num_items);
}

TEST(wavlTree, insert20)
{
  random_insert_helper(20, 1234);
}

TEST(wavlTree, insert100)
{
  random_insert_helper(100, 4567);
}

TEST(wavlTree, insert1000)
{
  random_insert_helper(1000, 7890);
}

TEST(wavlTree, deleteRoot1) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_delete(tree, cmp_sample_data, free_sample_data, s1);

  EXPECT_TRUE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(0, BLI_wavlTree_size(tree));
  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, deleteRoot2) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  SampleData *s2 = create_sample_node(15);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_delete(tree, cmp_sample_data, free_sample_data, s1);

  //debug_print_tree(tree);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(1, BLI_wavlTree_size(tree));
  EXPECT_EQ(s2, tree->root->data);
  EXPECT_EQ(s2, BLI_wavlTree_min_data(tree));
  EXPECT_EQ(s2, BLI_wavlTree_max_data(tree));
  EXPECT_EQ(NULL, tree->root->pred);
  EXPECT_EQ(NULL, tree->root->succ);

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, deleteRoot3) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  SampleData *s2 = create_sample_node(5);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_delete(tree, cmp_sample_data, free_sample_data, s1);

  //debug_print_tree(tree);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(1, BLI_wavlTree_size(tree));
  EXPECT_EQ(s2, tree->root->data);
  EXPECT_EQ(s2, BLI_wavlTree_min_data(tree));
  EXPECT_EQ(s2, BLI_wavlTree_max_data(tree));
  EXPECT_EQ(NULL, tree->root->pred);
  EXPECT_EQ(NULL, tree->root->succ);

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, deleteRoot4) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(5);
  SampleData *s2 = create_sample_node(3);
  SampleData *s3 = create_sample_node(7);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s3);
  BLI_wavlTree_delete(tree, cmp_sample_data, free_sample_data, s1);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(2, BLI_wavlTree_size(tree));
  EXPECT_EQ(s2, BLI_wavlTree_min_data(tree));
  EXPECT_EQ(s3, BLI_wavlTree_max_data(tree));

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

TEST(wavlTree, deleteTwo) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(5);
  SampleData *s2 = create_sample_node(3);
  SampleData *s3 = create_sample_node(7);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s3);
  BLI_wavlTree_delete(tree, cmp_sample_data, free_sample_data, s1);
  BLI_wavlTree_delete(tree, cmp_sample_data, free_sample_data, s2);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(1, BLI_wavlTree_size(tree));

  EXPECT_TRUE(tree_ordered(tree));
  EXPECT_TRUE(check_rank_rule(tree));

  BLI_wavlTree_free(tree, free_sample_data);
}

static void random_delete_helper(int num_items, int rng_seed)
{
  WAVLT_Tree *tree = BLI_wavlTree_new();
  SampleData **range = create_sample_range(num_items);
  for (int i = 0; i < num_items; i++) {
    SampleData *data = range[i];
    BLI_wavlTree_insert(tree, cmp_sample_data, data);
  }
  debug_print_tree(tree);
  BLI_array_randomize(range, sizeof(SampleData *), num_items, rng_seed);
  for (int i = 0; i < num_items; i++) {
    SampleData *data = range[i];
    std::cout << "Delete: " << data->my_data << "\n";
    BLI_wavlTree_delete(tree, cmp_sample_data, free_sample_data, data);
    debug_print_tree(tree);
    EXPECT_TRUE(tree_ordered(tree));
    EXPECT_TRUE(check_rank_rule(tree));
    EXPECT_EQ(num_items - (i + 1), BLI_wavlTree_size(tree));
  }
  
  EXPECT_EQ(0, BLI_wavlTree_size(tree));
  EXPECT_TRUE(BLI_wavlTree_empty(tree));

  BLI_wavlTree_free(tree, NULL);
  MEM_freeN(range);
}

TEST(wavlTree, delete10)
{
  random_delete_helper(16, 4567);
}

TEST(wavlTree, delete100)
{
  random_delete_helper(100, 1234);
}

TEST(wavlTree, delete1000)
{
  random_delete_helper(1000, 7890);
}