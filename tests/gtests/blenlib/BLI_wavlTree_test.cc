/* Apache License, Version 2.0 */

#include "testing/testing.h"
#include <string.h>

#include "MEM_guardedalloc.h"

extern "C" {
#include "BLI_compiler_attrs.h"
#include "BLI_wavlTree.h"
#include "BLI_utildefines.h"
#include "BLI_rand.h"
};

/* ************************************************************************ */
/* HELPER FUNCTIONS */

typedef struct SampleData {
  int my_data;
} SampleData;

static SampleData *create_sample_node(int data)
{
  SampleData *new_data = (SampleData *)MEM_callocN(sizeof(SampleData), __func__);
  new_data->my_data = data;
  return new_data;
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

static int tree_height(WAVLT_Node *root)
{
  if (root != NULL) {
    return tree_height(root->left) + tree_height(root->right) + 1;
  }
  return 0;
}

static void debug_print_tree_layer(WAVLT_Node *root, int level)
{
  if (root == NULL) {
    std::cout << "    ";
    return;
  }
  if (level == 1) {
    std::cout << ((SampleData *)(root->data))->my_data << "  ";
  }
  else if (level > 1) {
    debug_print_tree_layer(root->left, level - 1);
    debug_print_tree_layer(root->right, level - 1);
  }
}

static void debug_print_tree(WAVLT_Tree *tree)
{
  if (tree->root == NULL) {
    return;
  }
  std::cout << "Tree: \n";
  int h = tree_height(tree->root);
  for (int i = 1; i < h; i++) {
    debug_print_tree_layer(tree->root, i);
    std::cout << "\n";
  }
}

static bool tree_ordered(WAVLT_Tree *tree)
{
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
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
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
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
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
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
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

  BLI_wavlTree_free(tree, free_sample_data);
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
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

  BLI_wavlTree_free(tree, free_sample_data);
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
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

  EXPECT_EQ(s2n->size, 3);
  EXPECT_EQ(s1n->size, 1);
  EXPECT_EQ(s3n->size, 1);

  EXPECT_TRUE(tree_ordered(tree));

  BLI_wavlTree_free(tree, free_sample_data);
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
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

  EXPECT_EQ(s2n->size, 3);
  EXPECT_EQ(s1n->size, 1);
  EXPECT_EQ(s3n->size, 1);

  EXPECT_TRUE(tree_ordered(tree));

  BLI_wavlTree_free(tree, free_sample_data);
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
}

TEST(wavlTree, doubleLeftRotate) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(10);
  SampleData *s2 = create_sample_node(5);
  SampleData *s3 = create_sample_node(7);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s3);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(3, BLI_wavlTree_size(tree));

  WAVLT_Node *s1n = BLI_wavlTree_search(tree, cmp_sample_data, s1);
  WAVLT_Node *s2n = BLI_wavlTree_search(tree, cmp_sample_data, s2);
  WAVLT_Node *s3n = BLI_wavlTree_search(tree, cmp_sample_data, s3);

  //debug_print_tree(tree);

  EXPECT_EQ(tree->root, s3n);
  EXPECT_EQ(s3n->left, s2n);
  EXPECT_EQ(s3n->right, s1n);

  EXPECT_EQ(s3n->size, 3);
  EXPECT_EQ(s1n->size, 1);
  EXPECT_EQ(s2n->size, 1);

  EXPECT_TRUE(tree_ordered(tree));

  BLI_wavlTree_free(tree, free_sample_data);
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
}

TEST(wavlTree, doubleRightRotate) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  SampleData *s1 = create_sample_node(3);
  SampleData *s2 = create_sample_node(10);
  SampleData *s3 = create_sample_node(7);
  BLI_wavlTree_insert(tree, cmp_sample_data, s1);
  BLI_wavlTree_insert(tree, cmp_sample_data, s2);
  BLI_wavlTree_insert(tree, cmp_sample_data, s3);

  EXPECT_FALSE(BLI_wavlTree_empty(tree));
  EXPECT_EQ(3, BLI_wavlTree_size(tree));

  WAVLT_Node *s1n = BLI_wavlTree_search(tree, cmp_sample_data, s1);
  WAVLT_Node *s2n = BLI_wavlTree_search(tree, cmp_sample_data, s2);
  WAVLT_Node *s3n = BLI_wavlTree_search(tree, cmp_sample_data, s3);

  //debug_print_tree(tree);

  EXPECT_EQ(tree->root, s3n);
  EXPECT_EQ(s3n->left, s1n);
  EXPECT_EQ(s3n->right, s2n);

  EXPECT_EQ(s3n->size, 3);
  EXPECT_EQ(s1n->size, 1);
  EXPECT_EQ(s2n->size, 1);

  EXPECT_TRUE(tree_ordered(tree));

  BLI_wavlTree_free(tree, free_sample_data);
  EXPECT_TRUE(BLI_wavlTree_empty(tree));
}