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
  EXPECT_EQ(0, cmp_sample_data(s1, tree->root->data));
  EXPECT_EQ(0, cmp_sample_data(s1, tree->min_node->data));
  EXPECT_EQ(0, cmp_sample_data(s1, tree->max_node->data));

  BLI_wavlTree_free(tree, free_sample_data);
}