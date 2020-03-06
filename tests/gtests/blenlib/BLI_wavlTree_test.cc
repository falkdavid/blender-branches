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

typedef struct TestData {
  int my_data;
} TestData;

static TestData *create_test_node(int data)
{
  TestData *new_data = (TestData *)MEM_callocN(sizeof(TestData), __func__);
  new_data->my_data = data;
  return new_data;
}

static void free_test_data(TestData *data)
{
  MEM_freeN(data);
}

static short comapre_test_data(TestData *data_a, TestData *data_b)
{
  if (data_a->my_data > data_a->my_data) {
    return 1;
  }
  else if (data_a->my_data < data_a->my_data) {
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
  BLI_wavlTree_free(tree);
}