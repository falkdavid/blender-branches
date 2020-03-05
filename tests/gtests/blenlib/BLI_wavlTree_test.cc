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

TEST(wavlTree, newTree) 
{
  WAVLT_Tree *tree;
  tree = BLI_wavlTree_new();
  EXPECT_EQ(BLI_wavlTree_size(tree), 0);
}