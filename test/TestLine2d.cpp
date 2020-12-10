#include "gtest/gtest.h"

#include "cgalabc.hpp"

TEST(case1, function1){
  cg::Point2d mypoint(2,3);
  EXPECT_TRUE(true);
  mypoint.print();
  ASSERT_TRUE(false);
}