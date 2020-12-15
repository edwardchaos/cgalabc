#include <gtest/gtest.h>

#include "type.h"

TEST(Point, operations){
  cg::Point2d O(0,0);
  cg::Point2d A(1,0);
  cg::Point2d E(1,1);
  cg::Point2d B(0,1);
  cg::Point2d F(-1,1);
  cg::Point2d C(-1,0);
  cg::Point2d G(-1,-1);
  cg::Point2d D(0,-1);
  cg::Point2d H(1,-1);

  ASSERT_EQ(A+C, O);
  ASSERT_EQ(B+D, O);
  ASSERT_EQ(A+B, E);
  ASSERT_EQ(C+B, F);
  ASSERT_EQ(C+D, G);
  ASSERT_EQ(A+D, H);
  ASSERT_EQ(A+B, B+A);
  ASSERT_EQ(E-A, B);
  ASSERT_EQ(E-B , A);
  ASSERT_EQ(O-A, C);
  ASSERT_EQ(O-B , D);
}