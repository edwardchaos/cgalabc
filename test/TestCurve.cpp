#include <gtest/gtest.h>

#include "BezierCurve.h"

TEST(BezierCurve, construct){
  cg::BezierCurve b({0,0},{1,0},{0,1},{1,1},0.01);

  auto verts = b.vertices();
  ASSERT_EQ(verts.size(), 101);
  ASSERT_EQ(verts[0], cg::Point2d(0,0));
  ASSERT_EQ(verts.back(), cg::Point2d(1,0));
  ASSERT_NEAR(verts[51].x(), 0.5, 0.1);
  ASSERT_NEAR(verts[51].y(), 0.75, 0.1);

  ASSERT_EQ(b.edges().size(), 100);
}