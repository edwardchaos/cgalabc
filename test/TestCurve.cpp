#include <gtest/gtest.h>

#include "include/BezierCurve.h"
#include "include/CatmullRomCurve.h"

TEST(BezierCurve, construct){
  cg::BezierCurve b({0,0},{1,0},{0,1},{1,1},0.01);

  auto verts = b.vertices();
  ASSERT_EQ(verts.size(), 101);
  ASSERT_EQ(verts[0], cg::Point2d(0,0));
  ASSERT_EQ(verts.back(), cg::Point2d(1,0));
  ASSERT_DOUBLE_EQ(verts[50].x(), 0.5);
  ASSERT_DOUBLE_EQ(verts[50].y(), 0.75);

  ASSERT_EQ(b.edges().size(), 100);
}

TEST(CatmullRom, construct){
  cg::CatmullRomCurve cmr({0,0},{0,1},{1,1},{1,0}, 0.01);

  auto verts = cmr.vertices();
  ASSERT_EQ(verts.size(), 101);
  ASSERT_EQ(verts[0], cg::Point2d(0,1));
  ASSERT_EQ(verts.back(), cg::Point2d(1,1));
  ASSERT_DOUBLE_EQ(verts[50].x(), 0.5);
  ASSERT_DOUBLE_EQ(verts[50].y(), 1.125);

  ASSERT_EQ(cmr.edges().size(), 100);
}