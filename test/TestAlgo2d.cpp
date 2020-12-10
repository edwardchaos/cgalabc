#include "gtest/gtest.h"

#include "type.h"
#include "Algo2d.h"

TEST(Algo, basic_properties){
  ASSERT_TRUE(true);

  cg::Line2d l1({1,0});
  cg::Line2d l2({0,1}, {1,1});
  cg::Line2d l3({0,1});

  ASSERT_TRUE(cg::isParallel(l1, l2));
  ASSERT_FALSE(cg::isParallel(l1,l3));
  ASSERT_TRUE(cg::isOrthogonal(l1,l3));

  ASSERT_FALSE(cg::intersects(l1,l2));
  ASSERT_TRUE(cg::intersects(l1,l3));
  // Get intersection point of 2 lines
  auto intersect13 = cg::intersectPoint(l1,l3);
  ASSERT_NEAR(intersect13.x(), 0, cg::EPS);
  ASSERT_NEAR(intersect13.y(), 0, cg::EPS);
  cg::Line2d l4({10,10});
  cg::Line2d l5{{1,0},{1, 5}};
  auto intersect45 = cg::intersectPoint(l4,l5);
  ASSERT_NEAR(intersect45.x(), 1, cg::EPS);
  ASSERT_NEAR(intersect45.y(), 1, cg::EPS);
}

TEST(Algo, basic_algos){
  // Shortest distance of point to line
  // Shortest distance of point to line segment
  // Check polygon convex
  // Point in polygon
  // Clip polygon by line
  // Convex hull of point set

}
