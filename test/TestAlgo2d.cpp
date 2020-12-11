#include "gtest/gtest.h"

#include <cmath>
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
  ASSERT_NEAR(intersect13->x(), 0, cg::EPS);
  ASSERT_NEAR(intersect13->y(), 0, cg::EPS);
  cg::Line2d l4({10,10});
  cg::Line2d l5{{1,0},{1, 5}};
  auto intersect45 = cg::intersectPoint(l4,l5);
  ASSERT_NEAR(intersect45->x(), 1, cg::EPS);
  ASSERT_NEAR(intersect45->y(), 1, cg::EPS);

  ASSERT_FALSE(cg::intersects(l1,l2));
  ASSERT_EQ(cg::intersectPoint(l1,l2), nullptr);
  cg::Line2d l6({2,-1},{3,-2});
  ASSERT_FALSE(cg::intersects(l1,l6));
}

TEST(Algo, basic_algos){
  cg::Point2d pt1(5,5);
  cg::Point2d pt2(1,5);
  cg::Point2d pt3(1,0);
  cg::Point2d pt4(0.5,0);
  cg::Point2d pt5(0,-5);

  cg::Line2d l1({1,0});

  // Shortest distance of point to line
  auto dist1 = distancePointToLine(pt1, l1);
  auto dist2 = distancePointToLine(pt2, l1);
  auto dist3 = distancePointToLine(pt3, l1);
  auto dist4 = distancePointToLine(pt4, l1);
  auto dist5 = distancePointToLine(pt5, l1);

  ASSERT_NEAR(dist1, 5.0, cg::EPS);
  ASSERT_NEAR(dist2, 5.0, cg::EPS);
  ASSERT_NEAR(dist3, 0.0, cg::EPS);
  ASSERT_NEAR(dist4, 0.0, cg::EPS);
  ASSERT_NEAR(dist5, 5.0, cg::EPS);

  // Shortest distance of point to line segment
  auto dls1 = distancePointToLineSegment(pt1, l1);
  auto dls2 = distancePointToLineSegment(pt2, l1);
  auto dls3 = distancePointToLineSegment(pt3, l1);
  auto dls4 = distancePointToLineSegment(pt4, l1);
  auto dls5 = distancePointToLineSegment(pt5, l1);

  ASSERT_NEAR(dist1, sqrt(26), cg::EPS);
  ASSERT_NEAR(dist2, 5.0, cg::EPS);
  ASSERT_NEAR(dist3, 0.0, cg::EPS);
  ASSERT_NEAR(dist4, 0.0, cg::EPS);
  ASSERT_NEAR(dist5, 5.0, cg::EPS);

  // Check polygon convex
  // Area of polygon (shoelace method)
  // Point in polygon
  // Clip polygon by line
  // Convex hull of point set

}
