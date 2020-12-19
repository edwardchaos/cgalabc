#include "gtest/gtest.h"

#include <cmath>
#include "type.h"
#include "Algo2d.h"

TEST(Algo, basic_properties){
  ASSERT_TRUE(true);

  cg::Line2d l1({1,0});
  cg::Line2d rightof_l1{{2,0},{3,0}};
  cg::Line2d l2({0,1}, {1,1});
  cg::Line2d l3({0,1});
  cg::Line2d ontop_l3({0,2},{0,3});
  cg::Line2d diag1({1,1});
  cg::Line2d toprightof_diag2({2,2},{3,3});

  ASSERT_FALSE(cg::intersects(l3, ontop_l3));
  ASSERT_FALSE(cg::intersects(l1,rightof_l1));
  ASSERT_FALSE(cg::intersects(diag1,toprightof_diag2));

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
  cg::Point2d origin(0,0);
  cg::Point2d pt1(5,5);
  cg::Point2d pt2(1,5);
  cg::Point2d pt3(1,0);
  cg::Point2d pt4(0.5,0);
  cg::Point2d pt5(0,-5);
  cg::Point2d above_pt5(0,10);

  cg::Line2d l1({1,0});

  // Angle between 3 points
  ASSERT_NEAR(M_PI/2.0, cg::angle(pt3,pt2,pt1), cg::EPS);
  ASSERT_NEAR(1.6704649792860586,cg::angle(pt4,pt2,pt1), cg::EPS);
  ASSERT_NEAR(M_PI, cg::angle(origin, pt4, pt3), cg::EPS);
  ASSERT_NEAR(0, cg::angle(pt4,origin,pt3), cg::EPS);
  ASSERT_NEAR(M_PI, cg::angle(pt5,origin,above_pt5), cg::EPS);
  ASSERT_NEAR(0, cg::angle(origin, pt5, above_pt5), cg::EPS);

  // Collinear points
  ASSERT_NEAR(0,cg::angle(pt2,pt3,cg::Point2d(1,10)),cg::EPS);

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

  ASSERT_NEAR(dls1, sqrt(pow(4,2)+pow(5,2)), cg::EPS);
  ASSERT_NEAR(dls2, 5.0, cg::EPS);
  ASSERT_NEAR(dls3, 0.0, cg::EPS);
  ASSERT_NEAR(dls4, 0.0, cg::EPS);
  ASSERT_NEAR(dls5, 5.0, cg::EPS);

  // Point on the standard equation of a line but not on the segment
  cg::Line2d l2{{1,0},{2,0}};
  auto dist6 = cg::distancePointToLineSegment(origin, l2);
  ASSERT_NEAR(dist6, 1.0, cg::EPS);
}

TEST(Algo, convex_hull){
  // Create Triangle
  std::vector<cg::Point2d> simple_triangle{{0,0}, {1,0}, {0,1}};
  std::vector<cg::Point2d> simple_triangle_with_points{
      {0,0}, {1,0}, {0,1}, // Convex hull points
      {0.25,0.25}, {0.25,0.5}, {0.5,0.25} // Additional points contained within
  };
  cg::Polygon right_triangle{{0,0},{1,0},{0,1}};

  cg::Polygon_ptr ch1;
  try{
    ch1 = cg::convexHull(simple_triangle);
  }catch(...){
    FAIL() << "Should not throw any exceptions";
  }
  ASSERT_NE(ch1, nullptr);
  ASSERT_EQ(*ch1, right_triangle);

  // Create arbitrary points
  std::vector<cg::Point2d> pointset2{
      {3,-8}, {1,1}, {-1,-4}, {-5,-5}, {-3,4}, {2,6},
      {7,2}, {4,-3}, {8,-7}
  };
  cg::Polygon gt2{{-5,-5},{3,-8},{8,-7},{7,2},{2,6},{-3,4}};

  cg::Polygon_ptr ch2;
  try{
    ch2 = cg::convexHull(pointset2);
  }catch(std::exception &e){
    FAIL() << e.what();
  }
  ASSERT_NE(ch2, nullptr);
  ASSERT_EQ(*ch2, gt2);

  // Pointset with collinear points
  std::vector<cg::Point2d> pointset3{
      // Points of convex hull
      {3.5,3.5},{4,1},{1,2},{1,-1},{2,-1},{-1,-1},
      // Additional points
      {0.5,0.5},{1.5,1.5},{2,1},{2.5,0.5},{2,0}
  };
  cg::Polygon gt3{{-1,-1},{2,-1},{4,1},{3.5,3.5},{1,2},{-1,-1}};

  cg::Polygon_ptr ch3;
  try{
    ch3 = cg::convexHull(pointset3);
  }catch(std::exception &e){
    FAIL() << e.what();
  }
  ASSERT_NE(ch3, nullptr);
  ASSERT_EQ(*ch3, gt3);
}

TEST(Algo, lerp){
  cg::Point2d origin(0,0);
  cg::Point2d twotwo(2,2)

  cg::Point2d mid(1,1);
  cg::Point2d quart(0.5,0.5);
  cg::Point2d tquart(1.5, 1.5);

  ASSERT_DOUBLE_EQ(cg::lerp(origin,twotwo,0.5), mid);
  ASSERT_DOUBLE_EQ(cg::lerp(origin,twotwo,0.25), quart);
  ASSERT_DOUBLE_EQ(cg::lerp(origin,twotwo,0.75), tquart);
  ASSERT_DOUBLE_EQ(cg::lerp(origin,twotwo,0), origin);
  ASSERT_DOUBLE_EQ(cg::lerp(origin,twotwo,1), twotwo);
}
