#include <gtest/gtest.h>

#include <stdexcept>
#include "Polygon.h"

TEST(Polygon, create_polygon){
  try {
    // Create 2 valid polygons
    cg::Polygon first{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    cg::Polygon firstsame{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    cg::Polygon different{{0, 0}, {10, 0}, {10, 10}, {0, 11}};
    std::vector<cg::Point2d> pointset2{{-5, -5}, {5, -5}, {5, 5}};
    cg::Polygon second{pointset2};

    ASSERT_NE(first,second); // Different number of vertices
    ASSERT_EQ(first,firstsame);
    ASSERT_NE(first,different); // Different vertex
  }catch(...){FAIL() << "No exception should be thrown.";}

  // Polygon with only 2 unique points
  try {
    cg::Polygon{{0, 0}, {0, 1}, {0, 0}};
    FAIL() << "Should throw invalid_argument";
  }catch(std::invalid_argument &e) {
    ASSERT_STREQ(e.what(),
                 "Cannot create polygon; Requires at least 3 unique vertices");
  }catch(...){FAIL() << "Incorrect exception thrown";}

  // Polygon with 2 unique points, excluding the last "closing the loop point"
  try{
    cg::Polygon{{0,0},{0,1}};
    FAIL() << "Should throw invalid_argument";
  }catch(std::invalid_argument &e) {
    ASSERT_STREQ(e.what(),
                 "Cannot create polygon; Requires at least 3 unique vertices");
  }catch(...) {FAIL() << "Incorrect exception thrown";}

  // Polygon with repeated intermediate vertices
  try{
    cg::Polygon{{0,0},{1,1},{0,1},{-1,0},{1,1}};
    FAIL() << "Should throw invalid argument";
  }catch(std::invalid_argument &e){
  }catch(...){
    FAIL() << "Expected invalid argument exception";
  }

  // Polygon with 2 overlapping edges
  try{
    cg::Polygon{{0,0},{1,0},{1,1},{1,0}};
    FAIL() << "Should throw invalid argument";
  }
  catch(std::invalid_argument &e){
  }
  catch(...){FAIL() << "Expected invalid_argument exception";}

  // Pair of edges in polygon intersect
  try {
    cg::Polygon{{0, 0}, {1, 0}, {1, 1}, {0, -1}};
    FAIL() << "Should throw invalid argument";
  }
  catch(std::invalid_argument &e){
    ASSERT_STREQ(e.what(), "Cannot create polygon; Non adjacent edges intersect");
  }
  catch(...){FAIL() << "Incorrect exception thrown";}

  // Adjacent edges overlap
  try{
    cg::Polygon{{0,0},{1,0},{2,0},{0,0}};
    FAIL() << "Should throw invalid argument";
  }catch(std::invalid_argument &e){
    ASSERT_STREQ(e.what(),"Cannot create polygon; Adjacent edges overlap");
  }catch(...){FAIL() << "Incorrect exception thrown";}

  //Valid polygons
  try{
    /*    _              |\/\/|
     *  _| |_            |    |
     * |_____|    and    |/\/\|
     */
    cg::Polygon{{0,0},{1,0},{2,0},{3,0},{3,1},{2,1},{2,2},{1,2},{1,1},{0,1}};
    cg::Polygon{{0,0},{1,1},{2,0},{3,1},{4,0},{4,4},{3,3},{2,4},{1,3},{0,4}};
  }catch(...){FAIL() << "Should not throw exception, these are valid polygons";}
}

TEST(Polygon, simple_algos){
  cg::Polygon convex_poly{{0,0},{1,1},{-1,1}};
  cg::Polygon nonconvex_poly{{0,0},{1,1},{1,3},{0,2},{-1,3}};
  cg::Polygon convex_poly2{{0,0},{1,0},{2,0},{3,0},{3,1},{2,1},{2,2},{1,2},{1,
                                                                         1},{0,
                                                                           1}};
  cg::Polygon nonconvex_poly2{{0,0},{1,1},{2,0},{3,1},{4,0},{4,4},{3,3},{2,
                                                                         4},{1,
                                                                         3},{0,4}};

  // Test convex
  ASSERT_TRUE(convex_poly.isConvex());
  ASSERT_FALSE(nonconvex_poly.isConvex());
  ASSERT_TRUE(convex_poly.isConvex());
  ASSERT_FALSE(nonconvex_poly2.isConvex());

  // Compute area
  ASSERT_NEAR(1.0, convex_poly.area(), cg::EPS);
  ASSERT_NEAR(3.0, nonconvex_poly.area(), cg::EPS);
  ASSERT_NEAR(4.0, convex_poly2.area(), cg::EPS);
  ASSERT_NEAR(12.0, nonconvex_poly2.area(), cg::EPS);

  // Contains point
  cg::Point2d pt1(0,0);
  cg::Point2d pt2(0,0.5);
  cg::Point2d pt3(0.5,0.5);
  cg::Point2d pt4(1,1);
  cg::Point2d pt5(-1,1);
  cg::Point2d pt6(1,-1);
  cg::Point2d pt7(0,2);
  cg::Point2d pt8(-0.5,0);
  cg::Point2d pt9(0,2.5);
  cg::Point2d pt10(-1,-1);
  cg::Point2d pt11(0,2);

  ASSERT_TRUE(convex_poly.containsPoint(pt1,true));
  ASSERT_FALSE(convex_poly.containsPoint(pt1,false));
  ASSERT_TRUE(convex_poly.containsPoint(pt2,true));
  ASSERT_TRUE(convex_poly.containsPoint(pt2,false));
  ASSERT_TRUE(convex_poly.containsPoint(pt3,true));
  ASSERT_FALSE(convex_poly.containsPoint(pt3,false));
  ASSERT_TRUE(convex_poly.containsPoint(pt4,true));
  ASSERT_FALSE(convex_poly.containsPoint(pt4,false));
  ASSERT_TRUE(convex_poly.containsPoint(pt5,true));
  ASSERT_FALSE(convex_poly.containsPoint(pt5,false));
  ASSERT_FALSE(convex_poly.containsPoint(pt6,true));
  ASSERT_FALSE(convex_poly.containsPoint(pt6,false));
  ASSERT_FALSE(convex_poly.containsPoint(pt7,true));
  ASSERT_FALSE(convex_poly.containsPoint(pt7,false));
  ASSERT_FALSE(convex_poly.containsPoint(pt8,true));
  ASSERT_FALSE(convex_poly.containsPoint(pt8,false));

  ASSERT_TRUE(nonconvex_poly.containsPoint(pt2,true));
  ASSERT_TRUE(nonconvex_poly.containsPoint(pt2,false));
  ASSERT_FALSE(nonconvex_poly.containsPoint(pt9, true));
  ASSERT_FALSE(nonconvex_poly.containsPoint(pt9, false));
  ASSERT_FALSE(nonconvex_poly.containsPoint(pt5, true));
  ASSERT_FALSE(nonconvex_poly.containsPoint(pt5, false));
  ASSERT_FALSE(nonconvex_poly.containsPoint(pt5, true));
  ASSERT_FALSE(nonconvex_poly.containsPoint(pt5, false));
  ASSERT_FALSE(nonconvex_poly.containsPoint(pt10, true));
  ASSERT_FALSE(nonconvex_poly.containsPoint(pt10, false));
  ASSERT_TRUE(nonconvex_poly.containsPoint(pt11, true));
  ASSERT_FALSE(nonconvex_poly.containsPoint(pt11, false));
}

TEST(Polygon, clipping){
  // clip polygon by line
  cg::Polygon triangle{{0,0},{5,0},{5,5}};
  cg::Line2d cut_line1({0,-5},{5,0});
  cg::Line2d two_point_cut({2.5,0},{5,2.5});
  cg::Line2d cut_through_vertex({2.5,0},{5,5});
  cg::Line2d vertical_cut({2.5,-10},{2.5,10});
  cg::Line2d horizontal_cut({5,2.5},{0,2.5});

  auto cut_polys1 = triangle.clip(cut_line1);
  auto cut_polys2 = triangle.clip(two_point_cut);
  auto cut_polys3 = triangle.clip(cut_through_vertex);
  auto cut_polys4 = triangle.clip(vertical_cut);
  auto cut_polys5 = triangle.clip(horizontal_cut);

}

TEST(Polygon, convex_hull){
  // Convex hull

}


