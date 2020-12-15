#include "gtest/gtest.h"

#include <cmath>
#include "type.h"

TEST(Testlines, create_lines){
  // Create line given 1 point. By default, if only 1 point is given, the
  // other point is at (0,0).
  cg::Point2d pt1(2,3);
  cg::Line2d l1(pt1);

  // Test line equation ax+by+c=0
  ASSERT_EQ(l1.a(), -3.0/2.0);
  ASSERT_EQ(l1.b(), 1);
  ASSERT_EQ(l1.c(), 0);

  // Create line given 2 points
  cg::Point2d pt2(-2, -1);
  cg::Point2d pt3(10, 6);
  cg::Line2d l2(pt2, pt3);

  ASSERT_TRUE(pt1==pt1);
  ASSERT_TRUE(pt1!=pt3);

  // Test line equation
  ASSERT_NEAR(l2.a(), -7.0/12.0, cg::EPS);
  ASSERT_NEAR(l2.b(), 1, cg::EPS);
  ASSERT_NEAR(l2.c(), -1.0/6.0, cg::EPS);

  // Try creating line with 2 identical points,
  try{
    cg::Line2d(pt1, pt1);
    FAIL() << "Shouldn't be able to create line with 2 identical points";
  }catch(std::invalid_argument &e){
    EXPECT_EQ(e.what(), std::string("Identical points used to create Line2d."));
  }catch(...){
    FAIL() << "Expected to catch invalid argument exception type";
  }

  // Create horizontal line with 1 point
  cg::Line2d h_line({1,0});

  // Check line equation
  ASSERT_EQ(h_line.a(), 0);
  ASSERT_EQ(h_line.b(), 1);
  ASSERT_EQ(h_line.c(), 0);

  // Create horizontal line with 2 points
  cg::Point2d pt4(-10,4);
  cg::Point2d pt5(4,4);
  cg::Line2d h_line2(pt4,pt5);

  // check line equation
  ASSERT_EQ(h_line2.a(), 0);
  ASSERT_EQ(h_line2.b(), 1);
  ASSERT_EQ(h_line2.c(), -4);

  // Create vertical line
  cg::Point2d pt6(-32.5, 8);
  cg::Point2d pt7(-32.5, -99);
  cg::Line2d v_line(pt6, pt7);

  // Check line equation
  ASSERT_EQ(v_line.a(), 1);
  ASSERT_EQ(v_line.b(), 0);
  ASSERT_EQ(v_line.c(), 32.5);

  // Distance between 2 points
  cg::Point2d distpt1(-7,-4);
  cg::Point2d distpt2(17,6.5);
  auto dist = distpt1.dist(distpt2);
  ASSERT_NEAR(dist, sqrt(686.25), cg::EPS);
  auto dist2 = pt1.dist(pt1);
  ASSERT_NEAR(dist2, 0, cg::EPS);
}

TEST(Testlines, basic_operations){
  // Compute line segment length
  cg::Line2d l1({0,1});
  cg::Line2d l2({1,1});
  cg::Line2d l3({1,0});

  ASSERT_NEAR(l1.length(), 1, cg::EPS);
  ASSERT_NEAR(l2.length(), sqrt(2), cg::EPS);
  ASSERT_NEAR(l3.length(), 1, cg::EPS);

  // Dot product tests
  ASSERT_NEAR(l1.dot(l3), 0, cg::EPS);
  ASSERT_NEAR(l1.dot(l2), 1, cg::EPS);
  ASSERT_NEAR(l1.dot(l1), 1, cg::EPS);

  // Cross product aka wedge product aka determinant in 2D
  ASSERT_NEAR(l1.cross(l2), -1, cg::EPS);
  ASSERT_NEAR(l2.cross(l1), 1, cg::EPS);
  ASSERT_NEAR(l1.cross(l1), 0, cg::EPS);

  cg::Line2d l4({4,0});
  auto l4_length1 = l4.normalize();

  ASSERT_NEAR(l4_length1.length(), 1, cg::EPS);
}

TEST(Line, normal){
  // TODO: Test midpoint

  // TODO: Test normal vector

}