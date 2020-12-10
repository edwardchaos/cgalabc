#include "gtest/gtest.h"

#include "cgalabc.hpp"

TEST(Testlines, create_lines){
  // Create line given 1 point
  cg::Point2d pt1(2,3);
  cg::Line2d l1(pt1);

  // Test line equation
  ASSERT_EQ(l1.getA(), -3.0/2.0);
  ASSERT_EQ(l1.getB(), 1);
  ASSERT_EQ(l1.getC(), 0);

  // Create line given 2 points
  cg::Point2d pt2(-2, -1);
  cg::Point2d pt3(10, 6);
  cg::Line2d l2(pt2, pt3);
  // Test line equation
  ASSERT_NEAR(l2.getA(), -7.0/12.0, cg::EPS);
  ASSERT_NEAR(l2.getB(), 1, cg::EPS);
  ASSERT_NEAR(l2.getC(), -1.0/6.0, cg::EPS);

  // Try creating line with 2 identical points,
  try{
    cg::Line2d(pt1, pt1);
  }catch(std::invalid_argument &e){
    EXPECT_EQ(e.what(), std::string("Identical points used to create Line2d."));
  }catch(...){
    FAIL() << "Expected to catch an invalid argument exception but didn't.";
  }

  // Create horizontal line with 1 point
  cg::Line2d h_line({1,0});

  // Check line equation
  ASSERT_EQ(h_line.getA(), 0);
  ASSERT_EQ(h_line.getB(), 1);
  ASSERT_EQ(h_line.getC(), 0);

  // Create horizontal line with 2 points
  cg::Point2d pt4(-10,4);
  cg::Point2d pt5(4,4);
  cg::Line2d h_line2(pt4,pt5);

  // check line equation
  ASSERT_EQ(h_line2.getA(), 0);
  ASSERT_EQ(h_line2.getB(), 1);
  ASSERT_EQ(h_line2.getC(), -4);

  // Create vertical line
  cg::Point2d pt6(-32.5, 8);
  cg::Point2d pt7(-32.5, -99);
  cg::Line2d v_line(pt6, pt7);

  // Check line equation
  ASSERT_EQ(v_line.getA(), 1);
  ASSERT_EQ(v_line.getB(), 0);
  ASSERT_EQ(v_line.getC(), 32.5);
}