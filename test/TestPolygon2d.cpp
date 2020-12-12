#include <gtest/gtest.h>

#include <stdexcept>
#include "Polygon.h"

TEST(Polygon, create_polygon){
  try {
    // Create 2 valid polygons
    cg::Polygon{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    std::vector<cg::Point2d> pointset2{{-5, -5}, {5, -5}, {5, 5}};
    cg::Polygon{pointset2};
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

  // Test convex
  ASSERT_TRUE(convex_poly.isConvex());
  ASSERT_FALSE(nonconvex_poly.isConvex());

  // Compute area

  // Contains point

  // clip polygon by line

  // Convex hull

}


