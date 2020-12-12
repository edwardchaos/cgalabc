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
    ASSERT_STREQ(e.what(),
                 "Mon dieu! Cannot create polygon; 2 edges are coincident.");
  }
  catch(...){FAIL() << "Incorrect exception thrown";}

  // Pair of edges in polygon intersect
  try {
    cg::Polygon{{0, 0}, {1, 0}, {1, 1}, {0, -1}};
    FAIL() << "Should throw invalid argument";
  }
  catch(std::invalid_argument &e){
    ASSERT_STREQ(e.what(), "Cannot create polygon; 2 edges intersect.");
  }
  catch(...){FAIL() << "Incorrect exception thrown";}
}

TEST(Polygon, simple_algos){
  // Create Convex polygon

  // Create Concave polygon

  // Test convex

  // Compute area

  // Create point
  // Contains point

}


