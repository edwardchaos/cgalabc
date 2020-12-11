#include <gtest/gtest.h>

#include <stdexcept>
#include "Polygon.h"

TEST(Polygon, create_polygon){
  try {
    // Create 2 valid polygons
    cg::Polygon poly{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    std::vector<cg::Point2d> pointset2{{-5, -5}, {5, -5}, {5, 5}};
    cg::Polygon poly2(pointset2);
  }catch(...){FAIL() << "No exception should be thrown.";}

  // Polygon with only 2 unique points
  try {
    cg::Polygon poly1_2points{{0, 0}, {0, 1}, {0, 0}};
  }catch(std::invalid_argument &e) {
    ASSERT_STREQ(e.what(),
                 "Cannot create a 2d polygon with 2 unique points, requires 3 "
                 "minimum");
  }catch(...){FAIL() << "Incorrect exception thrown";}

  try{
    cg::Polygon poly2_2points{{0,0},{0,1}};
  }catch(std::invalid_argument &e) {
    ASSERT_STREQ(e.what(),
               "Cannot create a 2d polygon with 2 unique points, requires 3 "
               "minimum");
  }catch(...) {FAIL() << "Incorrect exception thrown";}

  // Check Pairwise edges do not overlap
  try{cg::Polygon pg{{0,0},{1,0},{1,1},{1,0}};}
  catch(std::invalid_argument &e){
    ASSERT_STREQ(e.what(),
                 "Mon dieu! Cannot create 2d polygon; 2 edges are coincident.");
  }
  catch(...){FAIL() << "Incorrect exception thrown";}

  // Check pairwise edges do not intersect other than at connecting vertices
  try{cg::Polygon pg2{{0,0}, {1,0}, {1,1}, {0,-1}};}
  catch(std::invalid_argument &e){
    ASSERT_STREQ(e.what(), "Cannot create 2d polygon; 2 edges intersect.");
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


