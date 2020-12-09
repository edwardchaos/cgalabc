#include "cgalabc.hpp"

#include <stdio.h>

namespace cg{
Point2d::Point2d(double x, double y) : x(x), y(y){}

void Point2d::print(){
  printf("Point: (%.3f, %.3f)\n", x, y);
}
} // namespace cg
