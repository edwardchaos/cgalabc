#ifndef CGALABC_LIBRARY_H
#define CGALABC_LIBRARY_H

namespace cg{

struct Point2d {
  double x, y;

  Point2d() = default;
  Point2d(double x, double y);

  void print() const;
};

} // namespace cg

#endif //CGALABC_LIBRARY_H
