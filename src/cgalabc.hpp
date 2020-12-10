#ifndef CGALABC_LIBRARY_H
#define CGALABC_LIBRARY_H

namespace cg{

const double EPS = 1e-5;

struct Point2d {
  double x, y;

  Point2d();
  Point2d(double x, double y);

  Point2d operator-(const Point2d &other) const;
  bool operator==(const Point2d &other) const;

  void print() const;
};

// Fundamentally, 2 points define a line.
class Line2d{
 public:
  Line2d();
  Line2d(Point2d v); // Vector from origin to point v
  Line2d(Point2d u, Point2d v); // Vector from point u to v

  double getA() const;
  double getB() const;
  double getC() const;

  // Euclidean length of this vector
  double length() const;

  // Cross product: This vec x other vec
  double cross(const Line2d &other) const;

  // Dot product
  double dot(const Line2d &other) const;

 private:
  // Line segment bounded by 2 points
  Point2d u,v;
  Point2d vec; // Vector independent of displacement (v-u)
  double a,b,c;

  void computeABC(const Point2d &pt1, const Point2d &pt2);
};

} // namespace cg

#endif //CGALABC_LIBRARY_H
