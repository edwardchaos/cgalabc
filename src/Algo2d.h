#pragma once

#include <memory>

#include "type.h"
#include "Polygon.h"

namespace cg{
bool isParallel(const Line2d& l1, const Line2d& l2);
bool isOrthogonal(const Line2d& l1, const Line2d& l2);

/*
 * returns angle ABC between the lines AB and BC
 * Output ranges [0,PI]
 */
double angle(const Point2d &a, const Point2d &b, const Point2d &c);

/*
 * Returns true if line segments bounded by 2 points intersect.
 */
bool intersects(const Line2d& l1, const Line2d& l2);

/*
 * Returns the point of intersection between 2 infinite length lines.
 * Returns nullptr if they are parallel.
 */
Point2d_ptr intersectPoint(const Line2d& l1, const Line2d& l2);

/*
 * Given a point and a line parameterized by ax + by + c = 0,
 * Returns the orthogonal line segment from the point to the line. If the
 * length is 0, returns nullptr.
 */
Line2d_ptr orthogonalSegment(const Point2d &pt, const Line2d &l);

/* Shortest distance of point to line equation ax + by + c = 0
 * See also: distancePointToLineSegment
 */
double distancePointToLine(const Point2d &pt, const Line2d &l);

/* Shortest distance of point to line segment bounded by 2 points
 * See also: distancePointToLine
 */
double distancePointToLineSegment(const Point2d &pt, const Line2d &l);

/*
 * Convex hull of a point set.
 * Returns nullptr if a convex polygon cannot be formed from the input set of
 * points
 */
Polygon_ptr convexHull(const std::vector<Point2d> &pts);

/*
 * Linear interpolation between 2 points.
 * Argument t in range [0,1]
 */
Point2d lerp(const Point2d& from, const Point2d& to, double t);

} // namespace cg