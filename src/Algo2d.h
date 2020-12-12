#pragma once

#include <memory>
#include "type.h"

namespace cg{
bool isParallel(const Line2d& l1, const Line2d& l2);
bool isOrthogonal(const Line2d& l1, const Line2d& l2);

/*
 * returns angle ABC between the lines AB and BC
 * Return 0 if points are collinear
 */

double angle(const Point2d &a, const Point2d &b, const Point2d &c);

/*
 * Returns true if line segments bounded by 2 points intersect. False if they
 * are parallel or they do not cross.
 */
bool intersects(const Line2d& l1, const Line2d& l2);
/*
 * Returns the point of intersection between 2 infinite length lines.
 * Returns nullptr if they are parallel.
 */
std::unique_ptr<Point2d> intersectPoint(const Line2d& l1, const Line2d& l2);

/*
 * Given a point and a line parameterized by ax + by + c = 0,
 * Returns the orthogonal line segment. If the length is 0, returns nullptr.
 */
std::unique_ptr<Line2d> orthogonalSegment(const Point2d &pt, const Line2d &l);

/* Shortest distance of point to line equation ax + by + c = 0
 * See also: distancePointToLineSegment
 */
double distancePointToLine(const Point2d &pt, const Line2d &l);

/* Shortest distance of point to line segment bounded by 2 points
 * See also: distancePointToLine
 */
double distancePointToLineSegment(const Point2d &pt, const Line2d &l);

} // namespace cg