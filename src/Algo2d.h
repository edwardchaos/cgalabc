#pragma once

#include <memory>
#include "type.h"

namespace cg{
bool isParallel(const Line2d& l1, const Line2d& l2);
bool isOrthogonal(const Line2d& l1, const Line2d& l2);
bool intersects(const Line2d& l1, const Line2d& l2);
/*
 * Returns the point of intersection between 2 line segments.
 * Returns nullptr if they do not intersect.
 */
std::unique_ptr<Point2d> intersectPoint(const Line2d& l1, const Line2d& l2);
};