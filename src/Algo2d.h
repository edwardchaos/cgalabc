#pragma once

#include "type.h"

namespace cg{
bool isParallel(const Line2d& l1, const Line2d& l2);
bool isOrthogonal(const Line2d& l1, const Line2d& l2);
bool intersects(const Line2d& l1, const Line2d& l2);
Point2d intersectPoint(const Line2d& l1, const Line2d& l2);
};