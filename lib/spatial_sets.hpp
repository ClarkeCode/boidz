#ifndef SPATIAL_SETS
#define SPATIAL_SETS

#include "linalg.h"

namespace spatialsets {

	using point = linalg::aliases::float2;
	//Rects are in the format of (x, y, width, height)
	using rect = linalg::aliases::float4;

	point getCentreOfArea(rect r) {
		return point(r.x + (r.z / 2), r.y + (r.w / 2));
	}

	//Returns true if the point is within, or on the border of the provided rect, otherwise return false
	//Analagous to set identity. Negation results in set complement.
	bool isPointWithinArea(point p, rect r) {
		return p.x >= r.x && p.x <= (r.x + r.z) && p.y >= r.y && p.y <= (r.y + r.w);
	}

	//Return true if point is within the intersection of area A and area B, otherwise return false
	//Corresponds to logical AND
	bool isPointWithinIntersection(point p, rect a, rect b) {
		return isPointWithinArea(p, a) && isPointWithinArea(p, b);
	}

	//Return true if point is anywhere within area A or area B, otherwise return false
	//Corresponds to logical OR
	bool isPointWithinUnion(point p, rect a, rect b) {
		return isPointWithinArea(p, a) || isPointWithinArea(p, b);
	}

	//Return true if point is within area A or area B but not in intersection AB, otherwise return false
	//Corresponds to logical XOR
	bool isPointWithinSymmetricDifference(point p, rect a, rect b) {
		return isPointWithinUnion(p, a, b) && !isPointWithinIntersection(p, a, b);
	}

	//Return true if point is within area A but not within area B, otherwise return false
	bool isPointWithinDifference(point p, rect a, rect b) {
		return isPointWithinArea(p, a) && !isPointWithinArea(p, b);
	}
}

#endif