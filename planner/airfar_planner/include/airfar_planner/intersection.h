#ifndef INTERSECTION_H
#define INTERSECTION_H

// A C++ program to check if two given line segments intersect
#include <iostream>
#include <opencv2/core.hpp>
using namespace std;
using namespace cv;

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'

namespace POLYOPS {

static bool onSegment(Point2f p, Point2f q, Point2f r)
{
	if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
		q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
	return true;

	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
static int orientation(Point2f p, Point2f q, Point2f r)
{
	// See https://www.geeksforgeeks.org/orientation-3-ordered-points/
	// for details of below formula.
	double val = (q.y - p.y) * (r.x - q.x) -
			     (q.x - p.x) * (r.y - q.y);

	if (abs(val) < 1e-7) return 0; // colinear

	return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
static bool doIntersect(Point2f p1, Point2f q1, Point2f p2, Point2f q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and q2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

static Point2f IntersectionwithLine(Point2f p1, Point2f p2, Point2f p3, Point2f p4) {
	// Line AB represented as a1x + b1y = c1
	double a1 = p2.y - p1.y;
	double b1 = p1.x - p2.x;
	double c1 = a1*(p1.x) + b1*(p1.y);
 
	// Line CD represented as a2x + b2y = c2
	double a2 = p4.y - p3.y;
	double b2 = p3.x - p4.x;
	double c2 = a2*(p3.x)+ b2*(p3.y);
 
	double determinant = a1*b2 - a2*b1;
 
	if (determinant == 0)
	{
		// The lines are parallel. This is simplified
		// by returning a pair of FLT_MAX
		return Point2f(FLT_MAX, FLT_MAX);
	}
	else
	{
		double x = (b2*c1 - b1*c2)/determinant;
		double y = (a1*c2 - a2*c1)/determinant;
		return Point2f(x, y);
	}
}

}

#endif
