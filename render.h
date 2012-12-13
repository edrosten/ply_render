#ifndef RENDER_INCLUDE_RENDER_H
#define RENDER_INCLUDE_RENDER_H

#include <TooN/se3.h>

struct EdgeSegment
{
	TooN::Vector<3> a3d, b3d;
	TooN::Vector<2> a2d, b2d;
};


std::vector<EdgeSegment> render(const TooN::SE3<>& E, const Model& m);

#endif
