#ifndef RENDER_INCLUDE_RENDER_H
#define RENDER_INCLUDE_RENDER_H

#include <TooN/se3.h>
#include <vector>

struct EdgeSegment
{
	TooN::Vector<3> a3d, b3d;
	TooN::Vector<2> a2d, b2d;
};

class Renderer
{
	public:
		virtual void set_vertex_world_coordinates(const std::vector<TooN::Vector<3>>& v)=0;
		virtual std::vector<EdgeSegment> render(const TooN::SE3<>& E)=0;
		virtual ~Renderer();

};

std::vector<EdgeSegment> render(const TooN::SE3<>& E, const Model& m);

#endif
