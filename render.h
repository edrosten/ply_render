#ifndef RENDER_INCLUDE_RENDER_H
#define RENDER_INCLUDE_RENDER_H

#include <memory>
#include <vector>
#include <TooN/se3.h>
#include "model_loader.h"


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

std::unique_ptr<Renderer> make_renderer_from_model(const Model& m)

std::vector<EdgeSegment> render(const TooN::SE3<>& E, const Model& m);

#endif
