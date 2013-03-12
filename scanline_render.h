#ifndef RENDERER_INCLUDE_SCANLINE_RENDER_H
#define RENDERER_INCLUDE_SCANLINE_RENDER_H

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <cvd/camera.h>
#include <cvd/image_ref.h>
#include <vector>
#include <array>

struct OutputSegment
{
	TooN::Vector<3> start_cam3d, end_cam3d;
	TooN::Vector<2> start_cam2d, end_cam2d;
	int triangle_index;
	int start_edge;
	int end_edge;
	bool connects_on_left, connects_on_right;

	OutputSegment(const TooN::Vector<3>& s, const TooN::Vector<3>& e, int i, int se, int ee, int ll, int rr)
	:start_cam3d(s),
	 end_cam3d(e),
	 triangle_index(i),
	 start_edge(se),
	 end_edge(ee),
	 connects_on_left(ll),
	 connects_on_right(rr)
	{
	}
};

class ScanlineRendererImpl;

class ScanlineRenderer
{
	public:

		static const int SimpleOcclusion=-1;
		static const int IntersectionOcclusion=-2;
		static const int SimpleDeocclusion=-3;
		static const int IntersectionDeocclusion=-4;
		static const int Invalid=-99999;

		
		std::vector<OutputSegment> render(const std::vector<TooN::Vector<3>>& v, const Camera::Linear& cam, CVD::ImageRef size);
		~ScanlineRenderer();

		ScanlineRenderer(const std::vector<std::array<int,3>>& triangles);

		ScanlineRenderer(const ScanlineRenderer&)=delete;
		ScanlineRenderer& operator=(const ScanlineRenderer&)=delete;

	private:
		ScanlineRendererImpl* impl;	
};


#endif
