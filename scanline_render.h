#ifndef RENDERER_INCLUDE_SCANLINE_RENDER_H
#define RENDERER_INCLUDE_SCANLINE_RENDER_H

#include <TooN/TooN.h>

struct OutputSegment
{
	Vector<3> start_cam3d, end_cam3d;
	Vector<2> start_cam2d, end_cam2d;
	int triangle_index;
	int start_edge;
	int end_edge;

	OutputSegment(const Vector<3>& s, const Vector<3>& e, int i, int se, int ee)
	:start_cam3d(s),
	 end_cam3d(e),
	 triangle_index(i),
	 start_edge(se),
	 end_edge(ee)
	{
	}
};

class ScanlineRenderer
{
	public:

	static const int SimpleOcclusion=-1;
	static const int IntersectionOcclusion=-2;
	static const int SimpleDeocclusion=-3;
	static const int IntersectionDeocclusion=-4;
	static const int Invalid=-99999;




	public:
		virtual std::vector<OutputSegment> render(const TooN::SE3<>& E, const std::vector<TooN::Vector<3>>& v)=0;
		virtual ~Renderer();
};


#endif
