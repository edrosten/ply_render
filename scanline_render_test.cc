#include "scanline_render.h"
#include "model_loader.h"

#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>
#include <cvd/camera.h>
#include <cvd/vector_image_ref.h>
#include <tag/printf.h>
#include <tag/stdpp.h>

#include <gvars3/instances.h>

using namespace CVD;
using namespace std;
using namespace TooN;
using namespace GVars3;

int main(int argc, char** argv)
{
	int last = GUI.parseArguments(argc, argv);
	Camera::Linear cam;
	ImageRef size(800, 600);


	cam.get_parameters().slice<0,2>() = Ones * 400;
	cam.get_parameters().slice<2,2>() = vec(size)/2;
	
	Model m(argv[last]);

//	SE3<> E = SE3<>::exp(makeVector(-.2,-.2,50,0,0,0));
	//E = E* SE3<>::exp(makeVector(0,0,0,.1,.5,.4));

	SE3<> E = SE3<>::exp(makeVector(-.5, -.63, 2, 0, 0, 0));
	E = E* SE3<>::exp(makeVector(0,0,0,.0,.0,.0));

	E = E* SE3<>::exp(makeVector(0,0,0,.1,.5,.4));
	E = E* SE3<>::exp(makeVector(0,0,0,.1,.5,.4));


//	vector<array<int,3>> triangles = m.get_edges();
//	vector<Vector<3>> cam3d;


//		cam3d.push_back(E * v);
	
	vector<Vector<3>> c3d;
	for(auto v:m.vertices)
		c3d.push_back(E*v);

	ScanlineRenderer renderer(m.get_edges());

	auto output = renderer.render(c3d, cam, size);

	VideoDisplay win(size, 1);
	cerr << "----------------------------------------------------------------------------------------\n";
	glClear(GL_COLOR_BUFFER_BIT);
	glBegin(GL_LINES);

	for(const auto& a: output)
	{
		//Vector<3> n = triangle_normals[a.triangle_index];

		//Do some primitive lighting

		double l=.3;
		//l += pow(max(0., unit(n) * unit(makeVector(1., -1.,-10))), 1)*.7;
		glColor3f(l,l,l);	

		glVertex(cam.project(project(a.start_cam3d)));
		glVertex(cam.project(project(a.end_cam3d)));
	}

	glEnd();

	glPointSize(1);
	glBegin(GL_POINTS);
	for(const auto& a: output)
	{
	/*
		if(a.start_edge < 0)
		{
			if(a.start_edge == BucketEntry::SimpleDeocclusion)
				glColor3f(0, 1, 0);
			else if(a.start_edge == BucketEntry::IntersectionDeocclusion)
			{
				glColor3f(1, 0, 0);
				glVertex(cam.project(project(a.start_cam3d)));
			}
			else
				glColor3f(0, 0, 1);
		}
	*/
		
		if(a.start_edge >= 0 && a.connects_on_left == false)
		{
			glColor3f(1, 0, 0);
			glVertex(cam.project(project(a.start_cam3d)) + Ones*.5);
		}
		if(a.end_edge >= 0 && a.connects_on_right == false)
		{
			glColor3f(0, 1, 0);
			glVertex(cam.project(project(a.end_cam3d)) + Ones*.5);
		}
	}
	glEnd();

	glFlush();
	cin.get();


}

