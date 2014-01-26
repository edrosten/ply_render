/* Copyright (C) Computer Vision Consulting, 2013.*/
#include <TooN/TooN.h>

#include "model_loader.h"
#include "render.h"
#include <cvd/timer.h>
#include <cvd/camera.h>
#include <cvd/vector_image_ref.h>

using namespace TooN;
using namespace CVD;
using namespace std;

int main(int, char** argv)
{
	Model m(argv[1]);
	
	ImageRef size(640,480);
	Camera::Linear cam;
	cam.get_parameters()[0] = 500;
	cam.get_parameters()[1] = 500;
	cam.get_parameters().slice<2,2>() = vec(size)/2;


	SE3<> E = SE3<>::exp(makeVector(-.2,-.2,3,0,0,0));
	E = E* SE3<>::exp(makeVector(0,0,0,.1,.5,.4));
	
	vector<EdgeSegment> a = render(E, m);

	for(auto e:a)
	{
		cout << cam.project(e.a2d) << endl << cam.project(e.b2d) << endl << endl;
	}
}

