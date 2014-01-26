/* Copyright (C) Computer Vision Consulting, 2013.*/
#include <TooN/TooN.h>

#include "model_loader.h"
#include "render.h"
#include <TooN/SymEigen.h>
#include <cvd/timer.h>
#include <cvd/camera.h>
#include <cvd/vector_image_ref.h>

using namespace TooN;
using namespace CVD;
using namespace std;


int main(int argc, char** argv)
{
	int last=argc-1;
	if(argc == 1)
	{
		cerr << "Please give PLY file as argument.\n";
		exit(1);
	}

	//Set up an image size and camera model.
	Camera::Linear cam;
	ImageRef size(800, 600);

	cam.get_parameters().slice<0,2>() = Ones * 400;
	cam.get_parameters().slice<2,2>() = vec(size)/2;
	
	//Load the specified PLY file.
	Model m(argv[last]);

	
	//Move the data to zero mean, unit variance just to make it easy for everything
	//to appear in the image..
	Vector<3> mean = Zeros;
	Matrix<3> cross = Zeros;
	for(auto v:m.vertices)
	{
		mean += v;
		cross += v.as_col() * v.as_row();
	}
	mean /= m.vertices.size();
	double scale = sqrt(SymEigen<3>(cross/m.vertices.size() - mean.as_col() * mean.as_row()).get_evalues()[0]);
	for(auto&& v:m.vertices)
		v = (v-mean)/scale;

	//Set a suitable position and rottion.
	SE3<> E = SE3<>::exp(makeVector(0.,0.,5.3,0,0,0));
	E = E* SE3<>::exp(makeVector(0,0,0,.1,.5,.4));
	

	//Transform all the vertices.
	vector<Vector<3>> c3d;
	for(auto v:m.vertices)
		c3d.push_back(E*v);

	vector<EdgeSegment> a = render(E, m);

	for(auto e:a)
	{
		cout << cam.project(e.a2d) << endl << cam.project(e.b2d) << endl << endl;
	}
}

