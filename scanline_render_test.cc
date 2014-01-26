/* Copyright (C) Computer Vision Consulting, 2013.*/
#include "scanline_render.h"
#include "model_loader.h"

#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>
#include <cvd/camera.h>
#include <cvd/image_io.h>
#include <cvd/vector_image_ref.h>
#include <TooN/SymEigen.h>
#include <tag/printf.h>
#include <tag/stdpp.h>

using namespace CVD;
using namespace std;
using namespace TooN;

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

	//Instantiate the renderer (this does essentially nothing)
	//Note the rederer keeps a reference to the model edges!
	ScanlineRenderer renderer(m.get_edges());


	//Actually do the rendering
	auto output = renderer.render(c3d, cam, size);
	


	//Now draw the line segments into an image
	Image<Rgb<byte>> out(size, Rgb<byte>(0,0,0));

	//Draw the segments 
	for(const auto& a: output)
	{
		Vector<3> n = unit(c3d[m.U(a.triangle_index)] ^ c3d[m.V(a.triangle_index)]);
		if(n[2] > 0)
			n=-n;

		//Do some primitive lighting

		double l=.3;
		l += pow(max(0., unit(n) * unit(makeVector(1., -0.,-.10))), .3)*0.6;

		Rgb<byte> c = Rgb<byte>(l*255, l*255, l*255);

		int xs =  cam.project(project(a.start_cam3d))[0]+.5;
		int y  =  cam.project(project(a.start_cam3d))[1]+.5;
		int xe =  cam.project(project(a.end_cam3d))[0]+.5;

		for(; xs <= xe; xs++)
			if(out.in_image(ImageRef(xs, y)))
				out[y][xs] = c;
	}

	//Flag the edges caused by de/occlusion and model boundaries.
	//"Sharp" edges are not flagged here but could be done with 
	//By checking normals, or if model contains edges marked
	//as "draw always" or equivalent.
	for(const auto& a: output)
	{
		if(a.start_edge >= 0 && a.connects_on_left == false)
		{
			ImageRef p = ir(cam.project(project(a.start_cam3d)) + Ones*.5);
			if(out.in_image(p))
				out[p] = Rgb<byte>(255,0,0);
		}
		if(a.end_edge >= 0 && a.connects_on_right == false)
		{
			ImageRef p = ir(cam.project(project(a.start_cam3d)) + Ones*.5);
			if(out.in_image(p))
				out[p] = Rgb<byte>(0,255,0);
		}
	}
	img_save(out, cout, ImageType::PNG);
}

