#include "model_loader.h"
#include "static_vector.h"

#include <cassert>

#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>
#include <cvd/camera.h>
#include <cvd/vector_image_ref.h>

#include <gvars3/instances.h>

using namespace std;
using namespace CVD;
using namespace GVars3;
using namespace TooN;


// Each triangle is put into a bucket corresponding to the 
// image scanline it is on. The scanline (3D plane) intersection with the 
// triangle yields a line segment. 
//
// For bookkeeping purposes we will at some point need to know which triangle
// and which edge on that triangle the segment belongs to.
struct BucketEntry
{
	double start_x_img2d, end_x_img2d;
	Vector<3> start, end;
	int triangle_index;
	int start_edge_index, end_edge_index;
};

double line_plane_intersection(const Vector<4>& p_n, const Vector<3>& t, const Vector<3>& v)
{
	/* Plane:
	   [x 1] . [ p n ]= 0
	   and line:
	   x = t + a v

	  So:

      p. (t + a v) + n = 0

	  a = -(p.t + n) / (p.v)

	*/

	auto p = p_n.slice<0,3>();
	auto n = p_n[3];

	return - (p*t + n) / (p*v);
}



void draw_all(const vector<Vector<2>>& v, const vector<array<int, 3>>& t)
{

	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	for(auto i:t)
	{
		glVertex(v[i[0]]);
		glVertex(v[i[1]]);

		glVertex(v[i[1]]);
		glVertex(v[i[2]]);

		glVertex(v[i[2]]);
		glVertex(v[i[0]]);
	}
	glEnd();
}



int main(int argc, char** argv)
{
	int last = GUI.parseArguments(argc, argv);
	Camera::Linear cam;
	ImageRef size(80, 60);

VideoDisplay win(size, 10);


	cam.get_parameters().slice<0,2>() = Ones * 40;
	cam.get_parameters().slice<2,2>() = vec(size)/2;
	
	Model m(argv[last]);

	//SE3<> E = SE3<>::exp(makeVector(-.2,-.2,3,0,0,0));
	//E = E* SE3<>::exp(makeVector(0,0,0,.1,.5,.4));

	SE3<> E = SE3<>::exp(makeVector(-.5, -.64, 2, 0, 0, 0));
	E = E* SE3<>::exp(makeVector(0,0,0,.3,.5,.4));


	vector<array<int,3>> triangles = m.get_edges();
	vector<Vector<3>> cam3d;
	vector<Vector<2>> img2d;

	for(const auto&v:m.vertices)
	{
		cam3d.push_back(E * v);
		img2d.push_back(cam.project(project(cam3d.back())));
	}

	vector<vector<BucketEntry>> triangle_buckets(size.y);

	//Figure out which triangles should go into which rows
	for(unsigned int t=0; t < triangles.size(); t++)
	{

glClear(GL_COLOR_BUFFER_BIT);
draw_all(img2d, triangles);
glFlush();
cin.get();

glColor3f(.1, .1, .2);
glBegin(GL_LINES);
for(int yy=0; yy < size.y; yy++)
{
	glVertex2f(0, yy);
	glVertex2f(size.x, yy);
}
glEnd();
draw_all(img2d, triangles);
glFlush();
cin.get();


glColor3f(1, 1, 0);
glBegin(GL_LINE_LOOP);
glVertex(img2d[triangles[t][0]]);
glVertex(img2d[triangles[t][1]]);
glVertex(img2d[triangles[t][2]]);
glEnd();
glFlush();

		//Let the rows be centred on the top for now
		

		//Compute the span of the triangal vertically
		double min_y = 1e99, max_y = -1e99;
		for(auto j:triangles[t])
		{
			min_y = min(min_y, img2d[j][1]);
			max_y = max(max_y, img2d[j][1]);
		}

		//Include the triangle in a row if it touches the row centre.
		int min_y_ind = ceil(min_y);
		int max_y_ind = floor(max_y);

glBegin(GL_QUADS);
glVertex2f(0, min_y_ind-0.5);
glVertex2f(10, min_y_ind-0.5);
glVertex2f(10, max_y_ind+0.5);
glVertex2f(0, max_y_ind+0.5);
glEnd();
glFlush();
cin.get();





		for(int y_ind=min_y_ind; y_ind <= max_y_ind; y_ind++)
		{
			double y = y_ind; // +0.5?????

glColor3f(0, 1, 0);
glBegin(GL_LINES);
glVertex2f(0, y);
glVertex2f(size.x, y);
glEnd();
glFlush();


			//Compute plane corresponding to the line y

			// A plane: p.x + n=0
			// A 2D line:
			// [ x/z y/z 1] . [ l1 l2 l3 ] = 0
			// Therefore [x y z] [ l1 l2 l3] = 0
			// Therefore n=0 (clearly --  a plane through the origin projects to a line)
			// And the 2D line equation just uses p.
			//
			// The line is horizontal, so:
			// p1=0
			// y p2 + p3 = 0
			// y = -p3/p2
			//
			
			double y_cam2d = cam.unproject(makeVector(0, y))[1];

			Vector<4> p = unit(makeVector(0, -1, y_cam2d, 0));
cerr << "p = " << p << endl;
			//Compute the insersection of the linescan plane with the 
			//lines
			static_vector<pair<double, int>, 2> line_alphas;
			
			for(int j=0; j < 3 && line_alphas.size() < 3; j++)
			{
				double alpha = line_plane_intersection(p, cam3d[triangles[t][j]], cam3d[triangles[t][(j+1)%3]] - cam3d[triangles[t][j]]);

				cerr << alpha << endl;
				
				//If the intersection with the line happens within the triangle, then
				//record it.
				if(alpha >= 0 && alpha <= 1)
					line_alphas.push_back(make_pair(alpha, j));
			}

			//Some checks.
			cerr << line_alphas.size() << endl;
			assert(line_alphas.size() == 2);

			
				
			//Find the positions of the two intersections.
			array<Vector<3>,2> intersection;
			array<double,2> proj_x;

			for(int j=0; j < 2; j++)
			{
				int l = line_alphas[j].second;
				double a = line_alphas[j].first;


				intersection[j] = cam3d[triangles[t][l]] + a * (cam3d[triangles[t][(l+1)%3]] - cam3d[triangles[t][l]]);
				proj_x[j] = cam.project(project(intersection[j]))[0];

				assert(abs(cam.project(project(intersection[j]))[1] - y) < 1e-8);
			}
			
			//Sort them in X
			int first;
			if(proj_x[0] < proj_x[1])
				first = 0;
			else 
				first = 1;

			//Create the bucket entry.
			BucketEntry b;
			b.triangle_index = t;
			
			b.start = intersection[first];
			b.end   = intersection[!first];

			b.start_x_img2d = proj_x[first];
			b.end_x_img2d   = proj_x[!first];

			b.start_edge_index = line_alphas[first].second;
			b.end_edge_index   = line_alphas[!first].second;

			triangle_buckets[y_ind].push_back(b);
cerr << proj_x[first] << endl;
cerr << proj_x[!first] << endl;

glBegin(GL_LINES);
glColor3f(1, 0, 0);
glVertex2f(b.start_x_img2d, y);
glColor3f(1, 0, 1);
glVertex2f(b.end_x_img2d, y);
glEnd();
glFlush();
//cin.get();

cout << b.start << endl;
cout << b.end << endl;
cout << endl;


		}
	}

}
