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






int main(int argc, char** argv)
{
	
	int last = GUI.parseArguments(argc, argv);
	Camera::Linear cam;
	ImageRef size(800, 600);

	cam.get_parameters().slice<0,2>() = Ones * 400;
	cam.get_parameters().slice<2,2>() = vec(size)/2;
	
	Model m(argv[last]);

	SE3<> E = SE3<>::exp(makeVector(-.2,-.2,3,0,0,0));
	E = E* SE3<>::exp(makeVector(0,0,0,.1,.5,.4));


	vector<array<int,3>> triangles = m.get_edges();
	vector<Vector<3>> cam3d;
	vector<Vector<2>> img2d;

	for(const auto&v:m.vertices)
	{
		cam3d.push_back(E * v);
		img2d.push_back(cam.project(project(cam3d.back())));
	}

	vector<vector<BucketEntry>> triangle_buckets;

	//Figure out which triangles should go into which rows
	for(unsigned int i=0; i < triangles.size(); i++)
	{
		//Let the rows be centred on the top for now
		

		//Compute the span of the triangal vertically
		double min_y = 1e99, max_y = -1e99;
		for(auto j:triangles[i])
		{
			min_y = min(min_y, img2d[j][1]);
			max_y = max(max_y, img2d[j][1]);
		}

		//Include the triangle in a row if it touches the row centre.
		int min_y_ind = floor(min_y);
		int max_y_ind = floor(max_y);


		for(int y_ind=min_y_ind; y_ind <= max_y_ind; y_ind++)
		{
			double y = y_ind; // +0.5?????

			//Compute plane
			Vector<4> p = TODO;

			//Compute the insersection of the linescan plane with the 
			//lines
			static_vector<pair<double, int>, 2> line_alphas;
			
			for(int j=0; j < 3 && line_alphas.size() < 3; j++)
			{
				double alpha = line_plane_intersection(p, cam3d[j], cam3d[(j+1)%3] - cam3d[j]);
				
				//If the intersection with the line happens within the triangle, then
				//record it.
				if(alpha >= 0 && alpha <= 1)
					line_alphas.push_back(make_pair(alpha, j));
			}

			//Some checks.
			assert(line_alphas.size() == 2);

			
				
			//Find the positions of the two intersections.
			array<Vector<3>,2> intersection;
			array<double,2> proj_x;

			for(int j=0; i < 2; i++)
			{
				int l = line_alphas[j].second;
				double a = line_alphas[j].first;

				intersection[j] = cam3d[l] + a * (cam3d[(l+1)%3] - cam3d[l]);
				proj_x[j] = cam.project(project(intersection[j]))[0];

				assert(abs(proj_x[j] - y) < 1e-8);
			}
			
			//Sort them in X
			int first;
			if(proj_x[0] < proj_x[1])
				first = 0;
			else 
				first = 1;

			//Create the bucket entry.
			BucketEntry b;
			b.triangle_index = i;
			
			b.start = intersection[first];
			b.end   = intersection[!first];

			b.start_x_img2d = proj_x[first];
			b.end_x_img2d   = proj_x[!first];

			b.start_edge_index = line_alphas[first].second;
			b.end_edge_index   = line_alphas[!first].second;

			triangle_buckets[y].push_back(b);
		}
	}

}
