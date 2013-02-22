#include "model_loader.h"
#include "static_vector.h"

#include <cassert>
#include <algorithm>

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

Vector<3> line_plane_intersection_point(const Vector<4>& p_n, const Vector<3>& t, const Vector<3>& v)
{
	return t + line_plane_intersection(p_n, t, v)*v;
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

Vector<2> xz(const Vector<3>& v)
{
	return makeVector(v[0], v[2]);
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

		for(int y_ind=min_y_ind; y_ind <= max_y_ind; y_ind++)
		{
			double y = y_ind; // +0.5?????

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
			//Compute the insersection of the linescan plane with the 
			//lines
			static_vector<pair<double, int>, 2> line_alphas;
			
			for(int j=0; j < 3 && line_alphas.size() < 3; j++)
			{
				double alpha = line_plane_intersection(p, cam3d[triangles[t][j]], cam3d[triangles[t][(j+1)%3]] - cam3d[triangles[t][j]]);

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

			for(int j=0; j < 2; j++)
			{
				int l = line_alphas[j].second;
				double a = line_alphas[j].first;


				intersection[j] = cam3d[triangles[t][l]] + a * (cam3d[triangles[t][(l+1)%3]] - cam3d[triangles[t][l]]);
				proj_x[j] = cam.project(project(intersection[j]))[0];

				assert(abs(cam.project(project(intersection[j]))[1] - y) < 1e-8);
			}
			
			//Sort the end points in X
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

		}
	}

	//Now perform a left to right sweep, bubble sorting by Z to find crossings (?)
	for(unsigned int y_ind = 0; y_ind < triangle_buckets.size(); y_ind++)
	{
		//cout << "\n";

double y = y_ind;
auto assshit = [&]()
{
	glClear(GL_COLOR_BUFFER_BIT);
	draw_all(img2d, triangles);
	glColor3f(.1, .1, .2);
	glBegin(GL_LINES);
	for(int yy=0; yy < size.y; yy++)
	{
		glVertex2f(0, yy);
		glVertex2f(size.x, yy);
	}
	glEnd();
	draw_all(img2d, triangles);


	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex2f(0, y);
	glVertex2f(size.x, y);
	glEnd();
};

assshit();
glFlush();

cerr << "There are " << triangle_buckets[y_ind].size() << " triangles\n";

cin.get();

		//Split segments into vertices
		struct Vertex
		{
			const BucketEntry* segment;
			double x;
			bool add;
		};
	
		vector<Vertex> segment_vertices;
		for(const BucketEntry& b: triangle_buckets[y_ind])
		{

assshit();
glColor3f(1, 1, 0);
glBegin(GL_LINE_LOOP);
glVertex(img2d[triangles[b.triangle_index][0]]);
glVertex(img2d[triangles[b.triangle_index][1]]);
glVertex(img2d[triangles[b.triangle_index][2]]);
glEnd();
glBegin(GL_LINES);
			Vertex v;

			v.segment = &b;
			v.x = b.start_x_img2d;
			v.add = true;
			segment_vertices.push_back(v);

glColor3f(1, 0, 0);
glVertex2f(v.x, y);

			v.x = b.end_x_img2d;
			v.add = false;
			segment_vertices.push_back(v);

glColor3f(1, 0, 1);
glVertex2f(v.x, y);
glEnd();
glFlush();
cin.get();

		}
		
		//TODO
		//Consider this:
		//segments within some epsilon compare as having equal x, and then are sorted
		//so that add==false comes before add==true.
		//
		//This would ensure that when a start and end vertex are actually the same but
		//differ by numerical error it will always remove before adding, thereby insuring
		//that no epsilon sizes segments are put in, or er can claim on said insurance.
		sort(segment_vertices.begin(), segment_vertices.end(), [](const Vertex& a, const Vertex& b)
		{
			return a.x < b.x;
		});

assshit();
glFlush();
cin.get();

		struct ActiveSegment
		{
			double z;
			const BucketEntry* segment;
		};

		vector<ActiveSegment> active_segments;

		Vector<3> last_output_cam3d = 1e99 * Ones;;


		for(const auto& v:segment_vertices)
		{
			//Compute the current vertical sweep plane corresponding 
			//to the current vertex, to see if anything interesting has happened.

			double x_cam_2d = cam.unproject(makeVector(v.x, 0))[0];
			
			//p.x + n = 0 (plane).
			//If it projects to line, then n=0 and line is l.[x/z y/z 1] = 0
			//So, p = l
			Vector<4> plane_of_vertical_x = unit(makeVector(-1, 0, x_cam_2d, 0));

			//Recompute the depths
			for(auto& s:active_segments)
				s.z = line_plane_intersection_point(plane_of_vertical_x, s.segment->start, s.segment->end-s.segment->start)[2];

			
			
			//Check to see if the frontmost has swapped with any other lines
			//
			//If a swap has occured, then position[0] will be the old frontmost as before
			//and position 1 will contain the segment involved in the leftmost crossing.
			//That means position 1 is the new front.
			//
			//Then, repeat starting from position 1.
			//
			//Repeat until no swaps occur.
			//
			//This should generate a list of foremost line segments.
			//
			for(int front=0; front< (int)active_segments.size()-1; front++)
			{
				bool swapped=0;
				Vector<3> leftmost_swap_pos = Ones * 1e99;;
				for(int i=front+1; i < (int)active_segments.size(); i++)
				{
					if(active_segments[front].z > active_segments[i].z)
					{
						//A swap has occured, compute where.
						Vector<2> a = xz(active_segments[front].segment->start);
						Vector<2> b = xz(active_segments[front].segment->end) - a;

						Vector<2> c = xz(active_segments[i].segment->start);
						Vector<2> d = xz(active_segments[i].segment->end) - c;

						//Compute the intersecton point
						Matrix<2> m;
						m[0] = b;
						m[1] = -d;

						Vector<2> alpha_beta = inv(m.T()) * (c-a);



						Vector<3> pos = alpha_beta[0]*(active_segments[front].segment->end-active_segments[front].segment->start) + active_segments[front].segment->start;
						
						//If this is the leftmost crossing, then move the crossing segment
						//to a convenient position.
						//
						//Note this is the 3D x coordinate, not the 2D one, but since they
						//are all on the same line then they share the same ordering.
						if(!swapped || leftmost_swap_pos[0] < pos[0])
						{
alpha_beta=makeVector(0, 1);
cerr << "Starts:\n";
cerr << a << endl;
cerr << c << endl;
cerr << "Ends:\n";
cerr << a+b << endl;
cerr << c+d << endl;

cerr << "Intersect:\n";
cerr << alpha_beta[0] * b + a << endl;
cerr << alpha_beta[1] * d + c << endl;

cerr << alpha_beta << endl;

cerr << "Boom!\n";
							swapped=true;
							leftmost_swap_pos = pos;
							swap(active_segments[front+1], active_segments[i]);
						}
					}
				}

				if(swapped)
				{
					Vector<3> swap_pos_3d = leftmost_swap_pos;

					//output the segment
					cout << last_output_cam3d << endl;
					cout << swap_pos_3d << endl;
					cout << " " << endl;
assshit();
glBegin(GL_LINES);
glColor3f(1, 0, 0);
glVertex(cam.project(project(last_output_cam3d)));
glVertex(cam.project(project(swap_pos_3d)));
glEnd();
glFlush();
cin.get();

					last_output_cam3d = swap_pos_3d;
				}
				else
				{
					//Make sure the list ends with the front most one at the head
					swap(active_segments[0], active_segments[front]);
					break;
				}
			}

			//Fiiiinally deal with the vertex.

			if(v.add)
			{
				ActiveSegment new_seg;
				new_seg.z = v.segment->start[2];
				new_seg.segment = v.segment;

				//If it's not at the front, then chuck it in somewhere.
				if(active_segments.empty())
				{
					last_output_cam3d = v.segment->start;
					active_segments.push_back(new_seg);
				}
				else if(v.segment->start[2] > active_segments.front().z)
					active_segments.push_back(new_seg);
				else
				{
					Vector<3> back_seg_ends = line_plane_intersection_point(plane_of_vertical_x, active_segments.front().segment->start, active_segments.front().segment->end - active_segments.front().segment->start);

					cout << last_output_cam3d << endl;
					cout << back_seg_ends << endl;
					cout <<" " <<  endl;
assshit();
glBegin(GL_LINES);
glColor3f(0, 0, 1);
glVertex(cam.project(project(last_output_cam3d)));
glVertex(cam.project(project(back_seg_ends)));
glEnd();
glFlush();
cin.get();

					active_segments.insert(active_segments.begin(), new_seg);

					last_output_cam3d = v.segment->start;
				}
			}
			else
			{
				auto to_kill = find_if(active_segments.begin(), active_segments.end(), [&](const ActiveSegment& a)
				{
					return a.segment == v.segment;
				});
				assert(to_kill != active_segments.end());

				//If the one to be removed is at the front, then we need to so something special
				//otherwise do nothing. Note if the 
				if(to_kill == active_segments.begin())
				{
					
					//Output the segment.
					cout << last_output_cam3d << endl;
					cout << to_kill->segment->end << endl;
					cout << " " <<  endl;

assshit();
glBegin(GL_LINES);
glColor3f(1, 0, 1);
glVertex(cam.project(project(last_output_cam3d)));
glVertex(cam.project(project(to_kill->segment->end)));
glEnd();
glFlush();
cin.get();

					active_segments.erase(active_segments.begin());

					//Find who is in front, an put it at the front of the list.
					if(!active_segments.empty())
					{
						auto m = min_element(active_segments.begin(), active_segments.end(), [](const ActiveSegment& a, const ActiveSegment& b)
						{
							return a.z < b.z;
						});

						swap(*active_segments.begin(), *m);

						last_output_cam3d = line_plane_intersection_point(plane_of_vertical_x, active_segments.front().segment->start, active_segments.front().segment->end - active_segments.front().segment->start);
					}
					else
						last_output_cam3d = Ones * 1e88;
				}
				else
					active_segments.erase(to_kill);
			}
		}


	}



}
