#include "model_loader.h"
#include "static_vector.h"

#include <cassert>
#include <algorithm>

#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>
#include <cvd/camera.h>
#include <cvd/vector_image_ref.h>
#include <tag/printf.h>
#include <tag/stdpp.h>

#include <gvars3/instances.h>

using namespace std;
using namespace CVD;
using namespace GVars3;
using namespace TooN;
using namespace tag;


// Each triangle is put into a bucket corresponding to the 
// image scanline it is on. The scanline (3D plane) intersection with the 
// triangle yields a line segment. 
//
// For bookkeeping purposes we will at some point need to know which triangle
// and which edge on that triangle the segment belongs to.
struct BucketEntry
{
	//Some constants to represent various cases where an edge
	//index doesn't represent a real edge, but instead represents
	//a segment disappearing or appearing due to a change in 
	//occlusion status.

	static const int SimpleOcclusion=-1;
	static const int IntersectionOcclusion=-2;
	static const int SimpleDeocclusion=-3;
	static const int IntersectionDeocclusion=-4;
	static const int Invalid=-99999;

	double start_x_img2d, end_x_img2d;
	Vector<3> start, end;
	int triangle_index;
	int start_edge_index, end_edge_index;
};

pair<int, int> edge_vertices(const BucketEntry& b, bool start, const vector<array<int,3>>& triangles)
{
	int index;
	if(start)
		index = b.start_edge_index;
	else
		index = b.end_edge_index;
	
	int v1 = triangles[b.triangle_index][index];
	int v2 = triangles[b.triangle_index][(index+1)%3];
	
	if(v1 < v2)
		return make_pair(v1, v2);
	else
		return make_pair(v2, v1);
}


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



vector<vector<BucketEntry>> bucket_triangles_and_compute_segments(const vector<array<int,3>>& triangles, const vector<Vector<3>>& cam3d, const vector<Vector<2>>& img2d, ImageRef size, const Camera::Linear& cam)
{
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
	
	return triangle_buckets;
}

struct OutputSegment
{
	Vector<3> start_cam3d, end_cam3d;
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
	E = E* SE3<>::exp(makeVector(0,0,0,.0,.0,.0));

	E = E* SE3<>::exp(makeVector(0,0,0,.1,.5,.4));

	vector<OutputSegment> output;

	vector<array<int,3>> triangles = m.get_edges();
	vector<Vector<3>> cam3d;
	vector<Vector<2>> img2d;



	for(const auto&v:m.vertices)
	{
		cam3d.push_back(E * v);
		img2d.push_back(cam.project(project(cam3d.back())));
	}

	vector<vector<BucketEntry>> triangle_buckets = bucket_triangles_and_compute_segments(triangles, cam3d, img2d, size, cam);

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

	
	for(const auto& a: output)
	{
		if(a.triangle_index == 0)
			glColor3f(1, 1, 0);
		else
			glColor3f(0, 0, 1);
		
		glVertex(cam.project(project(a.start_cam3d)));
		glVertex(cam.project(project(a.end_cam3d)));
	}


	glEnd();
	
	glPointSize(3);
	glBegin(GL_POINTS);
	for(const auto& a: output)
	{
		if(a.start_edge == BucketEntry::SimpleOcclusion)
		{
			glColor3f(1, 0, 0);
			glVertex(cam.project(project(a.start_cam3d)));
		}

		if(a.start_edge == BucketEntry::IntersectionOcclusion)
		{
			glColor3f(0, 1, 0);
			glVertex(cam.project(project(a.start_cam3d)));
		}
		
		if(a.end_edge ==  BucketEntry::IntersectionOcclusion)
		{
			glColor3f(0, 1, 0);
			glVertex(cam.project(project(a.end_cam3d)));
		}
	}
	glEnd();

	glColor3f(1,0,0);
	
	for(int i=0 ; i < (int) cam3d.size(); i++)
	{
		glBegin(GL_LINES);
		glVertex(img2d[i]);
		glVertex(img2d[i] + makeVector(.5, -.5));
		glEnd();
		
		glPushMatrix();
		glTranslate(img2d[i]);
		glTranslatef(.5, -.5, 0);
		glScalef(1, -1, 0);
		
		glDrawText(sPrintf("%1.1f %i", cam3d[i][2], i));
		
		glPopMatrix();
	}

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
		
		static const double epsilon=1e-6;
		//Consider this:
		//segments within some epsilon compare as having equal x, and then are sorted
		//so that add==false comes before add==true.
		//
		sort(segment_vertices.begin(), segment_vertices.end(), [](const Vertex& a, const Vertex& b)
		{
			if(abs(a.x - b.x) < epsilon)
			{
				//a.x and b.x are like rilly close. So close, in fact that they are probably the same.
				//Let's call them equal here. So sort lexicographically by add/remove so that removal 
				//comes first

				if(a.add == false)
				{
					if(b.add == false)
						return false; //a not less than b
					else
						return true;
				}
				else //a.add true
				{
					return false; //b cannot be strictly less than a.
				}


			}
			else if(a.x < b.x)
				return true;
			else
				return false;
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

		
		const BucketEntry* last_segment = 0;
		int last_edge_index=BucketEntry::Invalid;
		Vector<3> last_output_cam3d = 1e99 * Ones;;

		for(unsigned int vertex_num=0; vertex_num < segment_vertices.size(); vertex_num++)
		{
			const Vertex& v = segment_vertices[vertex_num];
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

cerr << "Current line segment list:\n";
for(int i=0; i < active_segments.size(); i++)
{
	cerr << print << i << active_segments[i].z << active_segments[i].segment->triangle_index;
}

			
			
			//Check to see if the line in 0 has changed Z ordering with any other lines. If it has
			//then swap the new front line into position 1.
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

			int global_front = 0;
			for(int front=0; front< (int)active_segments.size()-1; front++)
			{
				bool swapped=0;
				Vector<3> leftmost_swap_pos = Ones * 1e99;
				const BucketEntry* leftmost_old_front_segment=0, * leftmost_new_front_segment = 0;

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

							leftmost_old_front_segment = active_segments[front].segment;
							leftmost_new_front_segment = active_segments[i].segment;

							swap(active_segments[front+1], active_segments[i]);
							global_front = front+1;
						}
					}
				}

				if(swapped)
				{

cerr << "New line segment list:\n";
for(int i=0; i < active_segments.size(); i++)
{
	cerr << print << i << active_segments[i].z << active_segments[i].segment->triangle_index;
}

					//Process the swapping and output a segment.

					Vector<3> out_start = last_output_cam3d;
					Vector<3> out_end = leftmost_swap_pos;

					int out_triangle = last_segment->triangle_index;
					assert(out_triangle == leftmost_old_front_segment->triangle_index);
					
					int out_start_edge_index = last_edge_index;
					int out_end_edge_index = BucketEntry::IntersectionOcclusion; //Segment ends on an intersection, not a real edge



					output.push_back(OutputSegment(
						out_start, 
						out_end,
						out_triangle,
						out_start_edge_index,
						out_end_edge_index));

assshit();
glBegin(GL_LINES);
glColor3f(1, 0, 0);
glVertex(cam.project(project(last_output_cam3d)));
glVertex(cam.project(project(leftmost_swap_pos)));
glEnd();
glFlush();
cin.get();

					last_output_cam3d = leftmost_swap_pos;
					last_edge_index = BucketEntry::IntersectionDeocclusion; //next segment starts on an intersection not a real edge
					last_segment = leftmost_new_front_segment;
				}
				else
				{
					//Make sure the list ends with the front most one at the head
					swap(active_segments[0], active_segments[front]);
					break;
				}
			}

			//Note also that no swap will be attempted here 
			//if active_segments.size() == 0.
			if(global_front != 0)
				swap(active_segments[global_front], active_segments[0]);

cerr << "Final line segment list:\n";
for(int i=0; i < active_segments.size(); i++)
{
	cerr << print << i << active_segments[i].z << active_segments[i].segment->triangle_index;
}

			//Fiiiinally deal with the vertex.

			if(v.add)
			{
				ActiveSegment new_seg;
				new_seg.z = v.segment->start[2];
				new_seg.segment = v.segment;

				if(active_segments.empty())
				{
					//If it's the only active segment, then just start a new segment here.
					last_output_cam3d = v.segment->start;
					last_segment = v.segment;
					last_edge_index = v.segment->start_edge_index;

					active_segments.push_back(new_seg);
				}
				else if(v.segment->start[2] > active_segments.front().z)
				{
					//If it's not at the front, then chuck it in somewhere.
					active_segments.push_back(new_seg);
				}
				else
				{
					//If the new segment is the foremost one, the output the old
					//segment since it becomes occluded here.
					Vector<3> back_seg_ends = line_plane_intersection_point(plane_of_vertical_x, active_segments.front().segment->start, active_segments.front().segment->end - active_segments.front().segment->start);




					Vector<3> out_start = last_output_cam3d;
					Vector<3> out_end = back_seg_ends;

					int out_triangle = last_segment->triangle_index;
					assert(out_triangle == active_segments.front().segment->triangle_index);
					
					int out_start_edge_index = last_edge_index;
					int out_end_edge_index = BucketEntry::SimpleOcclusion; //Segment ends on an occlusion, not a real edge

					output.push_back(OutputSegment(
						out_start, 
						out_end,
						out_triangle,
						out_start_edge_index,
						out_end_edge_index));

assshit();
glBegin(GL_LINES);
glColor3f(0, 0, 1);
glVertex(cam.project(project(last_output_cam3d)));
glVertex(cam.project(project(back_seg_ends)));
glEnd();
glFlush();
cin.get();
				
cout << "aaaaaaaaaaaaaaaaaaaadddddddddddddddddddd\n";

					active_segments.insert(active_segments.begin(), new_seg);

					//Search forwards in vertices to see if they are (a) added 
					//and (b) share a common edge.
					unsigned int vv = vertex_num+1;
					for(; vv < segment_vertices.size(); vv++)
						if(segment_vertices[vv].add == false ||  edge_vertices(*v.segment, true, triangles) != edge_vertices(*segment_vertices[vertex_num+1].segment, true, triangles))
							break;
					
					for(unsigned int vi=vertex_num+1; vi < vv; vi++)
					{
						new_seg.segment = segment_vertices[vi].segment;
						cerr << "Woah batman!!!\n";
					}

					
					//Create the new output segment, which starts on the real occluding edge.
					last_output_cam3d = v.segment->start;
					last_segment = v.segment;
					last_edge_index = v.segment->start_edge_index;
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
				//otherwise do nothing. 
				if(to_kill == active_segments.begin())
				{
if(active_segments.size() > 1)
{
cout << active_segments[0].z << endl;
cout << active_segments[1].z << endl;
}
					Vector<3> out_start = last_output_cam3d;
					Vector<3> out_end = to_kill->segment->end;

					int out_triangle = last_segment->triangle_index;
					assert(out_triangle == to_kill->segment->triangle_index);
					
					int out_start_edge_index = last_edge_index;
					int out_end_edge_index = to_kill->segment->triangle_index; //Segment ends on a real edge.

					output.push_back(OutputSegment(
						out_start, 
						out_end,
						out_triangle,
						out_start_edge_index,
						out_end_edge_index));


assshit();
glBegin(GL_LINES);
glColor3f(1, 0, 1);
glVertex(cam.project(project(last_output_cam3d)));
glVertex(cam.project(project(to_kill->segment->end)));
glEnd();
glFlush();
cin.get();
					
					bool erase_front=true;
					//Check to see if the next vertex comes from a 
					//shared edge and is the start of a segment. This indicates part
					//of some triangle strip.
					//

					//TODO can we replace this with a loop if multiple triangles 
					//share an edge?
					if(vertex_num+1 < segment_vertices.size())
					{
						//If the next vertex indicates an added segment then substitute it straight in
						//and do not pass go or collect $200.
						//
						//Otherewise if it's a removed segment with a shared edge, then remove it too.
						if(segment_vertices[vertex_num+1].add == true && edge_vertices(*v.segment, false, triangles) == edge_vertices(*segment_vertices[vertex_num+1].segment, true, triangles))
						{
							//Note Z doesn't change, and the sort order doesn't change, so we can skip the next 
							//iteration.
							active_segments.front().segment = segment_vertices[vertex_num+1].segment;

							last_output_cam3d = segment_vertices[vertex_num+1].segment->start;
							last_segment = segment_vertices[vertex_num+1].segment;
							last_edge_index = segment_vertices[vertex_num+1].segment->start_edge_index;
							
							vertex_num++;

							erase_front=false;
						}
						else if(segment_vertices[vertex_num+1].add == false && edge_vertices(*v.segment, false, triangles) == edge_vertices(*segment_vertices[vertex_num+1].segment, false, triangles))
						{
							active_segments.erase(find_if(active_segments.begin(), active_segments.end(), [&](const ActiveSegment& a)
							{
								return a.segment == segment_vertices[vertex_num+1].segment;
							}));
							vertex_num++;
						}
					}

					if(erase_front)
					{
						active_segments.erase(active_segments.begin());

						//Find who is in front, an put it at the front of the list.
						if(!active_segments.empty())
						{
							//First check if the frontmost active segment shares an edge
							//with the old (removed) active segment.
							auto m = min_element(active_segments.begin(), active_segments.end(), [](const ActiveSegment& a, const ActiveSegment& b)
							{
								return a.z < b.z;
							});

							swap(*active_segments.begin(), *m);

							last_output_cam3d = line_plane_intersection_point(plane_of_vertical_x, active_segments.front().segment->start, active_segments.front().segment->end - active_segments.front().segment->start);
							last_segment = active_segments.front().segment; 
							last_edge_index = BucketEntry::SimpleDeocclusion; //Segment starts on a disappearing occlusion
						}
						else
						{
							//There is no active segment now.
							last_segment = 0;
							last_edge_index=BucketEntry::Invalid;
							last_output_cam3d = Ones * 1e99;
						}
					}
				}
				else
					active_segments.erase(to_kill);
			}
		}


	}



}
