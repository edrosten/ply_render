#define DEBUG

#ifdef DEBUG
	#ifndef TOON_CHECK_BOUNDS
		#define TOON_CHECK_BOUNDS
	#endif
	#ifndef TOON_INITIALIZE_NAN
		#define TOON_INITIALIZE_NAN
	#endif
#endif

#include "model_loader.h"
#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>
#include <cvd/camera.h>
#include <cvd/vector_image_ref.h>
#include <algorithm>
#include <exception>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <iomanip>
#include <TooN/se3.h>
#include <TooN/SymEigen.h>
#include <tag/stdpp.h>
using namespace CVD;
using namespace std;
using namespace tag;
using namespace TooN;


vector<pair<int, Vector<2>>> proj(const vector<Vector<3>>& vertices, const SE3<>& E)
{
	vector<pair<int,Vector<2>>> ret;
	int i=0;
	for(auto& v:vertices)
		ret.push_back(make_pair(i++, project(E*v)));

	return ret;
}

template<class C> void unset(C& c)
{
	c=C();
}
template<class C*> void unset(C& c)
{
	c = reinterpret_cast<C>(0xbadc0ffee0ddf00d);
}





#ifdef DEBUG
template<class C, size_t Size> class Array: private array<C, Size>
{
	public:
		using array<C, Size>::size;
		using array<C, Size>::begin;
		using array<C, Size>::end;
		using array<C, Size>::value_type;

		C& operator[](size_t i)
		{
			assert(i < Size);
			return array<C, Size>::operator[](i);
		}	

		const C& operator[](size_t i) const
		{
			assert(i < Size);
			return array<C, Size>::operator[](i);
		}	


};

#else
	template<class C, size_t Size> using Array = std::array<C, Size>;
#endif

template<class C, size_t Max> class static_vector
{
	private:
		array<C, Max> data;
		size_t num;	
	public:
		static_vector()
		:num(0)
		{}

		static_vector(const static_vector&)=default;
		static_vector& operator=(const static_vector&)=default;

		void push_back(C c)
		{	
			if(num == data.size())
				throw std::length_error("static_vector");
			data[num++] = c;
		}

		size_t size() const
		{
			return num;
		}

		const C& operator[](size_t i) const
		{
			assert(i < num);
			return data[i];
		}

		const C* begin() const
		{
			return data.begin();
		}

		const C* end() const
		{
			return data.begin() + num;
		}

		#ifdef DEBUG
			~static_vector()
			{
				for(size_t i=0;i<Max; i++)
					unset(data[i]);
				num=0;
			}
		#endif
};


struct Vertex;
struct Edge;
struct Face;


//Edge structure: an edge consists of two vertices
//and a list of faces.
//
//Edges are stored left most vertex first
struct Edge
{
	Vertex *vertex1, *vertex2;
	static_vector<Face*, 2> faces;

	inline Edge(Vertex*v1, Vertex* v2);

	//y as a function of x, where x is the x position
	//of a specified vertex. The entire vertex is supplied so that
	//the y vaule is if v_x is the terminating vertex of the line.
	//
	//With some care, this means that sorting a list of edges by y value
	//will mean that:
	//1: all edges terminating at vertex corresponding to a
	//   sweep line will be contiguous after the sort. 
	//2: the correct y value can be found efficiently by binary search.
	//
	//Additionally, it allows for some good sanity checks.
	inline double y_at_x_of(const Vertex& v_x, bool debug_no_checks=false) const;

	//Special version with no error checking to allow use in
	//std::is_sorted
	inline double y_at_x_of_unchecked(const Vertex& v_x) const;



	#ifdef DEBUG
		~Edge()
		{
			unset(vertex1);
			unset(vertex2);
		}

	#endif
	
	private:
		inline bool a_is_on_left(const Vertex* a, const Vertex* b) const;

		Vertex* left(Vertex* a, Vertex* b) const
		{
			if(a_is_on_left(a, b))
				return a;
			else
				return b;
		}

		Vertex* right(Vertex* a, Vertex* b) const
		{
			if(a_is_on_left(a, b))
				return b;
			else
				return a;
		}
};


//Sort a pair of vertices using an arbitrary but stable method
pair<const Vertex*, const Vertex*> order(const Vertex* a, const Vertex* b)
{
	if(a<b)
		return make_pair(a, b);
	else
		return make_pair(b, a);
}




struct Vertex
{
	Vector<3> world;
	Vector<3> cam3d; //3d vertex position in camera coordinates
	Vector<2> cam2d; //2d veretx position in ideal camera coordinates
	Vector<2> pixel; //projected image coordinates
	int index; //Which vertex is this?

	vector<Edge*> left_edges; //List of edges to the left of the current point

	
	vector<Edge*> right_edges; //List of edges to the right of the current point
	                           //The list of right edges needs to be sorted top to
							   //bottom to minimize superfluous sorting at later points.
							   //Top is smallest y

	void sort()
	{
		std::sort(right_edges.begin(), right_edges.end(), 
			[&](Edge* a, Edge* b)
			{
				assert(a->vertex1 ==this);
				assert(b->vertex1 ==this);
				
				//Calculate the height of a unit triangle as:
				//h / 1 = dy / dx
				//And sort by h

				Vector<2> d_a = a->vertex2->cam2d - cam2d;
				double  h_a = d_a[1]/d_a[0];
				assert(d_a[0] != 0);

				Vector<2> d_b = b->vertex2->cam2d - cam2d;
				double  h_b = d_b[1]/d_b[0];
				assert(d_b[0] != 0);

				return h_a < h_b;
			}
		);

		std::sort(left_edges.begin(), left_edges.end(), 
			[&](Edge* a, Edge* b)
			{
				assert(a->vertex2 ==this);
				assert(b->vertex2 ==this);
				
				//Calculate the height of a unit triangle as:
				//h / 1 = -dy / dx
				//Note the reflection: dx is always <  0
				//so the unit triangle implies a reflection around x=0
				//so we need it to ben negated to represent the
				//-unit triangle.
				//And sort by h


				Vector<2> d_a = a->vertex1->cam2d - cam2d; 
				double  h_a = -d_a[1]/d_a[0];
				assert(d_a[0] != 0);

				Vector<2> d_b = b->vertex1->cam2d - cam2d;
				double  h_b = -d_b[1]/d_b[0];
				assert(d_b[0] != 0);

				return h_a < h_b;
			}
		);
	}

};

struct Face
{
	Array<Vertex*,3> vertices;
	static_vector<Edge*, 3> edges;
	Vector<4> plane;
	Vector<4> cam_plane;

	void compute_normals(const SE3<>& E)
	{

		//In the current implementation (above)
		//we only have triangles, so we could comput the normal as the
		//cross product of the vertices. However, computing the covariance about
		//a point is more general and will work if we ever switch to 
		//a model not just involving triangles.
		//
		//Any point will do (the centre is conventional), so we pick the
		//first point. This is equivalent including a model reflected about this
		//point.
		//
		//It incurs only a linear cost at model build time.
		Matrix<3> M = Zeros;

		for(unsigned int i=1; i < vertices.size(); i++)
		{
			Vector<3> v = vertices[i]->world - vertices[0]->world;
			M += v.as_col() * v.as_row();
		}
		
		SymEigen<3> eig(M);
		
		//points on a plane p, obey (p - p0) . n = 0
		//
		//or, in homogenous coordinates
		//
		// (px px pz ps) * ( n1 n2 n3  -p0.n) = 0
		//
		plane.slice<0,3>() = eig.get_evectors()[0];
		plane[3] = -vertices[0]->world * eig.get_evectors()[0];
		
		//The camera plane goes by the inverse because
		//
		// A plane is defined (in homogeneous coordinates) as x.p = 0
		// x is transformed by E, so what happens to p?
		// (Ex).(Tp) = 0
		// (xE)^t (Tp) = 0
		// T = inv(E^t)
		cam_plane = plane * E.inverse();
	}

	double depth(const Vector<2>& cam2d) const
	{
		//Compute intersection
		//
		//points on a plane p, obey (p - p0) . n = 0
		//A line is defined by l = e + lambda f
		//Let l=p and solve gives:
		//
		//lambda = (p0 - e).n / (f * n) = (p0.n - e.n) / f.n

		//The ray through the camera goes through the origin, 
		//so e=0, giving 
		//
		//lambda = p0.n / f.n
		//
		//From compute_normals() above, the plane equation is stored as:
		//cam_plane = (n1 n2 n3 -p0.n)
		//
		//so:
		return -cam_plane[3] / (unproject(cam2d) * cam_plane.slice<0,3>());
	}
};

inline Edge::Edge(Vertex*v1, Vertex* v2)
:vertex1(left(v1, v2)),
 vertex2(right(v1, v2))
{
}

inline double Edge::y_at_x_of(const Vertex& v_x, bool debug_no_checks) const
{
	//In regular code, this function should never be used to at vertex1
	//In debugging tests, it might be.


	//debug_no_checks 
	//We should never be interpolating the y value if we're at
	//the start vertex.
	if(!debug_no_checks)
		assert(&v_x != vertex1);
	

	//Special case for the end vertex to ensure 100% accuracy.
	//necessary for using equal_range etc over y values.
	if(&v_x == vertex2)
		return vertex2->cam2d[1];
	else
	{
		double x = v_x.cam2d[0];	
		//Check that we're never extrapolating, and never
		//actually at one of our vertices by this point.
		if(!debug_no_checks)
			assert(x > vertex1->cam2d[0]);
		assert(x >= vertex1->cam2d[0]);
		assert(x < vertex2->cam2d[0]);


		double Dy = vertex2->cam2d[1]-vertex1->cam2d[1];
		double Dx = vertex2->cam2d[0]-vertex1->cam2d[0];
		double dx = x - vertex1->cam2d[0];

		assert(dx >= 0);
		assert(Dx > 0);

		return dx * Dy / Dx + vertex1->cam2d[1];
	}
}

inline double Edge::y_at_x_of_unchecked(const Vertex& v_x) const
{
	return y_at_x_of(v_x, true);
}

inline bool Edge::a_is_on_left(const Vertex* a, const Vertex* b) const
{
	assert(a->cam2d[0] != b->cam2d[0]);
	return a->cam2d[0] < b->cam2d[0];
}


struct Intersection
{
	Vector<2> cam2d;
	Vector<3> front_pos;
	Vector<3> back_pos;
	const Edge* front_edge, *back_edge;
};

struct EdgeSegment
{
	Vector<3> a3d, b3d;
	Vector<2> a2d, b2d;
};

enum class Visibility
{
	Visible, 
	Hidden, 
	MaybeVisible
};

struct ActiveEdge
{
	Edge* edge;
	Visibility previous;
	unordered_set<Face*> occluding_faces;
	int occlusion_depth;
};

void debug_draw_all(const Model& m, const Camera::Linear& cam, const SE3<>& E)
{
	double minz=1e99, maxz=-1e99;

	for(auto v:m.vertices)
	{
		double z = (E*v)[2];
		minz = min(minz, z);
		maxz = max(maxz, z);
	}


	auto vert = [&](const Vector<3>& x){
		glColor3f(0, 0, (1-(x[2]-minz)/(maxz-minz))*0.6 + 0.1);
		glVertex(cam.project(project(x)));
	};

	glBegin(GL_LINES);
	for(size_t i=0; i < m.get_edges().size(); i++)
	{
		Vector<3> EA = E*m.A_pos(i);
		Vector<3> EU = E*m.U_pos(i);
		Vector<3> EV = E*m.V_pos(i);
		
		vert(EA);
		vert(EU);

		vert(EA);
		vert(EV);

		vert(EU);
		vert(EV);
	}
	glEnd();
}


vector<Vertex> get_sorted_list_of_camera_vertices_without_edges(const Camera::Linear& cam, const SE3<>& E, const vector<Vector<3>>& model_vertices)
{
	const double x_delta=1e-6;

	//Rotate and project model into camera coordinates.
	//From now on, we will only work with camera centred
	//coordinates.
	vector<Vertex> vertices(model_vertices.size());
	for(size_t i=0; i < vertices.size(); i++)
	{
		vertices[i].world = model_vertices[i];
		vertices[i].cam3d = E*model_vertices[i];
		vertices[i].cam2d = project(vertices[i].cam3d);
		vertices[i].pixel = cam.project(vertices[i].cam2d);
		vertices[i].index = i;
	}
	
	//Sort the vertices left to right
	sort(vertices.begin(), vertices.end(), 
		[](const Vertex& v1, const Vertex& v2)
		{
			return v1.cam2d[0]  < v2.cam2d[0];
		}
	);

	//Now, some vertices might share the same x coordinate. 
	//Fix this, with an evil hack. Note that this deals
	//with multiple vertices sharing the same x coordinate.
	double x_prev = vertices[0].cam2d[0];
	for(size_t i=1; i < vertices.size(); i++)
	{
		if(vertices[i].cam2d[0] == x_prev)
			vertices[i].cam2d[0] = vertices[i-1].cam2d[0] + x_delta;
		else
			x_prev = vertices[i].cam2d[0];
	}

	return vertices;
}

int main()
{
	Model m("cube.ply");
	ImageRef size(640, 480);


	Camera::Linear cam;
	cam.get_parameters()[0] = 500;
	cam.get_parameters()[1] = 500;
	cam.get_parameters().slice<2,2>() = vec(size)/2;

	VideoDisplay d(size);



	SE3<> E = SE3<>::exp(makeVector(-.5,-.5,4,0,0,0));

	E = E* SE3<>::exp(makeVector(0,0,0,.1,.5,.4));
	cout << E << endl;
	
	glClear(GL_COLOR_BUFFER_BIT);
	debug_draw_all(m, cam, E);
	glFlush();
	cin.get();
	
	
	vector<Vertex> vertices = get_sorted_list_of_camera_vertices_without_edges(cam, E, m.vertices);

	//The vertices are now shuffled, so in order to refer to a particular vertex, 
	//we need a mapping:
	vector<Vertex*> index_to_vertex(vertices.size());
	for(auto& v:vertices)
	{
		index_to_vertex[v.index] = &v;
		cout << setprecision(15) << v.cam2d[0] << endl;
	}

	
	vector<Edge> edges;
	vector<Face> faces(m.get_edges().size());
	{
		//Get all the unique edges and faces
		//Note that the model is very badly named.
		//get_edges(), actually gets the list of *faces*
		//~yay~
		map<pair<const Vertex*, const Vertex*>, Edge> s_edges;
		
		for(unsigned int i=0; i < m.get_edges().size(); i++)
		{
			
			Face* face = &faces[i];
			//Fill in the list of vertices which each face has
			for(unsigned int j=0; j < m.get_edges()[i].size(); j++)
				face->vertices[j] = index_to_vertex[m.get_edges()[i][j]];


			//Now get the closed loop of edges
			for(unsigned int j=0; j < m.get_edges()[i].size(); j++)
			{

				Vertex* v1 = face->vertices[j];
				Vertex* v2 = face->vertices[(j+1) % face->vertices.size()];

				decltype(s_edges)::iterator edge;
				bool b;

				//Find the edge/insert it if it does not exist.
				tie(edge, b) = s_edges.insert(make_pair(order(v1, v2), Edge(v1, v2)));
				

				//Either way, associate the face with the edge
				edge->second.faces.push_back(face);
			}
		}
		
		//Pack the edges into a std::vector
		for(auto& e:s_edges)
			edges.push_back(e.second);
	}

	//Now go and propagate the edge information to the faces and vertices
	for(auto& edge: edges)
	{
		for(unsigned int i=0; i < edge.faces.size(); i++)
			edge.faces[i]->edges.push_back(&edge);
		
		
		//vertex1 is to the left of vertex2
		edge.vertex1->right_edges.push_back(&edge);
		edge.vertex2->left_edges.push_back(&edge);
	}

	//Now do the final sorting on the right hand edges of each vertex.
	for(auto& v:vertices)
		v.sort();

	//Compute the face normals
	for(auto& f:faces)
		f.compute_normals(E);

	auto cross=[](const Vector<2>& v)
	{
		glColor3f(1,0,0);
		int size=3;
		glVertex(v+makeVector( size, 0));
		glVertex(v+makeVector(-size, 0));
		glVertex(v+makeVector(0,  size));
		glVertex(v+makeVector(0, -size));
	};

	//At this point we have a sorted list of vertices (left to right), 
	//and faces, vertices and edges with all cross referencing
	//information.
	vector<Edge*> active_edges;
	
	for(const auto& v: vertices)
	{

cout << "Hello\n\n";

		auto debug_order_at_v = [&](const Edge* e1, const Edge* e2)
		{
			return e1->y_at_x_of_unchecked(v) < e2->y_at_x_of_unchecked(v);
		};

		//Horizontal position of the sweep line
		double x = v.cam2d[0];

		glClear(GL_COLOR_BUFFER_BIT);
		debug_draw_all(m, cam, E);
		glBegin(GL_LINES);

		glColor3f(.5, 0, 0);
		glVertex2f(v.pixel[0], 0);
		glVertex2f(v.pixel[0], 480);

		for(auto e:v.left_edges)
		{
			glColor3f(1, 1, 0);
			glVertex(e->vertex1->pixel);
			glColor3f(0, 1, 0);
			glVertex(e->vertex2->pixel);
		}

		for(auto e:v.right_edges)
		{
			glColor3f(0, 1, 0);
			glVertex(e->vertex1->pixel);
			glColor3f(0, 1, 1);
			glVertex(e->vertex2->pixel);
		}


		//Find the crossings by re-sorting.
		vector<pair<Edge*, Edge*>> crossings;

		//Sort edges top to bottom according to the intersection with the sweep
		//line using bubble sort since each exchange corresponds to a crossing.
		//Thanks, Tom!!!
		for(int n=active_edges.size(); ;)
		{
			bool swapped=false;
			int new_n=0;
			
			for(int i=1; i < n; i++)
			{
				if(active_edges[i-1]->y_at_x_of(v) > active_edges[i]->y_at_x_of(v))
				{
					swap(active_edges[i-1], active_edges[i]);
					crossings.push_back(make_pair(active_edges[i-1], active_edges[i]));
					swapped=true;
					new_n=i;

				}
			}

			if(!swapped)
				break;
			n = new_n;
		}

		assert(is_sorted(active_edges.begin(), active_edges.end(), debug_order_at_v));

		//Some sanity checks: make sure that ever edge terminating at the current vertex is
		//active.
		for(auto e:v.left_edges)
			assert(find(active_edges.begin(), active_edges.end(), e) != active_edges.end());
		


		//Now remove all left edges from active_edges
		//
		//Note that this is O(Num_of_active_edges). IF active_edges was a list
		//and IF vertex kept a record of iterators of its left edges in active_edges
		//then we could just remove the individual edges in O(Num_left_edges) time
		//instead of O(Num_of_active_edges). 
		//
		//However, the above bubble sort is at best O(Num_of_active_edges)
		//anyway, so we could never reduce the order of this section. Since using a list
		//would worsen the constant greatly, using a list would almost certainly
		active_edges.erase(
			remove_if(active_edges.begin(), active_edges.end(), 
				[&](const Edge* e)
				{
					return e->vertex2 == &v;
				}),
			active_edges.end());
	
		//Perform a vertical walk downwards along edges to see which faces come and go
		//as the walk is performed, until we hit the current vertex.
		//
		//Possible TODO: one could perform a walk upwards or downwards, depending on
		//how close to the top or bottom the current vertex is, for a factor of 2 saving.
		double vertex_y = v.cam2d[1];
		unordered_set<const Face*> faces_active;
		for(const auto& e:active_edges)
		{
			
			if(e->y_at_x_of(v) > vertex_y)
				break;

			for(auto& f:e->faces)
			{
				if(faces_active.count(f))
					faces_active.erase(f);
				else
					faces_active.insert(f);
			}
		}

cout << faces_active.size() << endl;

		//Since we're at a vertex, we may have previously added faces 
		//associted with this vertex. If so, then there must be both a left
		//and right edge associated with the face at this vertex.
		//
		//If a face is active and associated with this vertex, then we
		//have no remaining active edges associated with the face. So, we
		//need to explicitly remove all faces associated with this vertex.
		//
		//This is because faces associated with the vertex cannot occlude the
		//vertex.
		for(auto e:v.left_edges)
			for(auto f:e->faces)
				faces_active.erase(f);

		//Now, we need to check the vertex against all remaining active planes to 
		//see if it is occluded.


		int occlusion_depth=0;

		cout << "Num active faces: " << faces_active.size() << endl;
		unordered_set<Face*> occluders;
		for(auto f: faces_active)
		{
			for(const auto& fv:f->vertices)
				assert(fv != &v);
			
			double depth = f->depth(v.cam2d);

			cout << depth << "          " << v.cam3d << endl;
			cout << f->plane << endl;
			cout << f->cam_plane << endl;

			if(depth < v.cam3d[2])
				occlusion_depth++;
		}
		
cout << "Hidden = " << occlusion_depth << endl;

		//Some sanity checks: no left edges should remain active.
		for(auto e:v.left_edges)
			assert(find(active_edges.begin(), active_edges.end(), e) == active_edges.end());

		for(auto e:active_edges)
		{
			glColor3f(1, 0, 1);
			glVertex(e->vertex1->pixel);
			glVertex(e->vertex2->pixel);
		}

		//Edges are sorted top to bottom by intersection with the 
		//sweep line. Find the position to insert the new edges.
		//
		//upper bound returns a pointer to the first element greater
		//than the y coordinate. vector::insert will insert just before
		//this iterator.
		auto here = upper_bound(active_edges.begin(), active_edges.end(), v.cam2d[1],
								[&](double y, Edge* e)
								{
									return y < e->y_at_x_of(v);
								}
					);
		active_edges.insert(here, v.right_edges.begin(), v.right_edges.end());

		assert(is_sorted(active_edges.begin(), active_edges.end(), debug_order_at_v));
		
		












		glBegin(GL_LINES);
		//Lolhack;
		int n = &v - &*vertices.begin();
		if(n > 0)
		{
			glColor3f(.5, 0, 0);
			glVertex2f(vertices[n-1].pixel[0], 0);
			glVertex2f(vertices[n-1].pixel[0], 480);
		}

		glEnd();
		glFlush();
		cin.get();


		for(auto c: crossings)
		{
			glBegin(GL_LINES);
			glColor3f(1, 0, 1);
			for(auto c:crossings)
			{
				glVertex(c.first->vertex1->pixel);
				glVertex(c.first->vertex2->pixel);
				glVertex(c.second->vertex1->pixel);
				glVertex(c.second->vertex2->pixel);
			}

			glColor3f(1, 1, 1);
			glVertex(c.first->vertex1->pixel);
			glVertex(c.first->vertex2->pixel);
			glVertex(c.second->vertex1->pixel);
			glVertex(c.second->vertex2->pixel);


			glEnd();
			glFlush();
			cin.get();

		}
		cross(v.cam2d);

	}

}
