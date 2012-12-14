#define DEBUG
#define DEBUG
#undef DEBUG

#ifdef DEBUG
	#ifndef TOON_CHECK_BOUNDS
		#define TOON_CHECK_BOUNDS
	#endif
	#ifndef TOON_INITIALIZE_NAN
		#define TOON_INITIALIZE_NAN
	#endif
#else
	#define  NDEBUG
#endif

#define F(X) 

#include <cassert>

#include "model_loader.h"
#include <cvd/timer.h>
#include <algorithm>
#include <exception>
#include <map>
#include <set>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <iomanip>
#include <TooN/se3.h>
#include <TooN/SymEigen.h>
#include <tag/stdpp.h>


#include "debug_helpers.h"
#include "render.h"
#include "pointer_set.h"
#include "debug_array.h"
#include "static_vector.h"

using namespace CVD;
using namespace std;
using namespace tag;
using namespace TooN;


struct Vertex;
struct Face;


//Edge structure: an edge consists of two vertices
//and a list of faces.
//
//Edges are stored left most vertex first
//
//Somewhat astohishingly, for large, complex models the repeated computation
//of y_at_x_of() is the slowest part. So, we put a lot of effort into making that
//quicker by caching values.
//
//This structure defers adding and removing occluders to a derives class
//in order to allow us to keep track of every occluding face or merely
//a count of the occluders.
struct EdgeWithNoOcclusion
{
	//This is purely static information which never changes
	Vertex *vertex1, *vertex2;
	static_vector<Face*, 2> faces;
	
	
	//This is semi-dynamic information which changes each time
	//a new transformation is used
	double gradient;
	Vector<2> v1cam2d;
	bool should_render;

	//This is dynamic information which is in use
	//only when the edge is active. It ought to belong to
	//active edge, but that bloats the ActiveEdge structure.
	//
	//Since ActiveEdge is hammered and has many
	//deletes and inserts, minimizing its size improves the speed
	//by a factor of 1.3.
	//
	//Sad that the cleaner design is slower, but oh well.
	int occlusion_depth;
	Vector<3> previous_3d;
	Vector<2> previous_2d;
	static_vector<Face*, 2> faces_above, faces_below;

	inline EdgeWithNoOcclusion(Vertex*v1, Vertex* v2);

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
		~EdgeWithNoOcclusion()
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


struct EdgeWithOcclusionDepthOnly: public EdgeWithNoOcclusion
{
	inline EdgeWithOcclusionDepthOnly(Vertex*v1, Vertex* v2)
	:EdgeWithNoOcclusion(v1, v2)
	{
	}

	void add_occluding_face(const Face*)
	{
		occlusion_depth++;
	}

	void remove_occluding_face(const Face*)
	{
		occlusion_depth = max(0, occlusion_depth-1);
	}
	
	void set_occluders(const unordered_set<const Face*>&, int depth)
	{
		occlusion_depth = depth;
	}
};


struct EdgeWithFullOcclusion: public EdgeWithNoOcclusion
{
	unordered_set<const Face*> occluding_faces;

	inline EdgeWithFullOcclusion(Vertex*v1, Vertex* v2)
	:EdgeWithNoOcclusion(v1, v2)
	{
	}

	void add_occluding_face(const Face* f)
	{
		occluding_faces.insert(f);
		occlusion_depth++;
	}

	void remove_occluding_face(const Face* f)
	{
		if(occluding_faces.erase(f))
			occlusion_depth--;
	}
	
	void set_occluders(unordered_set<const Face*>& faces, int depth)
	{
		occluding_faces = faces;
		occlusion_depth = depth;
	}
};



typedef EdgeWithOcclusionDepthOnly Edge;





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
	int index; //Which vertex is this?

	vector<Edge*> left_edges; //List of edges to the left of the current point

	
	vector<Edge*> right_edges; //List of edges to the right of the current point
	                           //The list of right edges needs to be sorted top to
							   //bottom to minimize superfluous sorting at later points.
							   //Top is smallest y

	unordered_set<const Face*> faces;

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
	double max_y;

	void compute_data(const SE3<>& E)
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

		//Find the lowest point of the face
		max_y = (*max_element(vertices.begin(), vertices.end(),
		            [](const Vertex* v1, const Vertex* v2)
					{
						return v1->cam2d[1] < v2->cam2d[1];
					}))->cam2d[1];
		
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
		//From compute_data() above, the plane equation is stored as:
		//cam_plane = (n1 n2 n3 -p0.n)
		//
		//so:
		return -cam_plane[3] / (unproject(cam2d) * cam_plane.slice<0,3>());
	}

	bool plane_hides(const Vector<3>& v) const
	{
		//Is this point hidden?

		//Well, how does it compare to the depth?
		//
		// If the plane is p1 p2 p3 p4
		//
		// Then projecting v and computing the depth of the plane
		// along the ray is:
		//
		//d = - p4 / ( (vx vy vz)/vz . (p1 p2 p3))
		//
		//For v to be hidden:
		//
		// vz > d
		//
		//Giving:
		//
		// vz > - p4 / ( (vx vy vz)/vz . (p1 p2 p3))
		//
		// Can't rearrange division of a signed thing over an inequality,
		// so do:
		//
		// vz > -      p4
		//          -------------
        //           V . P123 / vz

		//             p4
		//  vz > -  --------------------------------------
		//           |V.P123| sgn(V.P123) / (|vz| sgn(vz))

		// Rearranges to:
		//
		// sgn(vz)sgn( v.p123)  (vx vy vz 1).p > 0

	
		double v_dot_p_normal = v * cam_plane.slice<0,3>();
		double v_p = v_dot_p_normal + cam_plane[3];

		if( (v_dot_p_normal > 0) == (v[2] > 0))
			return v_p > 0;
		else
			return v_p < 0;
	
	}
};

inline EdgeWithNoOcclusion::EdgeWithNoOcclusion(Vertex*v1, Vertex* v2)
:vertex1(left(v1, v2)),
 vertex2(right(v1, v2))
{
	double Dy = vertex2->cam2d[1]-vertex1->cam2d[1];
	double Dx = vertex2->cam2d[0]-vertex1->cam2d[0];

	gradient = Dy / Dx;

	assert(Dx > 0);

	v1cam2d = vertex1->cam2d;
}

inline double EdgeWithNoOcclusion::y_at_x_of(const Vertex& v_x, bool debug_no_checks) const
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


		double dx = x - v1cam2d[0];

		assert(dx >= 0);

		return dx * gradient + v1cam2d[1];
	}
}

inline double EdgeWithNoOcclusion::y_at_x_of_unchecked(const Vertex& v_x) const
{
	return y_at_x_of(v_x, true);
}

inline bool EdgeWithNoOcclusion::a_is_on_left(const Vertex* a, const Vertex* b) const
{
	assert(a->cam2d[0] != b->cam2d[0]);
	return a->cam2d[0] < b->cam2d[0];
}



//Apparently it's faster to have y in a special struct, rather than putting
//it in Edge and have active_edges just be an array of Edge*. Perhaps this is
//because it keeps the y's close which is important since there are many accesses
//to this structure.
struct ActiveEdge
{
	Edge* edge;
	double y;
};


struct Intersection
{
	Vector<2> cam2d;
	Vector<3> front_pos;
	Vector<3> back_pos;
		
	Edge* front_edge, *back_edge;
	bool back_was_above;
};


vector<Vertex> get_vertices_without_edges(const SE3<>& E, const vector<Vector<3>>& model_vertices)
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
		vertices[i].index = i;
	}
	

	return vertices;
}

class ModelInformation
{
	private:
		vector<Vertex> vertices;
		vector<Edge> edges;
		vector<Face> faces;



	public:
		Renderer(const Model& m);
		
		void render(const SE3<>& E);
}

Renderer::Renderer(const Model& m)
{
	//Extract all the projection independent information.
	//We could defer filling in the 3D coordinates too.
	vertices.resize(m.vertices.size());
	for(size_t i=0; i < vertices.size(); i++)
		vertices[i].world = m.vertices[i];

	faces.resize(m.get_edges().size());

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
			face->vertices[j] = &vertices[m.get_edges()[i][j]];


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

			//Also associate the face with the vertex.
			v1->faces.insert(face);
			v2->faces.insert(face);
		}
	}
	
	//Pack the edges into a std::vector
	for(auto& e:s_edges)
		edges.push_back(e.second);



}


vector<EdgeSegment> Render::render(const SE3<>& E, const Model& m)
{

	/* Here's how it works:
		
	  1. Create data structures such that evere element (e.g. face) knows
	     about every related element (each face has a list of vertices and edges).

	  2. Transform vertices and face normals into camera coordinates, and compute
	     projection of vertices into image coordinates.
	  
	  3. For each vertex, in image coordinates record which edges connect on the
	     left and which connect on the right. For the right hand edges, sort them
		 from top to bottom accroding to angle.

	  4. Sort the vertices from left to right in image coordinates.
	  
	  5. Initialise an empty list of edges, E, called active edges. Each active edge
	     also records an occlusion depth (integer) and a set of occluding faces. Also
		 record the start of the current segment. In image coordinates, record which 
		 edges connect above the line and which connect below.

		 E will be the list of edges intersected by a vertical line which sweeps from
		 right to left.
	     
	  6. For each vertex, V:
	     
		 1. A vertex implies a vertical sweep line through the vertex.

		 2. Sort E according to the vertical position of their intersection
		    with the sweep line, top to bottom. Sort using bubble sort, and record
			each swap since this implies edges have crossed. For each crossing, compute
			the position in 2D, and corresponding positions in 3D. Find which edge
			is in front.

	     3. For each crossing:
		    
			1. If the occlusion depth of the back edge is zero, draw it from the start
			   of its current segment to the crossing point.

			2. For the back edge, set the start of the current segment to the crossing
			   point.

			3. If the back line starts above and ends below the front line, then remove
			   the front line's upper edges from the back edge occlusion set, and add the 
			   front line's lower edges to the occlusion set.

			   Update the occlusion depth to match the size of the occlusion set.

	      4.  Remove every active edge connecting to the current vertex. For each line
		      removed, draw it if the occlusion depth is zero.

	      5.  To determine the visibility of V , initialise empty sets O,A of faces.

		  6.  Loop over the E vertically top to bottom until and including the
			  y coordinate of the current vertex is reached. For each edge, for
			  each connected face insert the face into O. 
			  
			  Then discard every face appearing an even number oftimes.
		  	  
		  7.  For each face connected to V, if the face is in O, remove it from O and record
		      it in A. We remove them from the potential occluders, O, since faces connected
			  to the vertex cannot occlude the vertex.

			  A ray from the camera centre through the vertex will now intersect every face in 
			  O.

		  7.  For each face compute whether it is in front of or behind V. Keep a set of and 
		      count of all faces occluding V.

		  8.  It is now time to add all right edges of V into E at the correct place to avoid
		      disturbing the sort order. For each edge, we also need the its occlusion depth 
			  which must be >= the occlusion depth of the current vertex.

			  For each edge R in the right edges of V, sorted top to bottom:
			  
			  1. For each face connected to R, if the face is in A then remove it from A,
			     otherwise add it from A.

			  2. For each face in A, determine if the other end of R will be hidden by the 
			     plane associated with the face face. Use this information and the occlusion
				 of V to record the occlusion depth and list of occluders associated with R.


    Notes:

	Assuming there are no intersecting faces, the actual set of occluders (5, 6.3.3, 6.8.2) is
	unnecessary. Changes in occlusion can only happen at edges and an even number of crossings
	with edges of a face means the face cannot be occluding. Therefore it is sufficient and 
	much faster to increment/decrement a simple count.

    For determining the visibility of a vertex (6.5) if there are incoming edges from the right
	then it is sufficient to propagate occlusion depths from these edges. However if there are 
	fae intersections, then this will lead to cascading errors. Performing a full search each
	time will limit the drawing errors.

	
	The vertical walk in 6.6 can be either from the top or bottom. If it is from the bottom then
	the vertical walk in 6.8 must also be from the bottom. To increase speed, one can choose which
	end to search from depending on which end the vertex in question is closest to.


	*/




cvd_timer T;
cvd_timer U;

#define X(Y) cerr << Y << " = " << T.reset() * 1000 << " ms" << endl
T.reset();
	vector<Vertex> vertices = get_sorted_list_of_camera_vertices_without_edges(E, m.vertices);
X("get_sorted_list_of_camera_vertices_without_edges");

	vector<Vertex*> ordered_vertices;
	for(auto& v:vertices)
		ordered_vertices.push_back(&v);

	//Sort the vertices left to right
	sort(ordered_vertices.begin(), ordered_vertices.end(), 
		[](const Vertex* v1, const Vertex* v2)
		{
			return v1->cam2d[0]  < v2->cam2d[0];
		}
	);
	
	//Now, some vertices might share the same x coordinate. 
	//Fix this, with an evil hack. Note that this deals
	//with multiple vertices sharing the same x coordinate.
	double x_prev = ordered_vertices[0]->cam2d[0];
	for(size_t i=1; i < ordered_vertices.size(); i++)
	{
		if(ordered_vertices[i]->cam2d[0] == x_prev)
			ordered_vertices[i]->cam2d[0] = ordered_vertices[i-1]->cam2d[0] + x_delta;
		else
			x_prev = vertices[i]->cam2d[0];
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
				face->vertices[j] = &vertices[m.get_edges()[i][j]];


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

				//Also associate the face with the vertex.
				v1->faces.insert(face);
				v2->faces.insert(face);
			}
		}
		
		//Pack the edges into a std::vector
		for(auto& e:s_edges)
			edges.push_back(e.second);
	}
X("find_edges");

	//Now go and propagate the edge information to the faces and vertices
	for(auto& edge: edges)
	{
		for(unsigned int i=0; i < edge.faces.size(); i++)
			edge.faces[i]->edges.push_back(&edge);
		
		
		//vertex1 is to the left of vertex2
		edge.vertex1->right_edges.push_back(&edge);
		edge.vertex2->left_edges.push_back(&edge);
	}

X("populate_faces");

	//Now do the final sorting on the right hand edges of each vertex.
	for(auto& v:vertices)
		v.sort();

X("sort_right_edges");

	
	//Compute the face normals
	for(auto& f:faces)
		f.compute_data(E);

X("compute_normals");


	//Compute whether we should render or not.
	//FIXME make this nicer.

	for(auto& e:edges)
	{
		if(e.faces.size() != 2)
			e.should_render = true;
		else
		{
			Vector<3> n1 = e.faces[0]->cam_plane.slice<0,3>();
			Vector<3> n2 = e.faces[1]->cam_plane.slice<0,3>();
			
			//The normal sign is arbitrary, so make sure they point
			//in more or less the same direction.
			if(n1 * n2 < 0)
				n2 = -n2;
			
			Vector<3> a_ray = e.vertex1->cam3d;


				
			if( (n1 * a_ray < 0) != (n2 * a_ray < 0))
				e.should_render = true;
			else
				e.should_render = false;
		}

		e.should_render=true;
	}


	//At this point we have a sorted list of vertices (left to right), 
	//and faces, vertices and edges with all cross referencing
	//information.

		
	
	//The final list of edge segments to record.
	vector<EdgeSegment> output;


	//Active adges will the list of edges intersecting with the current
	//vertival sweep line.
	vector<ActiveEdge> active_edges;


double tind=0;
double tbubble=0;
double tlookup=0;
double tsortcrossing=0;
double tproccrossing=0;
double toutputcrossing=0;
double tremoveincoming=0;
double teraseincoming=0;
double tfacesactive=0;
double tfacesatvertex=0;
double tdepthtest=0;
double tinsertpos=0;
double tnewactiveedges=0;
double tinsertactive=0;

#define P(X) cerr << #X " = " << X *1000 << " ms " << endl;
	
	//unordered_set<const Face*> faces_active;
	PointerSet<Face> faces_active(faces);
	unordered_set<const Face*> faces_at_vertex;

	for(const auto& vv: ordered_vertices)
	{
		
		const Vertex& v = *vv;

		#ifdef DEBUG
			auto debug_order_at_v = [&](const ActiveEdge& e1, const ActiveEdge& e2)
			{
				return e1.edge->y_at_x_of_unchecked(v) < e2.edge->y_at_x_of_unchecked(v);
			};
		#endif

		//Horizontal position of the sweep line
		//double x = v.cam2d[0];

		//Find the crossings by re-sorting.
		//
		//Note that each crossing needs to store a reference to 
		//an ActiveEdge, but the active edges are being constantly
		//moved during the sort. So, we need to refer to each by a unique
		//identifier. So, give each one a unique integer and use this as
		//the basis of a lookup later.
T.reset();
		for(unsigned int i=0; i < active_edges.size(); i++)
		{
			active_edges[i].y = active_edges[i].edge->y_at_x_of(v);
		}
tind += T.reset();


		//Sort edges top to bottom according to the intersection with the sweep
		//line using bubble sort since each exchange corresponds to a crossing.
		//Thanks, Tom!!!
		vector<Intersection> crossings;
		for(int n=active_edges.size(); ;)
		{
			bool swapped=false;
			int new_n=0;
			
			for(int i=1; i < n; i++)
			{
				if(active_edges[i-1].y > active_edges[i].y)
				{
					swap(active_edges[i-1], active_edges[i]);

					const ActiveEdge& e1 = active_edges[i];
					const ActiveEdge& e2 = active_edges[i-1];

					//Find where the crossing occurs.
					//
					//We know that straight lines project to straight lines. If we have
					//in 3D X = A + lambda B, then two projection points are l=0 and 
					//l = infinity (start and vanishing point) giving:
					// x_0 = P(A), x_inf = P(B)
					//
					//So the line in 2D is:
					//
					// x = b + delta (a-b)
					//
					// d = A3 / (A3 + l * B3)
					//

					Vector<3> A = e1.edge->vertex1->cam3d; //First point
					Vector<3> B = e1.edge->vertex1->cam3d - e1.edge->vertex2->cam3d; //Direction
					Vector<2> b = project(B); //Start point of the line in 2D (the vanishing point)
					Vector<2> a = e1.edge->vertex1->cam2d - b; //Direction of the line in 2D

					Vector<3> C = e2.edge->vertex1->cam3d;
					Vector<3> D = e2.edge->vertex1->cam3d - e2.edge->vertex2->cam3d;
					Vector<2> d = project(D);
					Vector<2> c = e2.edge->vertex1->cam2d - d;

					Matrix<2> m;
					m.T()[0] = a;
					m.T()[1] = -c;

					Vector<2> coeff = inv(m) * (d-b);
					
					Intersection intersection;
					intersection.cam2d = coeff[0] * a + b;

					double lambda = (A[2] - coeff[0] * A[2]) / (coeff[0] * B[2]);
					double delta  = (C[2] - coeff[1] * C[2]) / (coeff[1] * D[2]);

					Vector<3> e1_pos = (A + lambda * B);
					Vector<3> e2_pos = (C + delta  * D);

					if(e1_pos[2] < e2_pos[2])
					{
						intersection.front_edge = e1.edge;
						intersection.back_edge = e2.edge;
						intersection.front_pos = e1_pos;
						intersection.back_pos = e2_pos;
						intersection.back_was_above=false;
					}
					else
					{
						intersection.front_edge = e2.edge;
						intersection.back_edge = e1.edge;
						intersection.front_pos = e2_pos;
						intersection.back_pos = e1_pos;
						intersection.back_was_above=true;
					}

					crossings.push_back(intersection);
					
					swapped=true;
					new_n=i;
				}
			}

			if(!swapped)
				break;
			n = new_n;
		}
tbubble += T.reset();
		assert(is_sorted(active_edges.begin(), active_edges.end(), debug_order_at_v));

tlookup += T.reset();

		//Sort the intersections left to right. Given there are up to n^2 intersections
		//this is at worst (n log n)^2
		//
		//This is so we can alter the visibility of the segments left to right as 
		//they change. In principle, we could improve the order by finding all edges
		//with intersections in front of them and doing a per-edge sort.
		//Naturally, edges in front without faces should be ignored.
		//
		//We'll try that is the following sort ever proves too slow!
		sort(crossings.begin(), crossings.end(), 
			[](const Intersection& a, const Intersection& b)
			{
				return a.cam2d[0] < b.cam2d[0];
			});
tsortcrossing += T.reset();

		//Now process the crossings
		for(const auto& c: crossings)
		{
			Edge& front = *c.front_edge;
			Edge& back  = *c.back_edge;

			//If the previous occlusion depth was zero, emit an edge from the previous point
			//to the current point. Note that this will generate a superfluous edge pair if
			//the front edge has no faces!
			if(back.occlusion_depth==0 && back.should_render)
			{
				EdgeSegment e;
				e.a3d = back.previous_3d;
				e.a2d = back.previous_2d;

				e.b3d = c.back_pos;
				e.b2d = c.cam2d;
				
				output.push_back(e);
			}
			

			//If the back edge was above the front edge originally, then
			//all the faces connected above the front edge should be currently
			//occluding. 
			//
			//The assertions hold only if there are no self-intersecting faces.
			//Therefore for safety, leave them out.
			for(auto f: front.faces_above)
				if(c.back_was_above)
					back.remove_occluding_face(f);
				else
					back.add_occluding_face(f);

			for(auto f: front.faces_below)
				if(c.back_was_above)
					back.add_occluding_face(f);
				else
					back.remove_occluding_face(f);

			//Record the previous vertex posision.
			back.previous_2d = c.cam2d;
			back.previous_3d = c.back_pos;
		}

tproccrossing+=T.reset();

		//Some sanity checks: make sure that every edge terminating at the current vertex is
		//active.
		#ifdef DEBUG
			for(const auto& e:v.left_edges)
				assert(find_if(active_edges.begin(), active_edges.end(), [&](const ActiveEdge& a){return a.edge ==e;}) != active_edges.end());
		#endif
		

		//Note that not all incoming edges will have the same occlusion depth
		//some of them may be occluded by faces belonging to the vertex.


		double vertex_y = v.cam2d[1];

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
		//worsen the overall speed.

		auto edge_terminates_here = [&](const ActiveEdge& e)
		{
			return e.edge->vertex2 == &v;
		};
		
		//First, emit any unoccluded edges terminating at this vertex.

		//Edges are sorted by Y, so we can get to about the right place
		//very quicky.
		auto first_candidate=lower_bound(active_edges.begin(), active_edges.end(), vertex_y,
		                                 [&](const ActiveEdge& e, double y)
										 {
										 	return e.y < y;
										 });

		auto first_incoming_edge = find_if(first_candidate, active_edges.end(), edge_terminates_here);

		for(auto e = first_incoming_edge; e < active_edges.end(); e++)
			if(edge_terminates_here(*e) && e->edge->occlusion_depth==0)
			{
				if(e->edge->should_render)
				{
					EdgeSegment s;
					s.a3d = e->edge->previous_3d;
					s.a2d = e->edge->previous_2d;

					s.b3d = v.cam3d;
					s.b2d = v.cam2d;
					
					output.push_back(s);
				}
			}
			else if(e->y > vertex_y)
				break;

toutputcrossing += T.reset();
		
		auto erase_from = 	remove_if(first_incoming_edge, active_edges.end(), edge_terminates_here);

tremoveincoming += T.reset();
		active_edges.erase(erase_from , active_edges.end());

teraseincoming += T.reset();

		//Perform a vertical walk downwards along edges to see which faces come and go
		//as the walk is performed, until we hit the current vertex. Remember incoming
		//edges are gone.
		//
		//One could perform a walk upwards or downwards, depending on how close
		//to the top or bottom the current vertex is, for a factor of 2 saving. However,
		//one must perform the later miniture walk in the same direction.

		faces_active.clear();
		faces_at_vertex.clear();

		//Where are we?
		auto v_pos=lower_bound(active_edges.begin(), active_edges.end(), vertex_y,
		                                 [&](const ActiveEdge& e, double y)
										 {
										 	return e.y < y;
										 });
		
		bool from_top = v_pos-active_edges.begin() < (ptrdiff_t)active_edges.size()/2;

		if(from_top)
		{
			for(const auto& e:active_edges)
			{
				if(e.y > vertex_y)
					break;
				
				//Only consider faces which could possibly
				//span this vertex.
				for(auto& f:e.edge->faces)
					faces_active.flip(f);
			}
		}
		else
		{
			for(auto e = active_edges.rbegin() ; e != active_edges.rend(); e++)
			{
				if(e->y < vertex_y)
					break;
				
				//Only consider faces which could possibly
				//span this vertex.
				for(auto& f:e->edge->faces)
					faces_active.flip(f);
			}
		}

tfacesactive+=T.reset();
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
		//
		//However, keep a list of these faces since we'll need them later to 
		//determine the visibility of edges coming from this vertex.
		for(auto e:v.left_edges)
			for(auto f:e->faces)
				if(faces_active.erase(f))
					faces_at_vertex.insert(f);
tfacesatvertex+=T.reset();

		//Now, we need to check the vertex against all remaining active planes to 
		//see if it is occluded.
		//
		//In principle, we could propagate occlusion information from all incoming
		//edges to the current vertex. If, however there are intersecting faces
		//then that can lead to cascading errors. A fresh computation here will
		//stop the cascade, so only minor rendering errors will result.
		int occlusion_depth=0;
		unordered_set<const Face*> occluders;
		for(auto f: faces_active)
		{
			
			#ifdef DEBUG
				for(const auto& fv:f->vertices)
					assert(fv != &v);
			#endif
			
			double depth = f->depth(v.cam2d);

			if(depth < v.cam3d[2])
			{
				occlusion_depth++;
				occluders.insert(f);
			}
		}
	
tdepthtest += T.reset();
		//Some sanity checks: no left edges should remain active.
		#ifdef DEBUG
			for(auto e:v.left_edges)
				assert(find_if(active_edges.begin(), active_edges.end(), [&](const ActiveEdge& a){return a.edge ==  e;}) == active_edges.end());
		#endif

		//Edges are sorted top to bottom by intersection with the 
		//sweep line. Find the position to insert the new edges.
		//
		//upper bound returns a pointer to the first element greater
		//than the y coordinate. vector::insert will insert just before
		//this iterator.
		auto here = upper_bound(active_edges.begin(), active_edges.end(), v.cam2d[1],
								[&](double y, const ActiveEdge& e)
								{
									return y < e.edge->y_at_x_of(v);
								}
					);
tinsertpos += T.reset();

		//Set up right edges before inserting them, into active_edges.
		//Note that the complete occlusion depth is required, which cannot be 
		//inferred purely from the occlusion depth of the vertex.
		//
		//Walk downwards through the right hand edges. Note that this in itself is not
		//sufficient to determine where a face is on, because we cannot tell if we're
		//starting inside a face or not. For example:
		//               
		//        Vertical line epsilon to the right of the vertex X
		//               |                     |       
		//           ____|__                   |      _,-/
		//           \   | /                   |  _,-' ,/                                          .
		//            \  |/                    ,-'   ,/                                            .
		//       a     \ /                    X|   ,/                                                                          
		//              X|                   / | ,/                                                       
		//               |                  /  |/                                                         
		//               |                 / ,/|                                                          
		//                                /,/  |                                           
		//                               //    |
		//                               '
		//  Note that considering only lines emanating from the right of X, we start inside
		//  the face, so crossing the edge takes us to the outside. Therefore, we need a way
		//  of knowing which faces we start inside. There are several options:
		//
		//  1. For triangular faces, we know that if the triangle has a left edge and a right 
		//     edge AND the bisector at X points up, then we must start inside the triangle.
		//     Doable, but yuck.
		//
		//  2. For general faces there is no way of telling from just the edges at X. Instead
		//     count all crossings of the face edges with the vertical line.
		//     
		// We've already done this vertical walk above to see which faces are active at the 
		// current vertex, and which belong to the current vertex.
		//
		// During this walk, also figure whether faces connect above or below
		// the edge. 


		//Find out which faces are (a) active and (b) belong to the current vertex.
		vector<ActiveEdge> right(v.right_edges.size());
		
		for(unsigned int i=0; i < v.right_edges.size(); i++)
		{
			int j;
			if(from_top)
				j=i;
			else
				j = v.right_edges.size() - 1 - i;

			Edge* e = v.right_edges[j];

			ActiveEdge a;
			a.edge = e;

			e->set_occluders(occluders, occlusion_depth);

			e->previous_3d=v.cam3d;
			e->previous_2d=v.cam2d;

			//First, remove any faces associated with the current edge
			vector<const Face*> to_be_added;
			for(auto& f: e->faces)
				if(faces_at_vertex.count(f))
				{
					faces_at_vertex.erase(f);
					e->faces_above.push_back(f);
				}
				else
				{
					to_be_added.push_back(f);
					e->faces_below.push_back(f);
				}
			
			//Now any remaining faces are active and may or may not hide 
			//the current edge, depending on the relative angle.
			//
			//Obviously v1 is on the face, so we can check whether v2
			//is hidden by the plane (note plane, not face). This tells
			//us if the line starts off hidden.
			for(auto f:faces_at_vertex)
			{

				if(f->plane_hides(e->vertex2->cam3d))
				{
					//The current active face really shouldn't be in the way already
					F(assert(e->occluding_faces.count(f) == 0);)

					e->add_occluding_face(f);
				}
			}

			//Now insert edges which need to be inserted.
			for(auto& f:to_be_added)
				faces_at_vertex.insert(f);

			right[j] = a;
		}


tnewactiveedges+=T.reset();

		active_edges.insert(here, right.begin(), right.end());
tinsertactive+=T.reset();
		assert(is_sorted(active_edges.begin(), active_edges.end(), debug_order_at_v));
	}

P(tind);
P(tbubble);
P(tlookup);
P(tsortcrossing);
P(tproccrossing);
P(toutputcrossing);
P(tremoveincoming);
P(teraseincoming);
P(tfacesactive);
P(tfacesatvertex);
P(tdepthtest);
P(tinsertpos);
P(tnewactiveedges);
P(tinsertactive);

P(U.reset());

	return output;
}
