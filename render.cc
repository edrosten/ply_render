#include "model_loader.h"
#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>
#include <cvd/camera.h>
#include <cvd/vector_image_ref.h>
#include <algorithm>
#include <exception>
#include <set>
#include <iomanip>
#include <TooN/se3.h>
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


#define DEBUG

template<class C, size_t Size> class Array: private array<C, Size>
{
	public:
		using array<C, Size>::size;

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



template<class C> void unset(C& c)
{
	c=C();
}
template<class C*> void unset(C& c)
{
	c = reinterpret_cast<C>(0xbadc0ffee0ddf00d);
}

//Edge structure: an edge consists of two vertices
//and a list of faces.
//
//Edges are stored left most vertex first
struct Edge
{
	Vertex *vertex1, *vertex2;
	static_vector<Face*, 2> faces;

	Edge(Vertex*v1, Vertex* v2)
	:vertex1(left(v1, v2)),vertex2(right(v1, v2))
	{
	}


	#ifdef DEBUG
		~Edge()
		{
			unset(vertex1);
			unset(vertex2);
		}

	#endif
	
	private:
		inline bool a_is_on_left(const Vertex* a, const Vertex* b);

		Vertex* left(Vertex* a, Vertex* b)
		{
			if(a_is_on_left(a, b))
				return a;
			else
				return b;
		}

		Vertex* right(Vertex* a, Vertex* b)
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
	Vector<3> cam3d; //3d vertex position in camera coordinates
	Vector<2> cam2d; //2d veretx position in camera pixel coordinates
	int index; //Which vertex is this?

	vector<Edge*> left_edges; //List of edges to the left of the current point
	vector<Edge*> right_edges; //List of edges to the right of the current point
};

struct Face
{
	Array<Vertex*,3> vertices;
	static_vector<Edge*, 3> edges;
};

inline bool Edge::a_is_on_left(const Vertex* a, const Vertex* b)
{
	assert(a->cam2d[0] != b->cam2d[0]);
	return a->cam2d[0] < b->cam2d[0];
}


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
		vertices[i].cam3d = E*model_vertices[i];
		vertices[i].cam2d = cam.project(project(vertices[i].cam3d));
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


	auto cross=[](const Vector<2>& v)
	{
		glColor3f(1,0,0);
		int size=3;
		glVertex(v+makeVector( size, 0));
		glVertex(v+makeVector(-size, 0));
		glVertex(v+makeVector(0,  size));
		glVertex(v+makeVector(0, -size));
	};

	//At this point we have a sorted list of vertices, 
	//and faces, vertices and edges with all cross referencing
	//information.
	for(auto v: vertices)
	{
		glClear(GL_COLOR_BUFFER_BIT);
		debug_draw_all(m, cam, E);
		glBegin(GL_LINES);

		glColor3f(.5, 0, 0);
		glVertex2f(v.cam2d[0], 0);
		glVertex2f(v.cam2d[0], 480);

		for(auto e:v.left_edges)
		{
			glColor3f(1, 1, 0);
			glVertex(e->vertex1->cam2d);
			glColor3f(0, 1, 0);
			glVertex(e->vertex2->cam2d);
		}

		for(auto e:v.right_edges)
		{
			glColor3f(0, 1, 0);
			glVertex(e->vertex1->cam2d);
			glColor3f(0, 1, 1);
			glVertex(e->vertex2->cam2d);
		}

		cross(v.cam2d);

		glEnd();
		glFlush();
		cin.get();

	}

}
