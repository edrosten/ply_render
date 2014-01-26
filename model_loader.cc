/* Copyright (C) Computer Vision Consulting, 2013.*/
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <sys/types.h>
#include <sys/wait.h>

#include <vector>
#include <array>
#include <stdexcept>
#include <TooN/TooN.h>
#include <tag/stdpp.h>
#include <map>
#include <boost/iostreams/filter/zlib.hpp>


extern "C"{
#include "rply.h"
}
#include "model_loader.h"

using namespace std;
using namespace tag;
using namespace TooN;
using namespace boost;

class Model::Getter{
	public:
		static std::vector<std::array<int,3>>& get(Model& m)
		{
			return m.edges;
		}
};

struct Stuff{
	Model* m;
	int vertex, edge;
};

int v_cb(p_ply_argument arg)
{
	Stuff* d;
	void* v;
	long l;
	ply_get_argument_user_data(arg, (&v), &l);
	d = static_cast<Stuff*>(v);
	d->m->vertices[d->vertex][l] = ply_get_argument_value(arg);	
	if(l == 2)
		d->vertex++;

	return 1;
}

int e_cb(p_ply_argument arg)
{
	Stuff* d;
	void* v;
	long l, length, value_index;
	ply_get_argument_user_data(arg, (&v), &l);
	d = static_cast<Stuff*>(v);
	ply_get_argument_property(arg, NULL, &length, &value_index);
	int value = (int)ply_get_argument_value(arg);
	
	if(value_index == -1)
	{
		if(value != 3)
		{
			cerr << "Error: face is not a triangle :(\n";
			return 0;
		}
	}
	else
	{
//cerr << "Edge: " << d->edge << " " << d->m->edges.size() << endl;
//cerr << "value_index: " << value_index << " " << value << " " << d->m->vertices.size() << endl;
		Model::Getter::get(*(d->m))[d->edge][value_index] = value;
		if(value_index == 2)
			d->edge++;
	}
		
	return 1;
}

inline pair<int, int> ord(int a, int b)
{
	if(a < b)
		return make_pair(a, b);
	else
		return make_pair(b, a);
}


class ZLibFile
{
	private:
		int fd;
		//hee hee
		vector<char> temp;

		string fname;
		const char* f;

	public:
		ZLibFile(const string& fname_)
		:fname(fname_)
		{
			string tmps="/tmp/tmp_XXXXXX";
			for(auto a: tmps)
				temp.push_back(a);
			temp.push_back(0);
			

			int fd = mkstemp(temp.data());

			//(try to) decompress the file
			pid_t child = fork();
			if(child == 0)
			{
				//Connect stdout to fd	
				dup2(fd, 1);
				if(execlp("zcat", "zcat", fname.c_str(), 0) == -1)
					cerr << "Error in exec: " << strerror(errno) << endl;
				exit(5);
			}
			
			int status=-1;
			wait(&status);

			if(WEXITSTATUS(status) != 0)//Failed
				f = fname.c_str();
			else
				f = temp.data();
		}
		
		const char* file() const
		{
			return f;
		}	

		~ZLibFile()
		{
			unlink(temp.data());
		}
};


Model::Model(const std::string& fname)
{
	Stuff dat = {this, 0, 0};

	ZLibFile zlf(fname);

	p_ply ply = ply_open(zlf.file(), 0, 0, 0);
	
	
	if (!ply)
		throw "error";
	if (!ply_read_header(ply)) 
		throw "another error";

	int nvertices = ply_set_read_cb(ply, "vertex", "x", v_cb, &dat, 0);
	ply_set_read_cb(ply, "vertex", "y", v_cb, &dat, 1);
	ply_set_read_cb(ply, "vertex", "z", v_cb, &dat, 2);

	vertices.resize(nvertices);
	
	int ntriangles = ply_set_read_cb(ply, "face", "vertex_indices", e_cb, &dat, 0);
	int also_n_triangles = ply_set_read_cb(ply, "face", "vertex_index", e_cb, &dat, 0);
	edges.resize(max(ntriangles, also_n_triangles));


	if(!ply_read(ply))
		throw "baaad";


	ply_close(ply);

	//for(int i=0; i < numVert; i++)
	//	vertices.push_back(makeVector(vertices[i*3], vertices[i*3+1], vertices[i*3+2]));

	//edges.resize(numPoly);
	/*for(int i=0, j=0; i < numPoly; i++, j+=4)
	{
		cout << edgelist[j] << endl;
		if(edgelist[j] != 3)
			throw runtime_error("fuck");
		edges[i][0] = edgelist[j+1];	
		edges[i][1] = edgelist[j+2];	
		edges[i][2] = edgelist[j+3];	
	}*/


	map<pair<int, int>, int > edge_counts;

	for(unsigned int i=0; i < edges.size(); i++)
	{
		//Insert edges into map. Edges which are only used once
		//are the edges of the model.
		edge_counts[ord(A(i), U(i))]++;
		edge_counts[ord(A(i), V(i))]++;
		edge_counts[ord(U(i), V(i))]++;
	}

	
	array<bool,3> zeros={{0,0,0}};
	are_edges_on_holes.resize(edges.size(), zeros);
	boundary_vertex.resize(vertices.size(), false);
	for(unsigned int i=0; i < edges.size(); i++)
	{
		if(edge_counts[ord(A(i), U(i))] == 1)
		{
			boundary_vertex[A(i)] = true;
			boundary_vertex[U(i)] = true;
			are_edges_on_holes[i][0] = 1;
		}

		if(edge_counts[ord(A(i), V(i))] == 1)
		{
			boundary_vertex[A(i)] = true;
			boundary_vertex[V(i)] = true;
			are_edges_on_holes[i][1] = 1;
		}

		if(edge_counts[ord(U(i), V(i))] == 1)
		{
			boundary_vertex[U(i)] = true;
			boundary_vertex[V(i)] = true;
			are_edges_on_holes[i][2] = 1;
		}
	}


}

Model::Model()
{}

void Model::write_PLY_with_vertex_colors(ostream& o, const vector<array<int,3>>& c)
{
	o <<
		"ply" << endl << 
		"format ascii 1.0" << endl << 
		"element vertex " << vertices.size() << endl <<
		"property float x" << endl << 
		"property float y" << endl << 
		"property float z" << endl << 
		"property uchar red" << endl << 
		"property uchar green" << endl << 
		"property uchar blue" << endl << 
		"element face " << edges.size()  << endl << 
		"property list uchar int vertex_index" << endl << 
		"end_header" << endl;

	
	for(int i=0; i < vertices.size(); i++)
		o << vertices[i] << " " << c[i][0] << " " << c[i][1] << " " << c[i][2] << endl;

	for(const auto& e: edges)
		o << print << 3 << e[0] << e[1] << e[2];
}
void Model::write_PLY(ostream& o) const
{
	o <<
		"ply" << endl << 
		"format ascii 1.0" << endl << 
		"element vertex " << vertices.size() << endl <<
		"property float x" << endl << 
		"property float y" << endl << 
		"property float z" << endl << 
		"element face " << edges.size()  << endl << 
		"property list uchar int vertex_index" << endl << 
		"end_header" << endl;

	
	for(auto v: vertices)
		o << v << endl;

	for(const auto& e: edges)
		o << print << 3 << e[0] << e[1] << e[2];
}

void Model::write_STL(ostream& o, const string& name)
{
	o << "solid " << name << endl;
	for(const auto& e : edges)
	{
		o << "facet normal 0 0 0" << endl;
		o << "outer loop" << endl;
		for(auto i:e)
			o << "vertex " << vertices[i] << endl;
		o << "endloop" << endl;
		o << "endfacet" << endl;
	}
}
