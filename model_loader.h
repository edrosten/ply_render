#ifndef INC_READ_PLY_H
#define INC_READ_PLY_H

#include <TooN/TooN.h>
#include <vector>
#include <array>

class Model
{
	std::vector<std::array<bool,3>> are_edges_on_holes;
	
	
	friend class Getter;
	std::vector<std::array<int,3>> edges;

	std::vector<bool> boundary_vertex;

	public:
	class Getter;

	const std::vector<std::array<int,3>>&  get_edges() const
	{
		return edges;
	}

	std::vector<TooN::Vector<3> > vertices;

	Model(const std::string&);
	Model();
	
	bool is_vertex_on_boundary(int i) const
	{
		return boundary_vertex[i];
	}

	TooN::Vector<3> A_pos(int i) const
	{
		return vertices[A(i)];
	}

	TooN::Vector<3> U_pos(int i) const
	{
		return vertices[U(i)];
	}
	TooN::Vector<3> V_pos(int i) const
	{
		return vertices[V(i)];
	}

	bool is_AU_on_hole(int i) const
	{
		return are_edges_on_holes[i][0];
	}

	bool is_AV_on_hole(int i) const
	{
		return are_edges_on_holes[i][1];
	}

	bool is_UV_on_hole(int i) const
	{
		return are_edges_on_holes[i][2];
	}

	int A(int i) const
	{
		return edges[i][0];
	}

	int U(int i) const
	{
		return edges[i][1];
	}
	int V(int i) const
	{
		return edges[i][2];
	}

	void write_STL(std::ostream& o, const std::string& name);
	void write_PLY(std::ostream& o);
	void write_PLY_with_vertex_colors(std::ostream& o, const std::vector<std::array<int, 3>>& col);

};

Model load_model(const std::string& fname);

#endif
