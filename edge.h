#ifndef EDGE_H
#define EDGE_H
using VertexIndex = size_t;  // vertex index in a mesh
template <typename Point3D>
struct Edge {
	Edge(VertexIndex v1, Point3D p1, VertexIndex v2, Point3D p2)
	    : v1(v1), p1(p1), v2(v2), p2(p2) {}

	VertexIndex v1;
	Point3D p1;

	VertexIndex v2;
	Point3D p2;
};
#endif
