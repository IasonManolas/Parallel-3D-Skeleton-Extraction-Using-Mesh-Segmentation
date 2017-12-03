#ifndef NODE_H
#define NODE_H
using VertexIndex = size_t;
template <typename Point3D>
struct Node {
	Node(VertexIndex v, Point3D p) : v(v), p(p) {}

	VertexIndex v;
	Point3D p;
};
#endif
