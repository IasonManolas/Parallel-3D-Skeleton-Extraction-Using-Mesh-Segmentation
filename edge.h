#ifndef EDGE_H
#define EDGE_H
template <typename Point3D>
struct Edge {
	Edge(size_t v1, std::unique_ptr<Point3D> p1, size_t v2,
	     std::unique_ptr<size_t> p2)
	    : v1(v1), p1(p1), v2(v2), p2(p2) {}

	size_t v1;
	std::unique_ptr<Point3D> p1;

	size_t v2;
	std::unique_ptr<Point3D> p2;
};
#endif
