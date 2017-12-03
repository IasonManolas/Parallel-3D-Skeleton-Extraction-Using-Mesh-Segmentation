#ifndef EDGE_H
#define EDGE_H
#include "node.h"
struct Edge {
	Edge(Node n1, Node n2) : n1(n1), n2(n2) {}

	Node n1;
	Node n2;
};
#endif
