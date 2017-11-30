#ifndef BIDIRECTIONAL3DGRAPH_H
#define BIDIRECTIONAL3DGRAPH_H

#include <cgaltypedefs.h>
#include <boost/graph/adjacency_list.hpp>

class Bidirectional3DGraph {
	using BoostGraph = boost::adjacency_list<boost::setS, boost::vecS,
						 boost::bidirectionalS>;
	using GraphVertexIndex = BoostGraph::vertex_descriptor;
	using MeshVertexIndex = size_t;

	BoostGraph m_graph;
	std::vector<MeshVertexIndex> m_graphVertexIndex_to_meshVertexIndex;

       public:
	Bidirectional3DGraph() {}

	template <typename Point>
	concept PointIn3DSpace = requires(Point p) {
		a.x()->double;
		a.y()->double;
		a.z()->double;
	};
	using indexIn_nodes = size_t;
	void add(std::vector<std::pair<PointIn3DSpace, MeshVertexIndex>> nodes,
		 std::vector < std::pair<indexIn_nodes, indexIn_nodes> edges) {
		nodePositions
	}
};

#endif
