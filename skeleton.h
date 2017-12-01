#ifndef SKELETON_H
#define SKELETON_H

#include <boost/graph/adjacency_list.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
#include "cgaltypedefs.h"
#include "drawableskeleton.h"
#include "edge.h"
#include "undirectedgraph.h"

class Skeleton : public DrawableSkeleton {
	// public member functions
	// NOTE PointSphere should not be an argument
	// but for some reason PointSphere cannot be
	// loaded in this class but only through
	// Scene::load.Maybe because
	// GLWidget::makeCurrent() needs to be called?
	using BoostGraph =
	    boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS>;
	using GraphVertexDescriptor =
	    BoostGraph::vertex_descriptor;  // this represents a descriptor of a
	// vertex in boost::adjacency_list
	using MeshVertexIndex =
	    size_t;  // this represents an index in the actual mesh

	BoostGraph m_graph;
	std::vector<MeshVertexIndex> vd_to_meshIndex;

       public:
	Skeleton(const PointSphere &ps) : DrawableSkeleton(ps) {}
	void clear() { DrawableSkeleton::clearDrawableSkeleton(); }

	const std::vector<glm::vec3> &getNodePositions() const {
		return DrawableSkeleton::m_vertices;
	}
	size_t getNumberOfNodes() const { return boost::num_vertices(m_graph); }
	void initialize() {  // is called when there is an active OpenGL context
		DrawableSkeleton::initializeDrawingBuffers();
	}

	using Edge = Edge<CGALSurfaceMesh::Point>;
	// constructs the skeleton of the whole mesh
	void populateSkeleton(std::vector<Edge> edges) {}

	// adds to the skeleton one part based on the segment graph and the
	// provided index
	void addSkeletonPart(std::vector<Edge> edges, UndirectedGraph m_graph) {
		for (const Edge &e : edges) {
		}
	}

	void append(std::vector<std::vector<size_t>> newEdges,
		    std::vector<CGALSurfaceMesh::Point> newNodePositions) {
		// appendToGraph(newEdges, newNodePositions);
		appendEdges(newEdges);
		// appendNodes(newNodePositions);
		updateMeshBuffers();
	}

       private:
	void appendEdges(const std::vector<std::vector<size_t>> &newEdges) {
		for (const std::vector<size_t> edge : newEdges) {
			m_indices.push_back(edge[0] + m_vertices.size());
			m_indices.push_back(edge[1] + m_vertices.size());
		}
	}
	void appendNodes(std::vector<CGALSurfaceMesh::Point> newNodePositions,
			 PointSphere psPrototype) {
		for (CGALSurfaceMesh::Point p : newNodePositions) {
			PointSphere tempPS = psPrototype;
			tempPS.setPosition(p);
			tempPS.setColor(glm::vec3(1, 0, 0));
			m_drawingVector.push_back(tempPS);

			m_vertices.push_back(glm::vec3(p.x(), p.y(), p.z()));
		}
	}
};

inline Skeleton join(const Skeleton &s1, const Skeleton &s2) {}
inline double glmSquaredDistance(glm::vec3 v1, glm::vec3 v2) {
	return std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2) +
	       std::pow(v1.z - v2.z, 2);
}

inline std::pair<size_t, size_t> findClosestNodes(
    const Skeleton &s1,
    const Skeleton
	&s2)  // returns pair(indexInFirstSkeleton,indexInSecondSkeleton)
{
	const std::vector<glm::vec3> &s1Nodes = s1.getNodePositions();
	const std::vector<glm::vec3> &s2Nodes = s2.getNodePositions();
	double minDistance = glmSquaredDistance(s1Nodes[0], s2Nodes[0]);
	std::pair<size_t, size_t> indicesOfMinDistance(0, 0);
	for (size_t index1 = 0; index1 < s1.getNumberOfNodes(); index1++) {
		glm::vec3 s1NodePos = s1Nodes[index1];
		for (size_t index2 = 0; index2 < s2.getNumberOfNodes();
		     index2++) {
			glm::vec3 s2NodePos = s2Nodes[index2];
			double d = glmSquaredDistance(s1NodePos, s2NodePos);
			if (d < minDistance) {
				minDistance = d;
				indicesOfMinDistance =
				    std::pair<size_t, size_t>(index1, index2);
			}
		}
	}
	return indicesOfMinDistance;
}

#endif  // SKELETON_H
