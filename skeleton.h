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

       public:
	Skeleton(const PointSphere &ps) : DrawableSkeleton(ps) {}
	void clear() { DrawableSkeleton::clearDrawableSkeleton(); }

	const std::vector<glm::vec3> &getNodePositions() const {
		return DrawableSkeleton::m_vertices;
	}
	void initialize() {  // is called when there is an active OpenGL context
		DrawableSkeleton::initializeDrawingBuffers();
	}

	// using Node = Node<CGALSurfaceMesh::Point>;
	// constructs the skeleton of the whole mesh
	// nodes could be extracted from edges but are provided for efficiency

	template <typename Graph>
	void populateSkeleton(Graph skeleton) {
		populateDrawableSkeleton(skeleton);
		std::cout << "Finished populating skeleton using boost graph."
			  << std::endl;
	}
	// adds to the skeleton one part based on the segment graph and
	// the
	// provided index
	template <typename Graph>
	void addSkeletonPart(Graph skeleton, size_t segmentIndex,
			     UndirectedGraph) {
		populateDrawableSkeleton(skeleton);
		std::cout << "Finished adding skeleton part using boost graph."
			  << std::endl;
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
	for (size_t index1 = 0; index1 < s1Nodes.size(); index1++) {
		glm::vec3 s1NodePos = s1Nodes[index1];
		for (size_t index2 = 0; index2 < s2Nodes.size(); index2++) {
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
