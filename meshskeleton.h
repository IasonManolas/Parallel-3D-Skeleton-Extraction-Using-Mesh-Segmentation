#ifndef MESHSKELETON_H
#define MESHSKELETON_H

#include "linesegments.h"
#include "skeleton.h"
#include "undirectedgraph.h"

class MeshSkeleton {
  std::vector<boost::optional<Skeleton>> m_skeletonParts;
  const UndirectedGraph &m_segmentGraph;
  // const glm::mat4 &m_meshModelMatrix;
  LineSegments m_segmentConnectingEdges;

public:
  MeshSkeleton(UndirectedGraph &segmentGraph) : m_segmentGraph(segmentGraph) {}
  void setNumberOfSegments(size_t numberOfSegments) {
    m_skeletonParts.clear();
    m_skeletonParts.resize(numberOfSegments, boost::none);
  }
  void initialize() {
    m_segmentConnectingEdges.clear();
    m_skeletonParts.clear();
    m_segmentConnectingEdges.initializeDrawingBuffers();
  }
  void addSkeletonPart(Skeleton s,
                       boost::optional<size_t> segmentIndex = boost::none) {

    if (segmentIndex) { // skeleton part of a segment
      m_skeletonParts[segmentIndex.get()] = s;
      std::vector<size_t> adjacentSegments =
          m_segmentGraph.getAdjacentNodes(segmentIndex.get());
      for (size_t adjacentSegment : adjacentSegments) {
        if (m_skeletonParts[adjacentSegment]) // adjacent skeleton part has been
                                              // computed => connect parts
        {
          const Skeleton &adjacentSkeleton =
              m_skeletonParts[adjacentSegment].get();
          std::pair<size_t, size_t> closestNodes =
              findClosestNodes(s, adjacentSkeleton);
          auto sNodePos = s.getNodes();
          auto adjacentSegmentNodePos = adjacentSkeleton.getNodes();
          // LineSegment ls(sNodePos[closestNodes.first],
          //              adjacentSegmentNodePos[closestNodes.second]);
          m_segmentConnectingEdges.add_edge(
              sNodePos[closestNodes.first],
              adjacentSegmentNodePos[closestNodes.second]);
        }
      }

    } else { // skeleton of the whole mesh
      m_skeletonParts.push_back(s);
    }
  }

  void Draw(Shader *edgeShader, Shader *nodeShader,
            glm::mat4 meshModelMatrix) const {
    for (auto skeletonPart : m_skeletonParts)
      if (skeletonPart)
        skeletonPart.get().Draw(edgeShader, nodeShader, meshModelMatrix);
    m_segmentConnectingEdges.Draw(edgeShader);
  }

  void clear() {
    m_skeletonParts.clear();
    m_segmentConnectingEdges.clear();
  }
};

#endif // MESHSKELETON_H
