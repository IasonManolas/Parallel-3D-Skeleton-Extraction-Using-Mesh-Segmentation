#ifndef MESHSKELETON_H
#define MESHSKELETON_H

#include "skeleton.h"
#include "undirectedgraph.h"

class MeshSkeleton{
    std::vector<Skeleton> m_skeletonParts;
    UndirectedGraph m_segmentGraph;
    std::vector<LineSegments> m_segmentConnectingEdges;

public:
    addSkeleton(Skeleton);

};

#endif // MESHSKELETON_H
