#ifndef SKELETON_H
#define SKELETON_H

#include <boost/graph/adjacency_list.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
#include "cgaltypedefs.h"
#include "drawableskeleton.h"
#include "undirectedgraph.h"

struct SkeletonPart {
	SkeletonGraph skeleton;
	std::vector<std::set<SkeletonGraph::vertex_descriptor>>
	    possibleJunctioNodes;  // the vector holds the segment's loops and
				   // each set holds the skeleton nodes which
				   // are close to the loop.
	size_t segmentIndex;

	SkeletonPart(SkeletonGraph s,
		     std::vector<std::set<SkeletonGraph::vertex_descriptor>> v,
		     size_t pi)
	    : skeleton(s), possibleJunctioNodes(v), segmentIndex(pi) {}
};
class Skeleton : public DrawableSkeleton {
	// public member functions
	// NOTE PointSphere should not be an argument
	// but for some reason PointSphere cannot be
	// loaded in this class but only through
	// Scene::load.Maybe because
	// GLWidget::makeCurrent() needs to be called?
	std::vector<std::optional<SkeletonPart>>
	    m_skeletonParts;  // skeleton parts of which the skeleton consists
	const UndirectedGraph &m_segmentGraph;

       public:
	Skeleton(const PointSphere &ps, const UndirectedGraph &sg)
	    : DrawableSkeleton(ps), m_segmentGraph(sg) {}
	void clear() { DrawableSkeleton::clearDrawableSkeleton(); }

	void setNumberOfSkeletonParts(size_t numOfSkeletonParts) {
		m_skeletonParts.clear();
		m_skeletonParts.resize(numOfSkeletonParts, std::nullopt);
	}
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
	void addSkeletonPart(SkeletonPart sp) {
		assert(m_skeletonParts.size() != 0);
		if (!m_skeletonParts[sp.segmentIndex]) {
			populateDrawableSkeleton(sp.skeleton);
			m_skeletonParts[sp.segmentIndex] = sp;
		}
		//		std::cout << "Finished adding skeleton part using
		//boost graph."
		//			  << std::endl;
	}
	void connectNeighbourSegments() {
		size_t numOfComputedSkeletonParts = 0;
		for (auto opt_skPart : m_skeletonParts) {
			if (opt_skPart) numOfComputedSkeletonParts++;
		}
		assert(numOfComputedSkeletonParts == m_skeletonParts.size() &&
		       m_skeletonParts.size() != 0);

		// for (auto skeletonGraph : m_skeletonParts) {
		//	for (auto setVD :
		//	     skeletonGraph.value().possibleJunctioNodes) {
		//		for (auto VD : setVD) {
		//			const auto &p = skeletonGraph.value()
		//					    .skeleton[VD]
		//					    .point;
		//			std::cout << p.x() << " " << p.y()
		//				  << " " << p.z() << std::endl;
		//		}
		//	}
		//}

		// auto junctionNodesPerSegment =
		// getPossibleJunctionPositions();
		// std::cout << junctionNodesPerSegment.size() << std::endl;
		// for (auto loopsPerSegment : junctionNodesPerSegment) {
		//	for (auto loop : loopsPerSegment) {
		//		for (auto possibleJunctioPosition : loop) {
		//			std::cout
		//			    << possibleJunctioPosition->x()
		//			    << " "
		//			    << possibleJunctioPosition->y()
		//			    << " "
		//			    << possibleJunctioPosition->z()
		//			    << std::endl;
		//		}
		//	}
		//}

		// center of mass for
		// each loop of each
		// segment
		std::vector<std::vector<Point3D>>
		    representativePositionOfEachLoopOfEachSegment =
			getCenterOfMassForEachLoop();

		// for (size_t segmentIndex = 0;
		//     segmentIndex < m_skeletonParts.size(); segmentIndex++) {
		//	std::cout << "Center of mass of" << std::endl;
		//	auto skeleton =
		//	    m_skeletonParts[segmentIndex].value().skeleton;
		//	for (std::set<SkeletonGraph::vertex_descriptor>
		//		 sOfNodes : m_skeletonParts[segmentIndex]
		//				.value()
		//				.possibleJunctioNodes) {
		//		size_t loopIndex = 0;
		//		for (SkeletonGraph::vertex_descriptor node :
		//		     sOfNodes) {
		//			auto p = skeleton[node].point;
		//			std::cout << p.x() << " " << p.y()
		//				  << " " << p.z() << " "
		//				  << std::endl;
		//		}
		//		auto p =
		//		    representativePositionOfEachLoopOfEachSegment
		//			[segmentIndex][loopIndex];
		//		std::cout << " is " << p.x() << " " << p.y()
		//			  << " " << p.z() << std::endl;
		//		loopIndex++;
		//	}
		//}

		// convert representativePositionsOfEachLoopOfEachSegment'
		// elements to optional so the next part of the algorithm does
		// not need to recalculate connections between loops already
		// assigned.
		std::vector<std::vector<std::optional<Point3D>>> queueLike(
		    representativePositionOfEachLoopOfEachSegment.size());
		for (size_t i = 0;
		     i < representativePositionOfEachLoopOfEachSegment.size();
		     i++) {
			for (auto p3d :
			     representativePositionOfEachLoopOfEachSegment[i]) {
				queueLike[i].push_back(std::make_optional(p3d));
			}
		}

		for (auto vop3d : queueLike) {
			for (auto op3d : vop3d) {
				auto p3d = op3d.value();
				// std::cout << p3d.x() << " " << p3d.y() << " "
				//	  << p3d.z() << std::endl;
			}
		}

		// assert(queueLike.size() ==
		//       representativePositionOfEachLoopOfEachSegment.size());
		// define the following structure for cleaner code
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		using SegmentIndex = size_t;
		using LoopIndex = size_t;
		struct SegmentLoopPair {
			SegmentIndex segment;
			LoopIndex loop;
			SegmentLoopPair() {}
			SegmentLoopPair(SegmentIndex si, LoopIndex li)
			    : segment(si), loop(li) {}
		};
		struct SegmentLoopConnection {
			SegmentLoopPair a;
			SegmentLoopPair b;
			SegmentLoopConnection(SegmentLoopPair slp1,
					      SegmentLoopPair slp2)
			    : a(slp1.segment, slp1.loop),
			      b(slp2.segment, slp2.loop) {}
		};

		// find which loops should be connected to which based on the
		// center of mass of each loop
		std::vector<SegmentLoopConnection>
		    connectionVector;  // Thats what I want to populate
		for (SegmentIndex s1 = 0; s1 < m_segmentGraph.num_vertices();
		     s1++) {
			std::vector<SegmentIndex> adjSegments =
			    m_segmentGraph.getAdjacentNodes(s1);
			for (LoopIndex l1 = 0; l1 < queueLike[s1].size();
			     l1++) {
				if (!queueLike[s1][l1]) continue;
				SegmentLoopPair SLP1(s1, l1);
				Point3D loop1CM = queueLike[s1][l1].value();
				Point3D loop2CM;
				std::optional<double> minDistance;
				SegmentLoopPair closestSLP;

				for (SegmentIndex adjSegmentIndex :
				     adjSegments) {
					for (size_t li2 = 0;
					     li2 <
					     queueLike[adjSegmentIndex].size();
					     li2++) {
						assert(adjSegmentIndex != s1);
						if (queueLike[adjSegmentIndex]
							     [li2]) {
							loop2CM =
							    queueLike
								[adjSegmentIndex]
								[li2]
								    .value();
							double distance = CGAL::
							    squared_distance(
								loop1CM,
								loop2CM);
							if (!minDistance
								 .has_value() ||
							    minDistance >
								distance) {
								minDistance =
								    distance;
								closestSLP =
								    SegmentLoopPair(
									adjSegmentIndex,
									li2);
							}
						}
					}
				}
				SegmentLoopConnection connection(SLP1,
								 closestSLP);
				connectionVector.push_back(connection);
				queueLike[closestSLP.segment][closestSLP.loop] =
				    std::nullopt;
				queueLike[SLP1.segment][SLP1.loop] =
				    std::nullopt;
			}
		}

		//// find actuall closest skeleton nodes in loops found above
		for (auto SLC : connectionVector) {
			auto SLP1 = SLC.a;
			auto SLP2 = SLC.b;
			auto SI1 = SLP1.segment;
			auto LI1 = SLP1.loop;
			auto SI2 = SLP2.segment;
			auto LI2 = SLP2.loop;
			auto loop1 = m_skeletonParts[SI1]
					 .value()
					 .possibleJunctioNodes[LI1];
			auto loop2 = m_skeletonParts[SI2]
					 .value()
					 .possibleJunctioNodes[LI2];
			auto skeleton1 = m_skeletonParts[SI1].value().skeleton;
			auto skeleton2 = m_skeletonParts[SI2].value().skeleton;

			std::optional<double> minDistance;
			std::pair<Point3D, Point3D> closestNodePositions;
			for (auto skeletonNodeInLoop1 : loop1) {
				for (auto skeletonNodeInLoop2 : loop2) {
					auto nodePos1 =
					    skeleton1[skeletonNodeInLoop1]
						.point;
					auto nodePos2 =
					    skeleton2[skeletonNodeInLoop2]
						.point;
					auto distance = CGAL::squared_distance(
					    nodePos1, nodePos2);
					if (!minDistance.has_value() ||
					    distance < minDistance) {
						minDistance = distance;
						closestNodePositions =
						    std::make_pair(nodePos1,
								   nodePos2);
					}
				}
			}

			auto node1 = closestNodePositions.first;
			auto node2 = closestNodePositions.second;
			// visualize connection for this segment-loop connection
			m_vertices.push_back(
			    glm::vec3(node1.x(), node1.y(), node1.z()));

			m_vertices.push_back(
			    glm::vec3(node2.x(), node2.y(), node2.z()));
		}
		updateDrawingBuffers();
	}
	std::vector<std::vector<Point3D>> getCenterOfMassForEachLoop() {
		std::vector<std::vector<Point3D>> representativePositions(
		    m_segmentGraph.num_vertices());
		for (size_t segmentIndex = 0;
		     segmentIndex < m_segmentGraph.num_vertices();
		     segmentIndex++) {
			std::vector<Point3D>
			    segmentRepresentativePositions;  // each 3D point
							     // represents a
							     // segment's loop
			const auto &vectorOfSets =
			    m_skeletonParts[segmentIndex]->possibleJunctioNodes;
			const auto &skeleton =
			    m_skeletonParts[segmentIndex]->skeleton;
			for (auto segmentLoop : vectorOfSets) {
				Point3D centerOfMassOfLoop(0, 0, 0);
				for (auto loopNode : segmentLoop) {
					centerOfMassOfLoop = Point3D(
					    skeleton[loopNode].point.x() +
						centerOfMassOfLoop.x(),
					    skeleton[loopNode].point.y() +
						centerOfMassOfLoop.y(),
					    skeleton[loopNode].point.z() +
						centerOfMassOfLoop.z());
				}
				centerOfMassOfLoop = Point3D(
				    centerOfMassOfLoop.x() / segmentLoop.size(),
				    centerOfMassOfLoop.y() / segmentLoop.size(),
				    centerOfMassOfLoop.z() /
					segmentLoop.size());
				segmentRepresentativePositions.push_back(
				    centerOfMassOfLoop);
			}
			assert(segmentRepresentativePositions.size() ==
			       vectorOfSets.size());
			representativePositions[segmentIndex] =
			    segmentRepresentativePositions;
		}
		return representativePositions;
	}

	// returns segment->loops->optional(loop node positions)
	std::vector<std::vector<std::vector<std::optional<Point3D>>>>
	getPossibleJunctionPositions() {
		std::vector<std::vector<std::vector<std::optional<Point3D>>>>
		    segments(m_segmentGraph.num_vertices());
		for (size_t segmentIndex = 0;
		     segmentIndex < m_segmentGraph.num_vertices();
		     segmentIndex++) {
			std::vector<std::vector<std::optional<Point3D>>>
			    segmentLoops;
			const auto &vectorOfSets =
			    m_skeletonParts[segmentIndex]->possibleJunctioNodes;
			const auto &skeleton =
			    m_skeletonParts[segmentIndex]->skeleton;
			for (auto segmentLoop : vectorOfSets) {
				std::vector<std::optional<Point3D>> loop;
				for (auto loopNode : segmentLoop) {
					loop.push_back(std::make_optional(
					    skeleton[loopNode].point));
				}
				segmentLoops.push_back(loop);
			}
			segments.push_back(segmentLoops);
		}
		return segments;
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
