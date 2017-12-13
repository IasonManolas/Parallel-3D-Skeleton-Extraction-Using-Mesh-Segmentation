#ifndef REFINEMENTEMBEDDING_H
#define REFINEMENTEMBEDDING_H
#include <cmath>
#include <numeric>
#include "cgaltypedefs.h"
#include "skeleton.h"

namespace SkeletonRefinement {
template <typename Point3D>
double computeNodeCenterness(Point3D node,
			     std::vector<Point3D> pointsCollapsedToNode) {
	auto lambda = [node](double sum, Point3D p) {
		double squaredDis_pToNode = std::pow(p.x() - node.x(), 2) +
					    std::pow(p.y() - node.y(), 2) +
					    std::pow(p.z() - node.z(), 2);
		return sum + squaredDis_pToNode;
	};
	double sumOfSquaredDifferences =
	    std::accumulate(pointsCollapsedToNode.begin(),
			    pointsCollapsedToNode.end(), double(0), lambda);

	return std::sqrt(sumOfSquaredDifferences /
			 pointsCollapsedToNode.size());
}

template <typename T>
inline std::vector<T> concatenate(std::vector<T> a, std::vector<T> b) {
	std::vector<T> v;
	std::copy(a.begin(), a.end(), std::back_inserter(v));
	std::copy(b.begin(), b.end(), std::back_inserter(v));
	return v;
}

template <typename Skeleton, typename TriangleMesh>
inline void executeCenternessRefinement(Skeleton& skeleton, TriangleMesh M) {
	std::cout << "Entered centerness refinement.." << std::endl;
	using Point3D = typename TriangleMesh::Point;
	using Skeleton_vertex = typename Skeleton::vertex_descriptor;
	BOOST_FOREACH (Skeleton_vertex v, boost::vertices(skeleton)) {
		const std::vector<Point3D>& collapsed_positions =
		    skeleton[v].collapsed_positions;
		double node_centerness = computeNodeCenterness(
		    skeleton[v].point, collapsed_positions);

		typename Skeleton::adjacency_iterator vbegin, vend;
		std::tie(vbegin, vend) = boost::adjacent_vertices(v, skeleton);
		for (auto it = vbegin; it != vend; it++) {
			Point3D neighbours_position = skeleton[*it].point;
			const std::vector<Point3D>&
			    neighbours_collapsed_positions =
				skeleton[*it].collapsed_positions;
			std::vector<Point3D> combined_positions =
			    concatenate(collapsed_positions,
					neighbours_collapsed_positions);
			double combined_centerness = computeNodeCenterness(
			    neighbours_position, combined_positions);
			std::cout
			    << "Combined centerness:" << combined_centerness
			    << " Node centerness:" << node_centerness
			    << std::endl;
			if (combined_centerness < 0.9 * node_centerness)
				std::cout << "vertex should be merged"
					  << std::endl;
			// merge v to neighbour
		}
	}
}

template <typename Skeleton, typename TriangleMesh>
inline void refine(Skeleton& skeleton, TriangleMesh M) {
	executeCenternessRefinement(skeleton, M);
}
}

#endif  // REFINEMENTEMBEDIING_H
