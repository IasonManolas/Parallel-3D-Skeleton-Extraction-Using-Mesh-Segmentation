#ifndef REFINEMENTEMBEDDING_H
#define REFINEMENTEMBEDDING_H
#include <cmath>
#include <numeric>
#include "cgaltypedefs.h"
#include "skeleton.h"

template <typename Point>
double computeNodeCenterness(Point node,
			     std::vector<Point> pointsCollapsedToNode) {
	auto lambda = [node](double sum, Point p) {
		double squaredDis_pToNode = std::pow(p.x() - node.x(), 2) +
					    std::pow(p.y() - node.y(), 2) +
					    std::pow(p.z() - node.z(), 2);
		return sum + squaredDis_pToNode;
	};
	double sumOfSquaredDifferences =
	    std::accumulate(pointsCollapsedToNode.begin(),
			    pointsCollapsedToNode.end(), double(0),lambda);

	return std::sqrt(sumOfSquaredDifferences /
			 pointsCollapsedToNode.size());
}

#endif  // REFINEMENTEMBEDIING_H
