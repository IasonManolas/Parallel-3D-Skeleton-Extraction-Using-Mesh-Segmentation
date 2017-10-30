#ifndef CONNECTIVITYSURGEON_H
#define CONNECTIVITYSURGEON_H

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "cgaltypedefs.h"
#include <ctime>

/*
 ConnectivitySurgeon operates on an already contracted mesh. In short it
 collapses edges using a greedy algorithm until no faces are left.
 It implements 4 tasks which are:
 1.Edge collapse
 2.Tunnel collapse prohibition
 3.Shape cost for each edge
 4.Sampling cost for each edge
 For more details see:
 http://visgraph.cse.ust.hk/projects/skeleton/skeleton_sig08.pdf

  */
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/graph_traits.hpp>

#include "cgaltypedefs.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Kernel_traits.h>
#include <Eigen/Core>
#include <functional>

class EdgeQueue {
public:
  std::vector<size_t> edgeHeap;
  const std::vector<std::pair<double, size_t>> &m_edge_to_totalCost;
  EdgeQueue(const std::vector<std::pair<double, size_t>> &totalCosts)
      : m_edge_to_totalCost(totalCosts) {}
  bool compareEdges(size_t e0, size_t &e1) {
    return m_edge_to_totalCost[e0].first < m_edge_to_totalCost[e1].first;
  }
  void insert(size_t edgeIndex) {
    edgeHeap.push_back(edgeIndex);
    std::sort(edgeHeap.begin(), edgeHeap.end());
    edgeHeap.erase(std::unique(edgeHeap.begin(), edgeHeap.end()),
                   edgeHeap.end());
    std::sort(edgeHeap.begin(), edgeHeap.end(), [&](size_t ei0, size_t ei1) {
      return m_edge_to_totalCost[ei0].first < m_edge_to_totalCost[ei1].first;
    });
  }
  void update() {
    std::sort(edgeHeap.begin(), edgeHeap.end(), [&](size_t ei0, size_t ei1) {
      return m_edge_to_totalCost[ei0].first < m_edge_to_totalCost[ei1].first;
    });
  }
  size_t pop() {
    size_t firstElement = edgeHeap[0];
    edgeHeap.erase(edgeHeap.begin());
    return firstElement;
  }

  size_t size() { return edgeHeap.size(); }

  bool empty() { return edgeHeap.size() == 0; }
};

class ConnectivitySurgeon {
  using Matrix34d = Eigen::Matrix<double, 3, 4>;
  using Matrix4d = Eigen::Matrix4d;

  typedef boost::graph_traits<CGALSurfaceMesh>::vertex_descriptor
      vertex_descriptor;
  typedef boost::graph_traits<CGALSurfaceMesh>::face_descriptor face_descriptor;
  typedef boost::graph_traits<CGALSurfaceMesh>::edge_descriptor edge_descriptor;
  typedef boost::graph_traits<CGALSurfaceMesh>::halfedge_descriptor
      halfedge_descriptor;

  class EdgeCompareFunctor {
    std::vector<std::pair<double, size_t>> &m_edge_to_totalCost;

  public:
    EdgeCompareFunctor(std::vector<std::pair<double, size_t>> &totalCosts)
        : m_edge_to_totalCost(totalCosts) {}
  };

public:
  ConnectivitySurgeon(const CGALSurfaceMesh &M) : m_M(M) {}
  void extract_skeleton() {
    init();
    collapse();
  }

  std::vector<std::vector<size_t>> getSkeleton() {
    std::vector<std::vector<size_t>> skeletonEdges;
    for (size_t i = 0; i < is_edge_deleted.size(); i++) {
      if (is_edge_deleted[i] == false) {
        std::vector<size_t> edgeVertices{edge_to_vertex[i][0],
                                         edge_to_vertex[i][1]};
        skeletonEdges.push_back(edgeVertices);
      }
    }

    return skeletonEdges;
  }

  std::vector<std::vector<size_t>> getSkeletonMeshMapping() { return record; }

private:
  const CGALSurfaceMesh &m_M;

  std::vector<std::vector<size_t>> edge_to_face;
  std::vector<std::vector<size_t>> edge_to_vertex;
  std::vector<std::vector<size_t>> vertex_to_edge;
  std::vector<CGALSurfaceMesh::Point> vertex_to_point;
  std::vector<std::vector<size_t>> face_to_edge;

  std::vector<std::vector<size_t>> record;

  std::vector<bool> is_edge_deleted;
  std::vector<bool> is_face_deleted;
  std::vector<bool> is_vertex_deleted;

  std::vector<std::pair<double, size_t>> // pair.first is the totalCost value
                                         // and pair.second shows whether vertex
                                         // 0 or 1 of edge is source
      edge_to_totalCost;
  std::vector<std::pair<double, double>>
      edge_to_shapeCost; // pair.first is the shapeCost of  halfedge:
  // edge_to_vertex[edgeIndex][0]->edge_to_vertex[edgeIndex[1]
  // and pair.second the opposite halfedges shapeCost.
  std::vector<Matrix4d> vertex_to_Q;
  std::vector<std::pair<double, double>>
      edge_to_samplingCost; // same as shapeCost

  typedef typename boost::property_map<
      CGALSurfaceMesh, boost::halfedge_index_t>::type HalfedgeIndexMap;
  HalfedgeIndexMap hedge_id_pmap;

private:
  void init() {
    std::clock_t start;
    start = std::clock();
    initialize_edge_to_face();
    std::cout << "Time init_edge_to_face: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
    start = std::clock();
    initialize_edge_to_vertex();
    std::cout << "Time edge_to_vertex: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
    start = std::clock();
    initialize_vertex_to_edge();
    std::cout << "Time vertex_to_edge: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
    start = std::clock();
    initialize_vertex_to_point();
    std::cout << "Time vertex_to_point: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
    start = std::clock();
    initialize_face_to_edge();
    std::cout << "Time face_to_edge: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;

    start = std::clock();
    initialize_record();
    std::cout << "Time record: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;

    start = std::clock();
    initialize_is_edge_deleted();
    std::cout << "Time edge_deleted: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
    start = std::clock();
    initialize_is_face_deleted();
    std::cout << "Time face_deleted: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
    start = std::clock();
    initialize_is_vertex_deleted();
    std::cout << "Time vertex_deleted: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;

    start = std::clock();
    initialize_edge_to_totalCost();
    std::cout << "Time edge_to_totalCost: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
  }

  void initialize_edge_to_face() {
    edge_to_face.resize(m_M.number_of_edges());
    int face_id = 0;
    const CGALSurfaceMesh &hg = m_M;
    BOOST_FOREACH (face_descriptor fd, faces(hg)) {
      BOOST_FOREACH (halfedge_descriptor hd,
                     halfedges_around_face(halfedge(fd, hg), hg)) {
        size_t id = m_M.edge(hd);
        assert(id < m_M.number_of_edges());
        edge_to_face[id].push_back(face_id);
      }
      ++face_id;
    }
    // for (CGALSurfaceMesh::edge_index ei : m_M.edges()) {
    //   CGALSurfaceMesh::face_index f0(m_M.halfedge(ei, 0));
    //   CGALSurfaceMesh::face_index f1(m_M.halfedge(ei, 1));
    //   // ERROR
    //   // assert(f0 < m_M.number_of_faces() && f1 < m_M.number_of_faces());
    //   std::vector<size_t> facesOfEdge{size_t(f0), size_t(f1)};
    //   assert(size_t(ei) < m_M.number_of_edges());
    //   edge_to_face[size_t(ei)] = std::move(facesOfEdge);
    // }
  }

  void initialize_edge_to_vertex() {
    edge_to_vertex.resize(m_M.number_of_edges());
    for (CGALSurfaceMesh::edge_index ei : m_M.edges()) {
      CGALSurfaceMesh::vertex_index v0(m_M.vertex(ei, 0));
      CGALSurfaceMesh::vertex_index v1(m_M.vertex(ei, 1));
      assert(v0 < m_M.number_of_vertices() && v1 < m_M.number_of_vertices());
      assert(size_t(ei) < m_M.number_of_edges());
      std::vector<size_t> verticesOfEdge{size_t(v0), size_t(v1)};
      edge_to_vertex[size_t(ei)] = std::move(verticesOfEdge);
    }
  }

  void initialize_vertex_to_edge() {
    vertex_to_edge.resize(m_M.number_of_vertices());
    for (CGALSurfaceMesh::vertex_index vi : m_M.vertices()) {
      std::vector<size_t> edgesAroundVertex;
      // the following will get all edges whenever there exists halfedge there
      // exists its opposite and the map to the same edge.
      for (CGALSurfaceMesh::halfedge_index hei :
           CGAL::halfedges_around_source(vi, m_M)) {
        CGALSurfaceMesh::edge_index ei = m_M.edge(hei);
        assert(size_t(ei) < m_M.number_of_edges());
        edgesAroundVertex.push_back(size_t(ei));
      }
      assert(size_t(vi) < m_M.number_of_vertices());
      vertex_to_edge[size_t(vi)] = std::move(edgesAroundVertex);
    }
  }

  void initialize_vertex_to_point() {
    vertex_to_point.resize(m_M.number_of_vertices());
    for (CGALSurfaceMesh::vertex_index vi : m_M.vertices()) {
      vertex_to_point[size_t(vi)] = m_M.point(vi);
    }
  }

  void initialize_face_to_edge() {
    face_to_edge.resize(m_M.number_of_faces());
    for (CGALSurfaceMesh::face_index fi : m_M.faces()) {
      std::vector<size_t> edgesOfFace;
      for (CGALSurfaceMesh::edge_index ei :
           CGAL::edges_around_face(m_M.halfedge(fi), m_M)) {
        assert(size_t(ei) < m_M.number_of_edges());
        edgesOfFace.push_back(size_t(ei));
      }
      assert(size_t(fi) < m_M.number_of_faces());
      face_to_edge[size_t(fi)] = std::move(edgesOfFace);
    }
  }

  void initialize_record() {
    record.resize(m_M.number_of_vertices());
    for (size_t i = 0; i < record.size(); i++) {
      record[i].push_back(i);
    }
  }

  void initialize_is_edge_deleted() {
    is_edge_deleted.resize(m_M.number_of_edges(), false);
  }

  void initialize_is_face_deleted() {
    is_face_deleted.resize(m_M.number_of_faces(), false);
  }

  void initialize_is_vertex_deleted() {
    is_vertex_deleted.resize(m_M.number_of_vertices(), false);
  }

  void initialize_edge_to_totalCost() {
    edge_to_totalCost.resize(m_M.number_of_edges());

    std::clock_t start;
    start = std::clock();
    initialize_edge_to_shapeCost();
    std::cout << "Time edge_to_shapeCost: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
    start = std::clock();
    initialize_edge_to_samplingCost();
    std::cout << "Time edge_to_samplingCost: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;

    for (size_t index = 0; index < edge_to_totalCost.size(); index++) {
      double totalCost0 =
          edge_to_shapeCost[index].first + edge_to_shapeCost[index].first;
      double totalCost1 =
          edge_to_shapeCost[index].second + edge_to_shapeCost[index].second;
      double minCostOfEdge;
      size_t sourceIndex;
      if (totalCost0 < totalCost1) {
        minCostOfEdge = totalCost0;
        sourceIndex = 0;
      } else if (totalCost0 >= totalCost1) {
        minCostOfEdge = totalCost1;
        sourceIndex = 1;
      } else
        std::runtime_error("Something is wrong."); // For debugging purposes
      edge_to_totalCost[index] = std::make_pair(minCostOfEdge, sourceIndex);
    }
  }

  void initialize_edge_to_shapeCost() {
    edge_to_shapeCost.resize(m_M.number_of_edges());

    std::clock_t start;
    start = std::clock();
    initialize_vertex_to_Q();

    std::cout << "Time initialize_vertex_to_Q: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;

    start = std::clock();
    for (size_t edgeIndex = 0; edgeIndex < edge_to_shapeCost.size();
         edgeIndex++) {

      size_t i = edge_to_vertex[edgeIndex][0], j = edge_to_vertex[edgeIndex][1];

      double ij_shapeCost = compute_ShapeCostOfHE(i, j);
      double ji_shapeCost = compute_ShapeCostOfHE(j, i);

      edge_to_shapeCost[edgeIndex] = std::make_pair(ij_shapeCost, ji_shapeCost);
    }
    std::cout << "Time rest of shape_cost: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
  }

  void initialize_vertex_to_Q() {
    vertex_to_Q.resize(m_M.number_of_vertices(), Matrix4d::Zero());
    for (size_t vertexIndex = 0; vertexIndex < vertex_to_Q.size();
         vertexIndex++) {

      Matrix4d Qi = compute_Q(vertexIndex);
      vertex_to_Q[vertexIndex] = Qi;
    }
  }

  Matrix4d compute_Q(size_t vertexIndex) {
    Eigen::Matrix4d Qi = Matrix4d::Zero();

    std::vector<size_t> neighbours = getNeighbours(vertexIndex);

    for (size_t neighbourIndex : neighbours) {
      Matrix34d Kij = compute_KMatrix(vertexIndex, neighbourIndex);
      Qi += Kij.transpose() * Kij;
    }
    return Qi;
  }

  std::vector<size_t> getNeighbours(size_t ofVertex) {
    const std::vector<size_t> &edges = vertex_to_edge[ofVertex];
    std::vector<size_t> neighbours;
    for (size_t ei : edges) {
      size_t i = edge_to_vertex[ei][0];
      size_t j = edge_to_vertex[ei][1];
      if (i != ofVertex)
        neighbours.push_back(i);
      else if (j != ofVertex)
        neighbours.push_back(j);
      else
        throw std::runtime_error(
            "Something is wrong. Should have found at "
            "least on neighbour."); // NOTE for debug purposes
    }
    return neighbours;
  }

  double
  compute_ShapeCostOfHE(size_t i,
                        size_t j) const { // returns shape cost of halfedge i->j
    auto Pj = vertex_to_point[j];
    double F = compute_F(i, Pj) + compute_F(j, Pj);
    return F;
  }
  double compute_F(size_t vertexIndex, CGALSurfaceMesh::Point p) const {
    Eigen::Vector4d P(p.x(), p.y(), p.z(), 1);
    return P.transpose() * vertex_to_Q[vertexIndex] * P;
  }

  Matrix34d compute_KMatrix(size_t i, size_t j) const {
    auto Pi = vertex_to_point[i], Pj = vertex_to_point[j];
    Kernel::Vector_3 edge_vector(Pj.x() - Pi.x(), Pj.y() - Pi.y(),
                                 Pj.z() - Pi.z());
    Kernel::Vector_3 a = edge_vector / CGAL::sqrt(edge_vector.squared_length());

    Kernel::Vector_3 b =
        CGAL::cross_product(a, Kernel::Vector_3(Pi.x(), Pi.y(), Pi.z()));
    Matrix34d K;
    K << 0, -a.z(), a.y(), -b.x(), a.z(), 0, -a.x(), -b.y(), -a.y(), a.x(), 0,
        -b.z();
    return K;
  }

  void initialize_edge_to_samplingCost() {
    edge_to_samplingCost.resize(m_M.number_of_edges());

    for (size_t edgeIndex = 0; edgeIndex < edge_to_samplingCost.size();
         edgeIndex++) {
      size_t i = edge_to_vertex[edgeIndex][0], j = edge_to_vertex[edgeIndex][1];
      double ij_samplingCost = compute_samplingCostOfHE(i, j);
      double ji_samplingCost = compute_samplingCostOfHE(j, i);
      edge_to_samplingCost[edgeIndex] =
          std::make_pair(ij_samplingCost, ji_samplingCost);
    }
  }

  double compute_samplingCostOfHE(size_t i, size_t j) {
    auto Pi = vertex_to_point[i], Pj = vertex_to_point[j];
    double ijDistance = CGAL::sqrt((Kernel::Vector_3(Pi, Pj)).squared_length());

    double sumIKDistances =
        0; // sum of distances of the vertex i to all adjacent vertices of i
    for (size_t neighbourIndex : getNeighbours(i)) {
      Kernel::Vector_3 edge_vector(Pi, vertex_to_point[neighbourIndex]);
      sumIKDistances += CGAL::sqrt(edge_vector.squared_length());
    }
    double F = ijDistance * sumIKDistances;
    return F;
  }

  void initialize_queue(EdgeQueue &queue) {
    size_t numberOfEdges = m_M.number_of_edges();
    for (size_t edgeIndex = 0; edgeIndex < numberOfEdges; edgeIndex++) {
      queue.insert(edgeIndex);
    }
  }

  void collapse() {
    EdgeCompareFunctor edge_comparator(edge_to_totalCost);
    EdgeQueue edgeQueue(edge_to_totalCost);
    initialize_queue(edgeQueue);

    std::clock_t start;
    start = std::clock();
    while (!edgeQueue.empty()) {
      size_t ei = edgeQueue.pop();

      if (is_edge_deleted[ei])
        continue;
      if (edge_to_face[ei].size() == 0)
        continue;

      remove_incident_faces(ei);

      // which of the two vertex should be source so that the halfedges that is
      // deleted is the minimum
      size_t sourceIndex = edge_to_totalCost[ei].second;
      size_t vGoneIndex = edge_to_vertex[ei][sourceIndex];
      size_t vKeptIndex = edge_to_vertex[ei][size_t(!sourceIndex)];

      std::vector<size_t> neighboursOfvGone = getNeighbours(vGoneIndex);

      is_vertex_deleted[vGoneIndex] = true;
      update_record(vGoneIndex, vKeptIndex);
      delete_edge(vGoneIndex, vKeptIndex, ei);
      add_edge(vGoneIndex, vKeptIndex);

      // remove duplicate edges
      std::vector<size_t> edgesOfvKept(vertex_to_edge[vKeptIndex]);
      for (size_t i = 0; i < edgesOfvKept.size(); ++i) {
        // ei to be removed
        int ei = edgesOfvKept[i];
        for (size_t j = i + 1; j < edgesOfvKept.size(); ++j) {
          int ej = edgesOfvKept[j];
          if (is_same_edge(ei, ej) || is_edge_deleted[ei]) {
            // look for ei from p2's incident edges
            bool found;
            int ind;
            boost::tie(found, ind) = find_edge(vertex_to_edge[vKeptIndex], ei);
            if (!found) {
              continue;
            }

            // migrate faces from ei to ej
            move_face(ei, ej);

            // finally remove ei from p2
            remove_edge(vKeptIndex, ei, ind);
            break;
          }
        }
      }
      update_totalCost(vGoneIndex, vKeptIndex, neighboursOfvGone);
      // update total cost matrix of the comparator
      // edge_comparator.update_totalCostVector(edge_to_totalCost);

      edgeQueue.update();
    }
    std::cout << "Time collapse: "
              << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)
              << " ms" << std::endl;
    print_stat();
  }

  void update_totalCost(size_t vGone, size_t vKept,
                        std::vector<size_t> neighboursOfvGone) {
    update_Q(vGone, vKept); // returns a vector containing indices of the edges
                            // of whose the total cost changed

    // the following computes the total cost of an edge if its sampling cost or
    // shape cost changed.This is obviously not optimal but I prefer it since it
    // simplifies the code.

    std::set<size_t> processedEdgesSet;
    // shapeCost affected edges after the edge collapse
    for (size_t ei : vertex_to_edge[vKept]) {
      processedEdgesSet.insert(ei); // populate for next for loop
      update_edge_to_totalCost(ei);
    }

    // samplingCost affected edges after the edge collapse
    for (size_t vi : neighboursOfvGone) {
      for (size_t ei : vertex_to_edge[vi]) {
        if (processedEdgesSet.find(ei) ==
            processedEdgesSet.end()) // not already processed
        {
          processedEdgesSet.insert(ei);
          update_edge_to_totalCost(ei);
        }
      }
    }
  }

  void update_edge_to_totalCost(size_t edgeIndex) {
    size_t i = edge_to_vertex[edgeIndex][0], j = edge_to_vertex[edgeIndex][1];

    double ij_shapeCost = compute_ShapeCostOfHE(i, j);
    double ji_shapeCost = compute_ShapeCostOfHE(j, i);
    edge_to_shapeCost[edgeIndex] = std::make_pair(ij_shapeCost, ji_shapeCost);

    double ij_samplingCost = compute_samplingCostOfHE(i, j);
    double ji_samplingCost = compute_samplingCostOfHE(j, i);
    edge_to_samplingCost[edgeIndex] =
        std::make_pair(ij_samplingCost, ji_samplingCost);

    double totalCost0 = ji_shapeCost + ji_samplingCost;
    double totalCost1 = ji_shapeCost + ji_samplingCost;

    if (totalCost0 < totalCost1)
      edge_to_totalCost[edgeIndex] = std::make_pair(totalCost0, 0);
    else
      edge_to_totalCost[edgeIndex] = std::make_pair(totalCost1, 1);
  }

  void remove_incident_faces(
      size_t edgeIndex) { // removes incident faces + updates edge_to_face

    // 1)Update edge_to_face for all edges that belong to a face tha will be
    // removed
    for (size_t fid : edge_to_face[edgeIndex]) { // iterate over all faces that
                                                 // need to be removed
      is_face_deleted[fid] = true;               // mark them as deleted
      for (size_t j = 0; j < face_to_edge[fid].size();
           ++j) { // get the edges of the face that will be removed
        size_t e = face_to_edge[fid][j];
        if (e == edgeIndex) // when you fall onto the edge that will be deleted
                            // continue since for this we don't need to fix
                            // edge_to_face
          continue;
        for (size_t k = 0; k < edge_to_face[e].size();
             ++k) { // if you find an edge that will not be removed iterate over
                    // its adjacent faces and find the one which will be removed
          if (edge_to_face[e][k] == fid) {
            edge_to_face[e].erase(edge_to_face[e].begin() + k); // remove it
            break;
          }
        }
      }
    }
    // 2)Remove incident faces
    edge_to_face[edgeIndex].clear();
  }

  void update_record(size_t vGone, size_t vKept) {
    for (size_t i = 0; i < record[vGone].size(); i++) {
      record[vKept].push_back(record[vGone][i]);
    }
    record[vGone].clear();
  }

  void delete_edge(size_t vGone, size_t vKept,
                   size_t edgeIndex) { // remove edge from vertex_to_edge using
                                       // vKept and vGone
    for (size_t i = 0; i < vertex_to_edge[vGone].size(); i++) {
      if (vertex_to_edge[vGone][i] == edgeIndex) {
        vertex_to_edge[vGone].erase(vertex_to_edge[vGone].begin() + i);
        break;
      }
    }

    for (size_t i = 0; i < vertex_to_edge[vKept].size(); ++i) {
      if (vertex_to_edge[vKept][i] == edgeIndex) {
        vertex_to_edge[vKept].erase(vertex_to_edge[vKept].begin() + i);
        break;
      }
    }
    is_edge_deleted[edgeIndex] = true;
  }

  void
  add_edge(size_t vGone,
           size_t vKept) { // puts the edges pointing to vGone point to vKept
    for (size_t i = 0; i < vertex_to_edge[vGone].size(); ++i) {
      size_t edge = vertex_to_edge[vGone][i];
      if (is_edge_deleted[edge]) {
        continue;
      }
      vertex_to_edge[vKept].push_back(edge);

      // change incident vertex of edge from vGone to vKept
      for (size_t j = 0; j < edge_to_vertex[edge].size(); ++j) {
        if (edge_to_vertex[edge][j] == vGone) {
          edge_to_vertex[edge][j] = vKept;
        }
      }
    }
  }

  bool is_same_edge(int ei, int ej) {
    if (edge_to_vertex[ei][0] == edge_to_vertex[ej][0] &&
        edge_to_vertex[ei][1] == edge_to_vertex[ej][1]) {
      return true;
    }
    if (edge_to_vertex[ei][1] == edge_to_vertex[ej][0] &&
        edge_to_vertex[ei][0] == edge_to_vertex[ej][1]) {
      return true;
    }
    return false;
  }

  std::pair<bool, int> find_edge(std::vector<size_t> &edges, size_t eid) {
    for (size_t i = 0; i < edges.size(); ++i) {
      if (eid == edges[i]) {
        return std::make_pair(true, static_cast<int>(i));
      }
    }
    return std::make_pair(false, -1);
  }

  void move_face(size_t ei, size_t ej) {
    for (size_t i = 0; i < edge_to_face[ei].size(); ++i) {
      int fid = edge_to_face[ei][i];
      if (!is_face_deleted[fid]) {
        if (std::find(edge_to_face[ej].begin(), edge_to_face[ej].end(), fid) ==
            edge_to_face[ej].end()) {
          edge_to_face[ej].push_back(fid);
          for (size_t j = 0; j < face_to_edge[fid].size(); ++j) {
            if (face_to_edge[fid][j] == ei) {
              face_to_edge[fid][j] = ej;
              break;
            }
          }
        }
      }
    }
  }

  void remove_edge(int v, size_t e, int ind) {
    vertex_to_edge[v].erase(vertex_to_edge[v].begin() + ind);
    // and also remove ei from the other end point
    for (size_t i = 0; i < edge_to_vertex[e].size(); ++i) {
      int vid = edge_to_vertex[e][i];
      if (vid != v) {
        for (size_t j = 0; j < vertex_to_edge[vid].size(); ++j) {
          if (vertex_to_edge[vid][j] == e) {
            vertex_to_edge[vid].erase(vertex_to_edge[vid].begin() + j);
            break;
          }
        }
      }
    }
    is_edge_deleted[e] = true;
  }

  void update_Q(size_t vGone, size_t vKept) {
    vertex_to_Q[vKept] = vertex_to_Q[vKept] + vertex_to_Q[vGone];
  }

  void print_stat() {
    int cnt = 0;
    for (size_t i = 0; i < is_vertex_deleted.size(); ++i) {
      if (!is_vertex_deleted[i]) {
        ++cnt;
      }
    }
    std::cerr << "num of vertices " << cnt << "\n";

    cnt = 0;
    for (size_t i = 0; i < is_edge_deleted.size(); ++i) {
      if (!is_edge_deleted[i]) {
        ++cnt;
      }
    }
    std::cerr << "num of edges " << cnt << "\n";

    cnt = 0;
    for (size_t i = 0; i < is_face_deleted.size(); ++i) {
      if (!is_face_deleted[i]) {
        ++cnt;
      }
    }
    std::cerr << "num of faces " << cnt << "\n";
  }
};

#endif // CONNECTIVITYSURGEON_H
