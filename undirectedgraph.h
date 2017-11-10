#ifndef UNDIRECTEDGRAPH_H
#define UNDIRECTEDGRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <cgaltypedefs.h>

struct Node{
    size_t idx; //index to a class like CGALSurfaceMesh
    //CGALSurfaceMesh::Point pos;
};

struct UndirectedEdge{
    size_t idx1;
    size_t idx2;
};

class UndirectedGraph{
  using BoostGraph=boost::adjacency_list<boost::setS,boost::vecS,boost::undirectedS>;
    BoostGraph m_boostGraph;
  std::vector<boost::optional<BoostGraph::vertex_descriptor>> m_id_to_vd;
  std::vector<boost::optional<size_t>> m_vd_to_id;

public:
  UndirectedGraph(){}
  UndirectedGraph(size_t numberOfNodes)
  {
      m_id_to_vd.resize(numberOfNodes,boost::none);
      m_vd_to_id.resize(numberOfNodes,boost::none);
  }

  bool add_edge(size_t node1,size_t node2)
  {
      if(!m_id_to_vd[node1])
      {
          m_id_to_vd[node1]=boost::add_vertex(m_boostGraph);
          size_t vdIndex=size_t(m_id_to_vd[node1].get());
       m_vd_to_id[vdIndex]=node1;
      }
      if(!m_id_to_vd[node2])
      {
          m_id_to_vd[node2]=boost::add_vertex(m_boostGraph);
          size_t vdIndex=size_t(m_id_to_vd[node2].get());
       m_vd_to_id[vdIndex]=node2;
      }
      auto p=boost::add_edge(m_id_to_vd[node1].get(),m_id_to_vd[node2].get(),m_boostGraph);

      return p.second;
  }

 std::vector<size_t> getAdjacentNodes(size_t nodeIndex)
 {
    std::vector<size_t> adjacentNodes;
    BoostGraph::adjacency_iterator vit,vend;
    std::tie(vit,vend)=boost::adjacent_vertices(m_id_to_vd[nodeIndex].get(),m_boostGraph);

    for(auto it=vit;it!=vend;it++)
    {
        adjacentNodes.push_back(m_vd_to_id[size_t(*it)].get());
    }

    return adjacentNodes;
 }


  size_t num_vertices() const
  {
   return boost::num_vertices(m_boostGraph);
 }

  size_t num_edges()const
  {
      return boost::num_edges(m_boostGraph);
  }
};

#endif // UNDIRECTEDGRAPH_H
