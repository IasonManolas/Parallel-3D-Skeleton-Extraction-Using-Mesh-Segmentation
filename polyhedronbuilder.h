#ifndef POLYHEDRONBUILDER_CGAL_H
#define POLYHEDRONBUILDER_CGAL_H

#include <CGAL/Polyhedron_3.h>
//#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include "mesh.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel	Kernel;
typedef CGAL::Polyhedron_3<Kernel>							Polyhedron;
typedef Polyhedron::HalfedgeDS 								HalfedgeDS;

template<class HDS>

class  PolyhedronBuilder:public CGAL::Modifier_base<HDS>
{
public:
    Mesh 		M;

    PolyhedronBuilder(){}
    PolyhedronBuilder(const Mesh& model):M{model}{}
    void operator()(HDS& hds)
    {
        typedef typename HDS::Vertex	Vertex;
        typedef typename Vertex::Point 	Point;

        CGAL::Polyhedron_incremental_builder_3<HDS> B(hds,true);
        B.begin_surface(M.vertices.size(),M.indices.size()/3);

        //add vertices
        for(const auto& vert:M.vertices)
            B.add_vertex(Point(vert.Position.x,vert.Position.y,vert.Position.z));

        //add faces
        for(int i=0;i<M.indices.size()/3;i+=3)
        {
            B.begin_facet();
            B.add_vertex_to_facet(M.indices[i]);
            B.add_vertex_to_facet(M.indices[i+1]);
            B.add_vertex_to_facet(M.indices[i+2]);
            B.end_facet();
        }
        B.end_surface();

    }
};

#endif // POLYHEDRALSURFACE_CGAL_H
