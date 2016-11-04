#ifndef MYPOLYHEDRON_H
#define MYPOLYHEDRON_H

#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#include <glm/vec3.hpp>

#include "mesh.h"
#include "polyhedronbuilder.h"

using vd=boost::graph_traits<CGALPolyhedron>::vertex_descriptor;
class MyPolyhedron: public CGALPolyhedron
{
public:
    MyPolyhedron(){}
    MyPolyhedron(const Mesh& mesh)
    {

       PolyhedronBuilder<HalfedgeDS> builder(mesh);
       P.delegate(builder);

        std::map<vd,Kernel::Vector_3> normals;
        CGAL::Polygon_mesh_processing::compute_vertex_normals(P,boost::make_assoc_property_map(normals));

    }
private:
    CGALPolyhedron 			P;
};

#endif // MYPOLYHEDRON_H
