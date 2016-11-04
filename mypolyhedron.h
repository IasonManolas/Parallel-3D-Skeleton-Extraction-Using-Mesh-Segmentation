#ifndef MYPOLYHEDRON_H
#define MYPOLYHEDRON_H

//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
//#include <CGAL/Surface_mesh.h>
//#include <CGAL/property_map.h>
//#include <boost/property_map/property_map.hpp>

#include <glm/vec3.hpp>

#include "mesh.h"
#include "polyhedronbuilder.h"

//struct Vertex{
//    glm::vec3	Position;
//    glm::vec3	Normal;
//};

//using Kernel=CGAL::Exact_predicates_inexact_constructions_kernel;
using vd=boost::graph_traits<CGALPolyhedron>::vertex_descriptor;
//using fd=boost::graph_traits<CGALPolyhedron>::face_descriptor;
//using sm=CGAL::Surface_mesh<Kernel::Point_3>;
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

       std::cout<<normals.size()<<std::endl;
//        getDrawingVectors();
//       setupShaderObjects();
    }
    void Draw()
    {
//       glBindVertexArray(VAO);
//       glDrawElements(GL_TRIANGLES,size_of_facets(),GL_UNSIGNED_INT,0);
//       glBindVertexArray(0);

//       glEnableClientState(GL_VERTEX_ARRAY);
//       glEnableClientState(GL_NORMAL_ARRAY);

//       glVertexPointer(3,GL_DOUBLE,0,vertices);
//       glNormalPointer(GL_DOUBLE,0,normals);
    }

private:
    CGALPolyhedron 			P;
//    std::vector<MyVertex>	vertices;
//    std::vector<GLuint>		indices;
//    GLuint 					VAO,VBO,EBO;

//    void setupShaderObjects()
//    {

//        glGenVertexArrays(1,&VAO);
//        glGenBuffers(1,&VBO);
//        glGenBuffers(1,&EBO);

//        glBindVertexArray(VAO);

//        glBindBuffer(GL_ARRAY_BUFFER,VBO);
//        glBufferData(GL_ARRAY_BUFFER,P.size_of_vertices()*sizeof(P.Vertex),&P.vertices_begin(),GL_STATIC_DRAW);

//        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
//        glBufferData(GL_ELEMENT_ARRAY_BUFFER,P.size_of_facets()*sizeof(P.Face),&P.facets_begin(),GL_STATIC_DRAW);

//        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(GLvoid*)0);
//        glEnableVertexAttribArray(0);

//        glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(GLvoid*)offsetof(Vertex,Normal));
//        glEnableVertexAttribArray(1);

//        glBindVertexArray(0);
//    }

    void getDrawingVectors()
    {
//        for(vd verDesc:vertices(P))
//        BOOST_FOREACH(vd verDesc,vertices(P))
//            std::cout<<normals[verDesc]<<std::endl;
//        for(auto it=P.vertices_begin();it!=P.vertices_end();it++)
//        {
//            MyVertex point;
//            Kernel::Point_3 p=it->point();
//            point.Position=glm::vec3(p.x(),p.y(),p.z());



//        }
    }
};

#endif // MYPOLYHEDRON_H
