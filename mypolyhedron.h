#ifndef MYPOLYHEDRON_H
#define MYPOLYHEDRON_H

//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Polyhedron_3.h>
//#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#include <glm/vec3.hpp>

#include "mesh.h"
#include "polyhedronbuilder.h"

//struct Vertex{
//    glm::vec3	Position;
//    glm::vec3	Normal;
//};

//using Kernel=CGAL::Exact_predicates_inexact_constructions_kernel;


class MyPolyhedron: public CGALPolyhedron
{
public:
    MyPolyhedron(){}
    MyPolyhedron(const Mesh& mesh)
    {
       PolyhedronBuilder<HalfedgeDS> builder(mesh);
       P.delegate(builder);

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
    std::vector<Vertex>		vertices;
    std::vector<GLuint>	indices;
    GLuint 					VAO,VBO,EBO;

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

//    void getDrawingVectors()
//    {

//    }
};

#endif // MYPOLYHEDRON_H
