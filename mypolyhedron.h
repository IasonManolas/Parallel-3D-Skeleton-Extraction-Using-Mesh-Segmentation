#ifndef MYPOLYHEDRON_H
#define MYPOLYHEDRON_H

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

#include <glm/vec3.hpp>

#include "mesh.h"
#include "polyhedronbuilder.h"


class MyPolyhedron: public CGALPolyhedron
{
public:
    using vd=boost::graph_traits<CGALPolyhedron>::vertex_descriptor;
    MyPolyhedron(){}
    MyPolyhedron(const Mesh& mesh)
    {
       std::vector<Kernel::Point_3> points;
       std::vector<std::vector<std::size_t>> polygons;
       for(auto const& vert:mesh.vertices)
       {
           points.push_back(Kernel::Point_3(vert.Position.x,vert.Position.y,vert.Position.z));

       }
       for(int i=0;i<mesh.indices.size();i+=3)
       {
           std::vector<std::size_t> tri{mesh.indices[i],mesh.indices[i+1],mesh.indices[i+2]};
           polygons.push_back(tri);

       }
       CGAL::Polygon_mesh_processing::orient_polygon_soup(points,polygons);
       CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points,polygons,P);
//       std::cout<<points.size()<<std::endl;
//       std::cout<<polygons.size()<<std::endl;
        std::cout<<bool(P.is_valid())<<std::endl;
        std::map<vd,Kernel::Vector_3> normals;
        CGAL::Polygon_mesh_processing::compute_vertex_normals(P,boost::make_assoc_property_map(normals));

        vertices=points;
        triangles=polygons;
        this->normals=normals;
        std::cout<<normals.size()<<std::endl;
    }
private:
    CGALPolyhedron 			P;
    std::vector<Kernel::Point_3> vertices;
    std::map<vd,Kernel::Vector_3> normals;
    std::vector<std::vector<std::size_t>> triangles;

    GLuint VAO,VBO,EBO;
//    void setupDrawingBuffers()
//    {
//        glGenVertexArrays(1,&VAO);
//        glGenBuffers(1,&VBO);
//        glGenBuffers(1,&EBO);

//        glBindVertexArray(VAO);

//        glBindBuffer(GL_ARRAY_BUFFER,VBO);
//        glBufferData(GL_ARRAY_BUFFER,P.size_of_vertices()*sizeof(sizeof(Kernel::Point_3)),P.vertices_begin(),GL_STATIC_DRAW);

//        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
//        glBufferData(GL_ELEMENT_ARRAY_BUFFER,indices.size()*sizeof(GLuint),&indices[0],GL_STATIC_DRAW);

//        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(MyVertex),(GLvoid*)0);
//        glEnableVertexAttribArray(0);

//        glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(MyVertex),(GLvoid*)offsetof(MyVertex,Normal));
//        glEnableVertexAttribArray(1);

//        glBindVertexArray(0);

//    }
};

#endif // MYPOLYHEDRON_H
