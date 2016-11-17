#ifndef MYPOLYHEDRON_H
#define MYPOLYHEDRON_H

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "meshloader.h"
#include "shader.h"
#include "material.h"

using Kernel=CGAL::Exact_predicates_inexact_constructions_kernel;
using CGALPolyhedron=CGAL::Polyhedron_3<Kernel>;
using HalfedgeDS=CGALPolyhedron::HalfedgeDS;

class MyPolyhedron: public CGALPolyhedron
{
public:
    using vd=boost::graph_traits<CGALPolyhedron>::vertex_descriptor;
    MyPolyhedron(){}
    MyPolyhedron(const std::string filename)
    {
        //load mesh
        std::tie(indices,vertices)=meshLoader::load(filename);

        setupDrawingBuffers();
        //construct Polyhedron
        buildPolyhedron();

        //Find the model matrix that normalizes the mesh
        findCenterOfMass();
        findMaxDimensionOfMesh();
        updateModelMatrix();

        printMeshInformation();
    }
    void setUniforms(Shader* shader)const
    {
        material.setUniforms(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader->programID,"model"),1,GL_FALSE,glm::value_ptr(modelMatrix));
    }

    void Draw()
    {
        setupDrawingBuffers();
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES,indices.size(),GL_UNSIGNED_INT,0);
        glBindVertexArray(0);
    }

private:
    CGALPolyhedron 			P;
    std::vector<MyVertex> 	vertices;
    std::vector<GLuint> 	indices;
    std::vector<Kernel::Vector_3> normals;
    GLuint VAO,VBO,EBO;
    glm::vec3 centerOfMass{0,0,0};
    float maxDim{1.0};
    glm::mat4 modelMatrix{1.0};
    Material material{Material(glm::vec3(0,0,0),glm::vec3(0.5,0.5,0),glm::vec3(0.6,0.6,0.5),128*0.25)};

    void buildPolyhedron()
    {
       std::vector<Kernel::Point_3> points;
       std::vector<std::vector<std::size_t>> polygons;
       for(auto const& vert:vertices)
       {
       points.push_back(Kernel::Point_3(vert.Position.x,vert.Position.y,vert.Position.z));

       }
       for(int i=0;i<indices.size();i+=3)
       {
       std::vector<std::size_t> tri{indices[i],indices[i+1],indices[i+2]};
       polygons.push_back(tri);
       }
       CGAL::Polygon_mesh_processing::orient_polygon_soup(points,polygons);
       CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points,polygons,P);
//        std::map<vd,Kernel::Vector_3> normalsMap;
//        CGAL::Polygon_mesh_processing::compute_vertex_normals(P,boost::make_assoc_property_map(normalsMap));
    }
    void findCenterOfMass()
    {
        glm::vec3 sum{0,0,0};
        for(const MyVertex& vertex:vertices)
            sum+=vertex.Position;
        uint n=vertices.size();
        centerOfMass={sum.x/n,sum.y/n,sum.z/n};
    }

    void findMaxDimensionOfMesh()
    {
        std::vector<MyVertex>::iterator first,last;
        first=vertices.begin();
        last=vertices.end();
        if(first==last) maxDim=1;

        std::vector<MyVertex>::iterator xmin,xmax,ymin,ymax,zmin,zmax;

        xmin=first;
        xmax=first;
        ymin=first;
        ymax=first;
        zmin=first;
        zmax=first;

        while(++first!=last)
        {
        if((*first).Position.x<(*xmin).Position.x)
            xmin=first;
        else if((*first).Position.x>(*xmax).Position.x)
            xmax=first;

        if((*first).Position.y<(*ymin).Position.y)
            ymin=first;
        else if((*first).Position.y>(*ymax).Position.y)
            ymax=first;

        if((*first).Position.z<(*zmin).Position.z)
            zmin=first;
        else if((*first).Position.z>(*zmax).Position.z)
            zmax=first;
        }
        maxDim=std::max(std::max((*xmax).Position.x-(*xmin).Position.x,(*ymax).Position.y-(*ymin).Position.y),(*zmax).Position.z-(*zmin).Position.z);
    }

    void updateModelMatrix()
    {
        float scaleFactor=1.0/maxDim;
        modelMatrix=glm::scale(modelMatrix,glm::vec3(scaleFactor));

        glm::vec3 translationVector(-centerOfMass);
        modelMatrix=glm::translate(modelMatrix,translationVector);
    }

    glm::mat4 getModelMatrix() const
    {
        return modelMatrix;
    }

    void printMeshInformation()const
    {
        std::cout<<"Number of vertices:"<<vertices.size()<<std::endl;
        std::cout<<"Number of faces:"<<indices.size()/3<<std::endl;
    }
    void setupDrawingBuffers()
    {
        glGenVertexArrays(1,&VAO);
        glGenBuffers(1,&VBO);
        glGenBuffers(1,&EBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER,VBO);
        glBufferData(GL_ARRAY_BUFFER,vertices.size()*sizeof(MyVertex),&vertices[0],GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,indices.size()*sizeof(GLuint),&indices[0],GL_STATIC_DRAW);

        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(MyVertex),(GLvoid*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(MyVertex),(GLvoid*)offsetof(MyVertex,Normal));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);

    }
};

#endif // MYPOLYHEDRON_H
