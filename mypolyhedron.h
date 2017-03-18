#ifndef MYPOLYHEDRON_H
#define MYPOLYHEDRON_H

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
//segmentation
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>

//ray shooting
#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
//#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <boost/optional/optional_io.hpp>
//#include <QOpenGLContext>

struct MyVertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec3 Color{glm::vec3(0,0,0)};
};

#include "meshloader.h"
//#include "meshmeasuring.h"
#include "shader.h"
#include "material.h"

using Kernel=CGAL::Simple_cartesian<double>;
using CGALPolyhedron=CGAL::Polyhedron_3<Kernel>;
using CGALSurfaceMesh=CGAL::Surface_mesh<Kernel::Point_3>;
using HalfedgeDS=CGALPolyhedron::HalfedgeDS;

class MyPolyhedron
{
public:
    using vd=boost::graph_traits<CGALPolyhedron>::vertex_descriptor;
    MyPolyhedron(){}
    void load(std::string filename)
    {
        indices.clear();
        vertices.clear();
        centerOfMass=glm::vec3(1.0);
        modelMatrix=glm::mat4(1.0);
        maxDim=1;
        bool hasNormals;
        std::tie(indices,vertices,hasNormals)=meshLoader::load(filename); //vertices contains coords & normals
        vertices[0].Color=glm::vec3(1,0,0);
//        std::cout<<vertices[0].Colored<<endl;
        vertices[1].Color=glm::vec3(1,0,0);
        vertices[2].Color=glm::vec3(1,0,0);
        setupDrawingBuffers();
        //Find the model matrix that normalizes the mesh
        centerOfMass=meshMeasuring::findCenterOfMass(vertices);
        maxDim=meshMeasuring::findMaxDimension(vertices);
        updateModelMatrix();

//        buildPolyhedron();
        buildPolygonMesh();
//        meshSegmentation();

        printMeshInformation();
     }

    void printDebugInformation() const
    {
        glBindVertexArray(VAO);
        for ( int i = 0; i <= 1; i++ )
        {
            GLint   ival;
            GLvoid *pval;

            glGetVertexAttribiv       ( i, GL_VERTEX_ATTRIB_ARRAY_ENABLED       , &ival );printf( "Attr %d: ENABLED    		= %d\n", i, ival );
            glGetVertexAttribiv       ( i, GL_VERTEX_ATTRIB_ARRAY_SIZE          , &ival );printf( "Attr %d: SIZE		= %d\n", i, ival );
            glGetVertexAttribiv       ( i, GL_VERTEX_ATTRIB_ARRAY_STRIDE        , &ival );printf( "Attr %d: STRIDE		= %d\n", i, ival );
            glGetVertexAttribiv       ( i, GL_VERTEX_ATTRIB_ARRAY_TYPE          , &ival );printf( "Attr %d: TYPE		= 0x%x\n",i, ival );
            glGetVertexAttribiv       ( i, GL_VERTEX_ATTRIB_ARRAY_NORMALIZED    , &ival );printf( "Attr %d: NORMALIZED		= %d\n", i, ival );
            glGetVertexAttribiv       ( i, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &ival );printf( "Attr %d: BUFFER		= %d\n", i, ival );
            glGetVertexAttribPointerv ( i, GL_VERTEX_ATTRIB_ARRAY_POINTER       , &pval );printf( "Attr %d: POINTER		= %p\n", i, ival );
        }
        // Also print the numeric handle of the VAO:
        printf( "VAO = %ld\n", long( VAO ) );

    }
    void setUniforms(Shader* shader)const
    {
        material.setUniforms(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader->programID,"model"),1,GL_FALSE,glm::value_ptr(modelMatrix));
    }

    void Draw()
    {
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES,indices.size(),GL_UNSIGNED_INT,0);
        glBindVertexArray(0);
    }

    bool intersects(Kernel::Ray_3 ray)
    {

        typedef CGAL::AABB_face_graph_triangle_primitive<CGALSurfaceMesh> Primitive;
        typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
        typedef CGAL::AABB_tree<Traits> Tree;

        Tree tree(faces(M).first, faces(M).second,M);
        typedef boost::optional<Tree::Intersection_and_primitive_id<Kernel::Ray_3>::Type> Ray_intersection;
        boost::optional<int> a=3;

        Ray_intersection intersection=tree.first_intersection(ray);
//		if(tree.do_intersect(ray))
        if(intersection)
        {
            std::cout<<"Intersection(s) found!"<<std::endl;
            if(boost::get<CGAL::SM_Face_index>(&(intersection->second))){
              const CGAL::SM_Face_index* p =  boost::get<CGAL::SM_Face_index>(&(intersection->second) );

              std::cout <<  *p << std::endl;
            }
//            boost::optional<Primitive> bo_p=tree.first_intersected_primitive(ray);
//            if(bo_p_id) std::cout<<"INTERSECTS"<<std::endl;
//            std::cout<<bo_p_id<<std::endl;
//            Tree::Primitive_id p_id=bo_p_id.get();
//            std::cout<<p_id<<std::endl;

            return true;
        }
        std::cout<<"No Intersection found."<<std::endl;
        return false;

    }

private:
    CGALPolyhedron 			P;
    CGALSurfaceMesh			M;
    std::vector<MyVertex> 	vertices;
    std::vector<GLuint> 	indices;
//    std::vector<Kernel::Vector_3> normals;
    GLuint VAO,VBO,EBO;
    glm::vec3 centerOfMass{0,0,0};
    float maxDim{1.0};
    glm::mat4 modelMatrix{1.0};
    Material material{Material(glm::vec3(0,0,0),glm::vec3(0.5,0.5,0),glm::vec3(0.6,0.6,0.5),128*0.25)};

    int meshSegmentation()
    {
//        CGALPolyhedron mesh;
//        std::ifstream input("../../cgal-demos/examples/Surface_mesh_segmentation/data/cactus.off");
//        if ( !input || !(input >> mesh) || mesh.empty() ) {
//            std::cerr << "Not a valid off file." << std::endl;
//            return EXIT_FAILURE;
//        }
        std::cout<<"Segmenting Mesh.."<<std::endl;
        using Facet_int_map=std::map<CGALPolyhedron::Facet_const_handle,std::size_t>;
        Facet_int_map internal_segment_map;
        boost::associative_property_map<Facet_int_map> segment_property_map(internal_segment_map);
        std::size_t number_of_segments=CGAL::segmentation_via_sdf_values(P
                                                                         ,segment_property_map);
        std::cout<<"Number of segments: "<<number_of_segments<<std::endl;
        std::cout<<"Segmentation finished."<<std::endl;
    }


    void buildPolygonMesh()
    {
        M.clear();
        std::cout<<"Building CGALPolygonMesh.."<<std::endl;
       std::vector<Kernel::Point_3> points;
       std::vector<std::vector<std::size_t>> polygons;

       for(auto const& vert:vertices)
       {
       points.push_back(Kernel::Point_3((vert.Position.x-centerOfMass.x)/maxDim,(vert.Position.y-centerOfMass.y)/maxDim,(vert.Position.z-centerOfMass.z)/maxDim));

       }
       for(int i=0;i<indices.size();i+=3)
       {
       std::vector<std::size_t> tri{indices[i],indices[i+1],indices[i+2]};
       polygons.push_back(tri);
       }
//       CGAL::Polygon_mesh_processing::orient_polygon_soup(points,polygons);
       std::cout<<"is polygon soup polygon mesh:"<<CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(polygons)<<std::endl;
       CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points,polygons,M);
//       if (CGAL::is_closed(M) && (!CGAL::Polygon_mesh_processing::is_outward_oriented(M)))
//         CGAL::Polygon_mesh_processing::reverse_face_orientations(M);


//        std::map<vd,Kernel::Vector_3> normalsMap;
//        CGAL::Polygon_mesh_processing::compute_vertex_normals(P,boost::make_assoc_property_map(normalsMap));
        std::cout<<"Finished building Polygon Mesh."<<std::endl;
    }
    void buildPolyhedron()
    {
        P.clear();
        std::cout<<"Building Polyhedron.."<<std::endl;
       std::vector<Kernel::Point_3> points;
       std::vector<std::vector<std::size_t>> polygons;

       for(auto const& vert:vertices)
       {
       points.push_back(Kernel::Point_3((vert.Position.x-centerOfMass.x)/maxDim,(vert.Position.y-centerOfMass.y)/maxDim,(vert.Position.z-centerOfMass.z)/maxDim));

       }
       for(int i=0;i<indices.size();i+=3)
       {
       std::vector<std::size_t> tri{indices[i],indices[i+1],indices[i+2]};
       polygons.push_back(tri);
       }
       CGAL::Polygon_mesh_processing::orient_polygon_soup(points,polygons);
       std::cout<<"is polygon soup polygon mesh:"<<CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(polygons)<<std::endl;
       CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points,polygons,P);
       if (CGAL::is_closed(P) && (!CGAL::Polygon_mesh_processing::is_outward_oriented(P)))
         CGAL::Polygon_mesh_processing::reverse_face_orientations(P);


//        std::map<vd,Kernel::Vector_3> normalsMap;
//        CGAL::Polygon_mesh_processing::compute_vertex_normals(P,boost::make_assoc_property_map(normalsMap));
        std::cout<<"Finished building Polyhedron."<<std::endl;
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
        std::cout<<"Entering setupDrawingBuffers().."<<std::endl;

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

        glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(MyVertex),(GLvoid*)offsetof(MyVertex,Color));
        glEnableVertexAttribArray(2);

         glBindVertexArray(0);
//   std::cout<<"printDebugInformation was called in :"<<__func__<<std::endl;
//        printDebugInformation();

        std::cout<<"Exiting setupDrawingBuffers().."<<std::endl;
    }
};

#endif // MYPOLYHEDRON_H
