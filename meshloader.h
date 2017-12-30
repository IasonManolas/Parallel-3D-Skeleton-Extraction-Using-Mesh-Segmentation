#ifndef MESHLOADER_H
#define MESHLOADER_H
#include <random>
#include <tuple>
#include <vector>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <CGAL/Polygon_mesh_processing/measure.h>

#include "cgaltypedefs.h"

// void customDebug()
//{
//    std::cout << "Line " << __LINE__ << " in file " << __FILE__ << " was
//    executed" << std::endl;
//}
namespace meshLoader {
inline void processMeshAssimp(aiMesh *mesh, std::vector<uint> &indices,
			      std::vector<MyVertex> &vertices) {
	// std::cout << "Mesh has " << mesh->mNumVertices << " vertices" <<
	// std::endl;
	// Walk through each of the mesh's vertices
	for (size_t i = 0; i < mesh->mNumVertices; i++) {
		MyVertex vertex;
		glm::vec3 vector;  // We declare a placeholder vector since
				   // assimp uses its
		// own vector class that doesn't directly convert to glm's
		// vec3 class so we transfer the data to this placeholder
		// glm::vec3 first.
		// Positions
		vector.x = mesh->mVertices[i].x;
		vector.y = mesh->mVertices[i].y;
		vector.z = mesh->mVertices[i].z;
		vertex.Position = vector;
		// Normals
		vector.x = mesh->mNormals[i].x;
		vector.y = mesh->mNormals[i].y;
		vector.z = mesh->mNormals[i].z;
		vertex.Normal = glm::normalize(vector);
		vertices.push_back(vertex);
	}
	// Now wak through each of the mesh's faces (a face is a mesh its
	// triangle)
	// and retrieve the corresponding vertex indices.
	for (size_t i = 0; i < mesh->mNumFaces; i++) {
		aiFace face = mesh->mFaces[i];
		// Retrieve all indices of the face and store them in the
		// indices vector
		for (size_t j = 0; j < face.mNumIndices; j++) {
			indices.push_back(face.mIndices[j]);
		}
	}
}

inline void loadMeshAssimp(std::string path, std::vector<uint> &indices,
			   std::vector<MyVertex> &vertices) {
	Assimp::Importer importer;
	const aiScene *scene = importer.ReadFile(
	    path,
	    aiProcess_Triangulate | aiProcess_FlipUVs |
		aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices);

	if (!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE ||
	    !scene->mRootNode) {
		std::cout << "ERROR::ASSIMP::" << importer.GetErrorString()
			  << std::endl;
		//            std::cout << "ERROR::ASSIMP::" <<
		//            importer.GetImporterInfo(GetImporterCount() <<
		//            std::endl;
		return;
	}
	for (size_t i = 0; i < scene->mNumMeshes; i++) {
		processMeshAssimp(scene->mMeshes[i], indices, vertices);
	}
}

inline std::tuple<std::vector<uint>, std::vector<MyVertex>> load(
    std::string filename) {
	std::cout << "Loading " << filename << " using Assimp library."
		  << std::endl;
	std::vector<uint> indices;
	std::vector<MyVertex> vertices;
	loadMeshAssimp(filename, indices, vertices);

	return std::make_tuple(indices, vertices);
}
}
namespace meshMeasuring {

inline double findMaxDimension(std::vector<MyVertex> vertices) {
	assert(vertices.size() != 0);
	double xmax{vertices[0].Position.x}, xmin{vertices[0].Position.x},
	    ymax{vertices[0].Position.y}, ymin{vertices[0].Position.y},
	    zmax{vertices[0].Position.z}, zmin{vertices[0].Position.z};
	for (auto vertex : vertices) {
		auto p = vertex.Position;

		if (p.x > xmax)
			xmax = p.x;
		else if (p.x < xmin)
			xmin = p.x;

		if (p.y > ymax)
			ymax = p.y;
		else if (p.y < ymin)
			ymin = p.y;

		if (p.z > zmax)
			zmax = p.z;
		else if (p.z < zmin)
			zmin = p.z;
	}
	return std::max(std::max(xmax - xmin, ymax - ymin), zmax - zmin);
}
inline glm::vec3 findCenterOfMass(std::vector<MyVertex> vertices) {
	glm::vec3 centerOfMass;
	// std::cout << "Computing center of mass.." << std::endl;
	glm::vec3 sum{0, 0, 0};
	for (const MyVertex &vertex : vertices) sum += vertex.Position;
	uint n = vertices.size();
	centerOfMass = {sum.x / n, sum.y / n, sum.z / n};
	// std::cout << "Finished computing center of mass" << std::endl;
	return centerOfMass;
}

inline float findAverageEdgeLength(const CGALSurfaceMesh &M) {
	// std::cout << "Computing average edge length.." << std::endl;

	int numOfSamples = M.number_of_vertices() / 7;
	double sum = 0;
	for (int i = 0; i < numOfSamples; i++) {
		// random number in range [0,M.number_of_vertices()-1]
		int vertexIndex = rand() % M.number_of_vertices();

		CGALSurfaceMesh::Halfedge_index hi =
		    M.halfedge(CGALSurfaceMesh::vertex_index(vertexIndex));
		double edgeLength =
		    CGAL::Polygon_mesh_processing::edge_length(M.edge(hi), M);
		sum += edgeLength;
	}
	double meanEdgeLength = sum / numOfSamples;
	return meanEdgeLength;
}
}

#endif  // MESHLOADER_H
