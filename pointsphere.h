#ifndef POINTSPHERE_H
#define POINTSPHERE_H

#include <string>

#include "material.h"
#include "mypolyhedron.h"
class PointSphere : public MyPolyhedron {

public:
    void load(std::string filename, double size)
    {
	radius = size;
	resetMeshAttributes();
	std::tie(indices, vertices) = meshLoader::load(filename); //vertices contains coords & normals
	setupDrawingBuffers();

	//Find the model matrix that normalizes the mesh
	centerOfMass = meshMeasuring::findCenterOfMass(vertices);
	maxDim = meshMeasuring::findMaxDimension(vertices);
	updateModelMatrix();
	buildPolygonMesh();

	averageEdgeLength = meshMeasuring::findAverageEdgeLength(M);

	printMeshInformation();
    }
    void setIntersectingVertexIndex(int vertexIndex)
    {
	intersectingVertexIndex = vertexIndex;
    }
    void setPosition(CGAL::Point_3<CGAL::Exact_predicates_inexact_constructions_kernel> newPosition)
    {
	position = newPosition;
	std::cout << position << std::endl;
	updateModelMatrix();
    }
    void setUniforms(Shader* shader)
    {
	material.setUniforms(shader);
	glUniformMatrix4fv(glGetUniformLocation(shader->programID, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
    }
    int getIntersectionIndex()
    {
	return intersectingVertexIndex;
    }
    CGAL::Point_3<CGAL::Exact_predicates_inexact_constructions_kernel> getPosition()
    {
	return position;
    }

private:
    double radius;
    int intersectingVertexIndex{ 0 };
    CGAL::Point_3<CGAL::Exact_predicates_inexact_constructions_kernel> position;
    glm::vec3 translationPoint{ 0 };
    Material material{ Material(glm::vec3(0, 0, 0), glm::vec3(1, 0, 0), glm::vec3(1, 0, 0), 128 * 0.25) };

    void updateModelMatrix() // Model space -> World space
    {
	modelMatrix = glm::mat4{ 1.0 };

	//first scale and than translate to the desired position. Transforms apply from last to first
	glm::vec3 translationVector(-centerOfMass + glm::vec3(position.x(), position.y(), position.z()));
	modelMatrix = glm::translate(modelMatrix, translationVector);

	float scaleFactor = radius / averageEdgeLength;
	modelMatrix = glm::scale(modelMatrix, glm::vec3(scaleFactor));
    }
};
#endif POINTSPHERE_H
