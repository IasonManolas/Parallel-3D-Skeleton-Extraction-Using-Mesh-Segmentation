#ifndef DRAWABLESKELETON_H
#define DRAWABLESKELETON_H

#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <vector>
#include "pointsphere.h"
#include "shader.h"

class DrawableSkeleton {
       public:
	void Draw(Shader *edgeShader, Shader *nodeShader,
		  glm::mat4 meshModelMatrix) {
		edgeShader->Use();
		GLint modelLoc =
		    glGetUniformLocation(edgeShader->programID, "model");
		if (modelLoc == -1)
			std::cout << "Could not find model(matrix) uniform."
				  << std::endl;
		else
			glUniformMatrix4fv(modelLoc, 1, GL_FALSE,
					   glm::value_ptr(meshModelMatrix));
		drawEdges();
		nodeShader->Use();
		drawNodes(nodeShader, meshModelMatrix);
	}
	// void setMaterial(const Material &value) { material = value; }
       private:
	void drawEdges() {
		glBindVertexArray(VAO);
		glDrawArrays(GL_LINES, 0, m_vertices.size());
		// glDrawElements(GL_LINES, m_indices.size(), GL_UNSIGNED_INT,
		// 0);
		// if (!m_vertices.empty())
		//  for (auto vertex : m_vertices)
		//    std::cout << glm::to_string(vertex) << std::endl;

		glBindVertexArray(0);
	}
	void drawNodes(Shader *shader, glm::mat4 meshModelMatrix) {
		shader->Use();
		for (PointSphere ps : m_nodeDrawingVector) {
			ps.handle_drawing(shader, meshModelMatrix);
		}
	}

       protected:
	std::vector<PointSphere> m_nodeDrawingVector;
	std::vector<glm::vec3> m_vertices;
	std::vector<uint> m_indices;
	uint VAO, VBO, EBO;
	const PointSphere &m_PS;
	// Material material{glm::vec3(0.05, 0.05, 0.05),
	//                  glm::vec3(0.5, 0.5, 0), // should be static
	//                  constexpr
	//                  glm::vec3(0.6, 0.6, 0.5), 128 * 0.25};
       protected:
	DrawableSkeleton(const PointSphere &ps) : m_PS(ps) {}
	void setUniforms() {}
	void clearDrawableSkeleton() {
		m_nodeDrawingVector.clear();
		m_vertices.clear();
		m_indices.clear();
		updateMeshBuffers();
	}

	void updateMeshBuffers() {
		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER,
			     (m_vertices.size() + 1) * sizeof(glm::vec3),
			     &m_vertices[0], GL_DYNAMIC_DRAW);

		// glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		// glBufferData(GL_ELEMENT_ARRAY_BUFFER,
		// 	     m_indices.size() * sizeof(GLuint), &m_indices[0],
		// 	     GL_DYNAMIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
				      sizeof(glm::vec3), (GLvoid *)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
				      sizeof(glm::vec3),
				      (GLvoid *)offsetof(glm::vec3, y));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE,
				      sizeof(glm::vec3),
				      (GLvoid *)offsetof(glm::vec3, z));
		glEnableVertexAttribArray(2);

		glBindVertexArray(0);
	}
	void initializeDrawingBuffers() {
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);
		// glGenBuffers(1, &EBO);

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER,
			     (m_vertices.size() + 1) * sizeof(glm::vec3),
			     &m_vertices[0], GL_DYNAMIC_DRAW);

		// glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		// glBufferData(GL_ELEMENT_ARRAY_BUFFER,
		// 	     m_indices.size() * sizeof(GLuint), &m_indices[0],
		// 	     GL_DYNAMIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
				      sizeof(glm::vec3), (GLvoid *)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
				      sizeof(glm::vec3),
				      (GLvoid *)offsetof(glm::vec3, y));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE,
				      sizeof(glm::vec3),
				      (GLvoid *)offsetof(glm::vec3, z));
		glEnableVertexAttribArray(2);

		glBindVertexArray(0);
		// std::cout << "printDebugInformation was called in
		// "<<__func__<<std::endl;
		//     // printDebugInformation();
		//
	}
	void printDebugInformation() const {
		glBindVertexArray(VAO);
		for (int i = 0; i <= 1; i++) {
			GLint ival;
			GLvoid *pval;

			glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED,
					    &ival);
			printf("Attr %d: ENABLED    		= %d\n", i,
			       ival);
			glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_SIZE,
					    &ival);
			printf("Attr %d: SIZE		= %d\n", i, ival);
			glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_STRIDE,
					    &ival);
			printf("Attr %d: STRIDE		= %d\n", i, ival);
			glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_TYPE,
					    &ival);
			printf("Attr %d: TYPE		= 0x%x\n", i, ival);
			glGetVertexAttribiv(
			    i, GL_VERTEX_ATTRIB_ARRAY_NORMALIZED, &ival);
			printf("Attr %d: NORMALIZED		= %d\n", i,
			       ival);
			glGetVertexAttribiv(
			    i, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &ival);
			printf("Attr %d: BUFFER		= %d\n", i, ival);
			glGetVertexAttribPointerv(
			    i, GL_VERTEX_ATTRIB_ARRAY_POINTER, &pval);
			printf("Attr %d: POINTER		= %d\n", i,
			       ival);
		}
		// Also print the numeric handle of the VAO:
		printf("VAO = %ld\n", long(VAO));
	}
};
#endif  // DRAWABLESKELETON_H
