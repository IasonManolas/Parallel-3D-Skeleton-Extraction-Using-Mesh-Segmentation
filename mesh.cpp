#include "mypolyhedron.h"

MyPolyhedron::MyPolyhedron():S()
{
    for (auto& color : colorPalette) {
	color /= 255.0;
    }
}

void MyPolyhedron::resetMeshAttributes()
{
    indices.clear();
    vertices.clear();
    centerOfMass = glm::vec3(1.0);
    modelMatrix = glm::mat4(1.0);
}

void MyPolyhedron::load(std::string filename)
{
    resetMeshAttributes();
    std::tie(indices, vertices) = meshLoader::load(filename); // vertices contains coords & normals

    setupDrawingBuffers();
    // Find the model matrix that normalizes the mesh
    centerOfMass = meshMeasuring::findCenterOfMass(vertices);
    maxDim = meshMeasuring::findMaxDimension(vertices);
    normalizeMeshViaModelMatrix();
    buildPolygonMesh();
    averageEdgeLength = meshMeasuring::findAverageEdgeLength(M);

    S = MeshSkeleton(M);

    printMeshInformation();
}

void MyPolyhedron::assignSegmentColors()
{
    for (CGALSurfaceMesh::Face_iterator it = M.faces_begin();
	 it != M.faces_end(); it++) {
	CGALSurfaceMesh::Face_index fIndex(*it);
	auto r = M.vertices_around_face(M.halfedge(fIndex));
	auto vbegin = r.begin();
	std::size_t segmentID = segment_property_map[fIndex];
	glm::vec3 color = colorPalette[segmentID];
	for (auto vbegin = r.begin(); vbegin != r.end(); vbegin++) {
	    std::size_t vIndex = std::size_t(*vbegin);
	    vertices[vIndex].Color = color;
	}
    }
}

void MyPolyhedron::constructSegmentMap()
{
    std::cout << "Segmenting mesh.." << std::endl;
    // create a property-map ean are you sure the edge sean are you sure the
    // edge shared by two triangles is not duplicatehared by two triangles is
    // not duplicatefor segment-id
    segment_property_map = M.add_property_map<face_descriptor, std::size_t>("f:sid").first;
    ;
    segmentsComputed = true;
    // segment the mesh using default parameters for number of levels, and
    // smoothing lambda
    // Any other scalar values can be used instead of using SDF values computed
    // using the CGAL function
    std::size_t number_of_segments = CGAL::segmentation_via_sdf_values(M, segment_property_map);

    std::cout << "Number of segments: " << number_of_segments << std::endl;
}

void MyPolyhedron::deformSegment(const int segmentIndex, char deltaSign)
{
    float deformationFactor;
    if (deltaSign == '+')
	deformationFactor = 0.01;
    else
	deformationFactor = -0.01;

    for (CGALSurfaceMesh::Face_iterator it = M.faces_begin();
	 it != M.faces_end(); it++) {
	CGALSurfaceMesh::Face_index fIndex(*it);
	if (segment_property_map[fIndex] == segmentIndex) {
	    auto r = M.vertices_around_face(M.halfedge(fIndex));
	    auto vbegin = r.begin();
	    for (auto vbegin = r.begin(); vbegin != r.end(); vbegin++) {
		// update M's position
		// vbegin.
		// update vertices vector position
		std::size_t vIndex = std::size_t(*vbegin);
		vertices[vIndex].Position += deformationFactor * vertices[vIndex].Normal;
	    }
	}
    }
    updateMeshBuffers();
}

void MyPolyhedron::printDebugInformation() const
{
    glBindVertexArray(VAO);
    for (int i = 0; i <= 1; i++) {
	GLint ival;
	GLvoid* pval;

	glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &ival);
	printf("Attr %d: ENABLED    		= %d\n", i, ival);
	glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_SIZE, &ival);
	printf("Attr %d: SIZE		= %d\n", i, ival);
	glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_STRIDE, &ival);
	printf("Attr %d: STRIDE		= %d\n", i, ival);
	glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_TYPE, &ival);
	printf("Attr %d: TYPE		= 0x%x\n", i, ival);
	glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_NORMALIZED, &ival);
	printf("Attr %d: NORMALIZED		= %d\n", i, ival);
	glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &ival);
	printf("Attr %d: BUFFER		= %d\n", i, ival);
	glGetVertexAttribPointerv(i, GL_VERTEX_ATTRIB_ARRAY_POINTER, &pval);
	printf("Attr %d: POINTER		= %p\n", i, ival);
    }
    // Also print the numeric handle of the VAO:
    printf("VAO = %ld\n", long(VAO));
}

void MyPolyhedron::setUniforms(Shader* shader)
{
    modelShader = shader; // maybe this is not good design?
    material.setUniforms(modelShader);
    glUniformMatrix4fv(glGetUniformLocation(modelShader->programID, "model"), 1,
	GL_FALSE, glm::value_ptr(modelMatrix));
    // setIntersectingTriangleUniform((size_t)intersectingTriangleIndex);
    //
}

void MyPolyhedron::setIntersectingTriangleUniform(int faceIndex)
{
    int location = glGetUniformLocation(modelShader->programID, "intersectingFace");
    glUniform1i(location, int(faceIndex));
}

void MyPolyhedron::Draw()
{
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}
std::size_t MyPolyhedron::getSegmentIndex(const CGALSurfaceMesh::Face_index face_index)
{
    return segment_property_map[face_index];
}

inline Ray_intersection MyPolyhedron::intersects(Kernel::Ray_3 ray)
{

    Tree tree(faces(M).first, faces(M).second, M);

    Ray_intersection intersection = tree.first_intersection(ray);
    //    if (intersection) {
    //        //      std::cout << "Intersection(s) found!" << std::endl;
    //            //      if (boost::get<Point>(&(intersection->first))) {
    //                //        Point *p = boost::get<Point>(&(intersection->first));
    //                    //        intersectingTriangleIndex = intersection->second;
    //                        //        intersectionIndex = findClosestVertex(
    //                            //            *p, intersectingTriangleIndex); // returns vertices[someIndex]
    //                                //        glm::vec3 intersectionPos = glm::vec3(
    //                                    //            modelMatrix * glm::vec4(vertices[intersectionIndex].Position,
    //                                        //            1.0));
    //                                            //        intersectionPosition =
    //                                                //            Point(intersectionPos.x, intersectionPos.y,
    //                                                    //            intersectionPos.z);
    //                                                        //      }
    //                                                            //    } else {
    //                                                                //      std::cout << "No Intersection found." << std::endl;
    //                                                                    //    }
    //                                                                          }
    //
    //                                                            }
    //                                )
    //                        )
    //            }
    //    }
    return intersection;
}

int MyPolyhedron::findClosestVertex(Point intersectionPoint,
    CGALSurfaceMesh::Face_index intersectingFaceIndex)
{
    CGALSurfaceMesh::halfedge_index beginHalfedge = M.halfedge(intersectingFaceIndex);
    CGALSurfaceMesh::halfedge_index h = beginHalfedge;
    CGALSurfaceMesh::vertex_index vIndex = M.source(h);
    Kernel::Point_3 vertex = M.point(vIndex);
    double minDistance = CGAL::squared_distance(vertex, intersectionPoint);
    int closestVertexIndex = (int)vIndex;
    do {
	std::cout << h << std::endl;
	vIndex = M.source(h);
	Kernel::Point_3 vertex = M.point(vIndex);
	double distance = CGAL::squared_distance(vertex, intersectionPoint);
	if (distance < minDistance) {

	    minDistance = distance;
	    vIndex = M.source(h);
	    closestVertexIndex = int(vIndex);
	}
	h = M.next(h);
    } while (h != beginHalfedge);
    return closestVertexIndex;
}

void MyPolyhedron::buildPolygonMesh()
{
    M.clear();
    std::cout << "Building CGALPolygonMesh.." << std::endl;
    std::vector<Kernel::Point_3> points;
    std::vector<std::vector<std::size_t> > polygons;

    for (auto const& vert : vertices) {
	points.push_back(
	    Kernel::Point_3((vert.Position.x - centerOfMass.x) / maxDim,
		(vert.Position.y - centerOfMass.y) / maxDim,
		(vert.Position.z - centerOfMass.z) / maxDim));
    }
    for (size_t i = 0; i < indices.size(); i += 3) {
	std::vector<std::size_t> tri{ indices[i], indices[i + 1], indices[i + 2] };
	polygons.push_back(tri);
    }
    //               CGAL::Polygon_mesh_processing::orient_polygon_soup(points,polygons);
    std::cout << "is polygon soup polygon mesh:"
	      << CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(
		     polygons)
	      << std::endl;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points,
	polygons, M);
    if (!CGAL::is_closed(M))
	CGAL::Polygon_mesh_processing::stitch_borders(M);
    std::cout << "Finished building Polygon Mesh." << std::endl;
    std::cout << "Is closed:" << CGAL::is_closed(M) << std::endl;
}

void MyPolyhedron::normalizeMeshViaModelMatrix()
{
    float scaleFactor = 1.0 / maxDim;
    modelMatrix = glm::scale(modelMatrix, glm::vec3(scaleFactor));

    glm::vec3 translationVector(-centerOfMass);
    modelMatrix = glm::translate(modelMatrix, translationVector);
}

void MyPolyhedron::setupDrawingBuffers()
{

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(MyVertex),
	&vertices[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
	&indices[0], GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
	(GLvoid*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
	(GLvoid*)offsetof(MyVertex, Normal));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
	(GLvoid*)offsetof(MyVertex, Color));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
    // std::cout << "printDebugInformation was called in "<<__func__<<std::endl;
    //     // printDebugInformation();
    //
}

void MyPolyhedron::updateMeshBuffers()
{

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(MyVertex),
	&vertices[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
	&indices[0], GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
	(GLvoid*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
	(GLvoid*)offsetof(MyVertex, Normal));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
	(GLvoid*)offsetof(MyVertex, Color));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
}