#include "scene.h"

void Scene::Draw(Shader *activeShader, Shader *axisShader, Shader *edgeShader,
		 Shader *nodeShader) {
	setSceneUniforms(activeShader, axisShader, edgeShader, nodeShader);
	M.handle_drawing(activeShader, edgeShader, nodeShader);
	if (showAxes) {
		axisShader->Use();
		sceneAxes.Draw();
	}
}
void Scene::setSkeletonShaderUniforms(Shader *shader) {
	shader->Use();
	camera.setUniforms(shader);
	setProjectionMatrixUniform(shader);
}

void Scene::updateProjectionMatrix(int w, int h) {
	projectionMatrix = glm::perspective(camera.fov, float(w) / float(h),
					    nearPlane, farPlane);
}

void Scene::handle_cameraZoomChange(float delta) {
	camera.processWheelMovement(delta);
}

void Scene::handle_cameraReset() {
	camera.resetCamera();
	light.changeLightDirection(camera.getPosition());
}

void Scene::initializeScene() {
	// loadMesh("../Models/tyra.obj");
	loadPointSphere();
	PS.setPosition(0, 0, 0);
	// loadMesh("../Models/Small/test.obj");

	// loadMesh("../Models/tyra_rightFoot.off");
	loadMesh("../Models/Test Set/bunny_low.obj");
	// loadMesh("../Models/cylinder.obj");
	// loadMesh("../Models/Small/coctel.obj");
	// loadMesh("../Models/Small/Wrong by assimp/stretched cube.obj");
	// loadMesh("../Models/teapot.obj");
	// loadMesh("../Models/icosahedron.obj");
}

void Scene::loadPointSphere() { PS.load("../Models/Small/icosahedron.obj"); }

void Scene::handle_segmentSelection(float mousePosX, float mousePosY,
				    int windowWidth, int windowHeight) {
	Ray_intersection intersection;
	bool intersectionFound = rayIntersectsPolyhedron(
	    mousePosX, mousePosY, windowWidth, windowHeight, intersection);
	if (intersectionFound) {
		M.handle_segmentSelection(intersection);
	}
}

void Scene::handle_showSegments() { M.handle_showSegments(); }

void Scene::handle_segmentContraction(bool automatic,
				      bool usingCGALSSkeletonization = false) {
	if (!usingCGALSSkeletonization) {
		M.handle_segmentContraction(automatic);
	} else {
		M.handle_segmentSkeletonization();
	}
}

#include <CGAL/extract_mean_curvature_flow_skeleton.h>
void Scene::handle_meshContraction(bool automatic,
				   bool usingCGALSSkeletonization = false) {
	if (!usingCGALSSkeletonization) {
		M.handle_meshContraction(automatic);
	} else {
		// M.handle_skeletonization();
		typedef CGAL::Mean_curvature_flow_skeletonization<
		    CGALSurfaceMesh>
		    Skeletonization;
		typedef Skeletonization::Skeleton Skeleton;
		// if (!CGAL::is_triangle_mesh(tmesh)) {
		// 	std::cout << "Input geometry is not
		// triangulated." <<
		// std::endl;
		// 	return EXIT_FAILURE;
		// }
		Skeleton skeleton;
		if (!CGAL::is_closed(M.m_M)) {
			using halfedge_handle = boost::graph_traits<
			    CGALSurfaceMesh>::halfedge_descriptor;
			using facet_handle = boost::graph_traits<
			    CGALSurfaceMesh>::face_descriptor;
			BOOST_FOREACH (halfedge_handle h, halfedges(M.m_M)) {
				if (M.m_M.is_border(h)) {
					std::vector<facet_handle> patch_facets;
					CGAL::Polygon_mesh_processing::
					    triangulate_hole(M.m_M, h,
							     std::back_inserter(
								 patch_facets));
				}
			}
		}

		// CGAL::extract_mean_curvature_flow_skeleton(M.m_M, skeleton);

		// M.m_skeleton.populateSkeleton(skeleton);
		// M.alphaValue = 0.4;

		auto cgalContractionTimeStart =
		    std::chrono::high_resolution_clock::now();
		Skeletonization mcs(M.m_M);
		mcs.contract_until_convergence();
		auto cgalContraactionTimeEnd =
		    std::chrono::high_resolution_clock::now();

		std::cout << "total contraction time using cgal:"
			  << std::chrono::duration<double>(
				 cgalContraactionTimeEnd -
				 cgalContractionTimeStart)
				 .count()
			  << std::endl;

		std::cout << "Volume meso skeleton/original:"
			  << CGAL::Polygon_mesh_processing::volume(
				 mcs.meso_skeleton()) /
				 CGAL::Polygon_mesh_processing::volume(M.m_M)
			  << std::endl;

		// auto contractedMesh = mcs.meso_skeleton();
		// std::cout << "Meso skeleton has: "
		// 	  << contractedMesh.number_of_vertices() << "
		// vertices"
		// 	  << std::endl;

		// std::cout << "Number of vertices of the skeleton: "
		// << boost::num_vertices(skeleton) << "\n";
		// std::cout << "Number of edges of the skeleton: "
		// 	  << boost::num_edges(skeleton) << "\n";
	}
}

void Scene::handle_meshConnectivitySurgery() {
	M.handle_meshConnectivitySurgery();
}

void Scene::handle_meshRefinementEmbedding() {
	M.handle_meshRefinementEmbedding();
}

void Scene::handle_segmentRefinementEmbedding() {}

void Scene::handle_segmentConnectivitySurgery() {
	M.handle_segmentConnectivitySurgery();
}

void Scene::handle_meshContractionReversing() {
	M.handle_meshContractionReversing();
}

void Scene::handle_meshInflation() { M.handle_inflation(); }

void Scene::handle_meshDeflation() { M.handle_deflation(); }

void Scene::loadMesh(std::__cxx11::string filename) {
	camera.resetCamera();
	light.changeLightDirection(camera.getPosition());

	// M = Mesh(PS);
	M.load(filename);
	PS.updateRadius(M.getM());
	// M.setPointSphere(PS);
}

void Scene::handle_axesStateChange(bool value) { showAxes = value; }

bool Scene::rayIntersectsPolyhedron(const int &mouseX, const int &mouseY,
				    const int width, const int height,
				    Ray_intersection &intersection) {
	glm::vec3 camPos = camera.getPosition();
	float x = 2.0 * mouseX / width - 1;
	float y = 1 - (2.0 * mouseY) / height;
	float z = -1;  // we want the ray to point into the screen
	glm::vec3 ndcs(x, y, z);
	// NDCSpace -> clipSpace
	glm::vec4 ray4_clipSpace(ndcs.x, ndcs.y, ndcs.z, 1.0);
	// clipSpace -> eyeSpace
	glm::vec4 ray4_eyeSpace =
	    glm::inverse(projectionMatrix) * ray4_clipSpace;
	ray4_eyeSpace = glm::vec4(ray4_eyeSpace.x, ray4_eyeSpace.y, -1.0, 0.0);
	// eyeSpace -> worldSpace
	glm::vec4 ray4_worldSpace =
	    glm::inverse(camera.getViewMatrix()) * ray4_eyeSpace;

	// worldSpace -> modelSpace
	glm::vec4 ray4_modelSpace =
	    glm::inverse(M.getModelMatrix()) * ray4_worldSpace;

	glm::vec3 ray3_modelSpace(ray4_modelSpace.x, ray4_modelSpace.y,
				  ray4_modelSpace.z);
	ray3_modelSpace = glm::normalize(ray3_modelSpace);

	glm::vec4 camPos4_worldSpace = glm::vec4(camPos, 1.0);
	glm::vec4 camPos4_modelSpace =
	    glm::inverse(M.getModelMatrix()) * camPos4_worldSpace;
	glm::vec3 camPos3_modelSpace(camPos4_modelSpace.x, camPos4_modelSpace.y,
				     camPos4_modelSpace.z);

	// glm ray -> CGAL ray
	CGAL::Ray_3<Kernel> ray_modelSpace(
	    Kernel::Point_3(camPos3_modelSpace.x, camPos3_modelSpace.y,
			    camPos3_modelSpace.z),
	    Kernel::Direction_3(ray3_modelSpace.x, ray3_modelSpace.y,
				ray3_modelSpace.z));

	intersection = M.intersects(ray_modelSpace);

	if (intersection) {
		return true;
	}
	return false;
}

void Scene::handle_mouseMovement(const QVector2D &mouseDV) {
	camera.processMouseMovement(mouseDV);
	light.changeLightDirection(camera.getPosition());
}

void Scene::handle_meshVerticesStateChange(int state) {
	M.handle_showVerticesStateChange(state);
	// updatePointSpheresOnVerticesPositions();
}

void Scene::handle_saveModel(const std::__cxx11::string destinationDirectory) {
	M.handle_saveModel(destinationDirectory);
}

void Scene::handle_saveSegment(
    const std::__cxx11::string destinationDirectory) {
	M.handle_saveSegment(destinationDirectory);
}

void Scene::handle_clearSkeleton() { M.handle_clearSkeleton(); }

void Scene::setSceneUniforms(Shader *modelShader, Shader *axisShader,
			     Shader *edgeShader, Shader *nodeShader) {
	modelShader->Use();
	light.setUniforms(modelShader);
	camera.setUniforms(modelShader);
	setProjectionMatrixUniform(modelShader);

	axisShader->Use();
	camera.setUniforms(axisShader);
	setProjectionMatrixUniform(axisShader);
	glUniformMatrix4fv(glGetUniformLocation(axisShader->programID, "model"),
			   1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));

	edgeShader->Use();
	camera.setUniforms(edgeShader);
	setProjectionMatrixUniform(edgeShader);

	nodeShader->Use();
	light.setUniforms(nodeShader);
	camera.setUniforms(nodeShader);
	setProjectionMatrixUniform(nodeShader);
}

void Scene::setProjectionMatrixUniform(Shader *shader) {
	glUniformMatrix4fv(
	    glGetUniformLocation(shader->programID, "projection"), 1, GL_FALSE,
	    glm::value_ptr(projectionMatrix));
}
