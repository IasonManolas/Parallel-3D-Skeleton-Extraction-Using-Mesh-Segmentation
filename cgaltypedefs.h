#ifndef CGALTYPEDEFS_H
#define CGALTYPEDEFS_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CGALSurfaceMesh = CGAL::Surface_mesh<Kernel::Point_3>;
using Ray = Kernel::Ray_3;
using Point = Kernel::Point_3;

// segment mesh
using face_descriptor = boost::graph_traits<CGALSurfaceMesh>::face_descriptor;
using Facet_double_map = CGALSurfaceMesh::Property_map<face_descriptor, double>;
using Facet_int_map =
    CGALSurfaceMesh::Property_map<face_descriptor, std::size_t>;

// intersects
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
using Primitive = CGAL::AABB_face_graph_triangle_primitive<CGALSurfaceMesh>;
using Traits = CGAL::AABB_traits<Kernel, Primitive>;
using Tree = CGAL::AABB_tree<Traits>;
using Ray_intersection =
    boost::optional<Tree::Intersection_and_primitive_id<Kernel::Ray_3>::Type>;
#include <glm/vec3.hpp>
struct MyVertex {
  glm::vec3 Position;
  glm::vec3 Normal;
  glm::vec3 Color{glm::vec3(0.0f, 0.0f, 0.0f)};
};
#endif // CGALTYPEDEFS_H
