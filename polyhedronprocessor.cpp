#include "polyhedronprocessor.h"

Kernel::Iso_cuboid_3 PolyhedronProcessor::getBbox() const
{
    return bbox;
}
void PolyhedronProcessor::computeBBox()
{
    bbox=CGAL::bounding_box(P.points_begin(),P.points_end());

}
