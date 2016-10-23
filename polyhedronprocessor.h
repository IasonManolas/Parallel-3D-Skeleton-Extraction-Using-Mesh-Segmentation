#ifndef POLYHEDRONPROCESSOR_H
#define POLYHEDRONPROCESSOR_H

#include<iostream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/bounding_box.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> 							Polyhedron;

class PolyhedronProcessor
{
public:
    PolyhedronProcessor(){}
    PolyhedronProcessor(Polyhedron P):P{P}
    {
        computeBBox();
    }

    Kernel::Iso_cuboid_3 getBbox() const;

private:
    Polyhedron P;
    Kernel::Iso_cuboid_3 bbox;
    void computeBBox();
};

#endif // POLYHEDRONPROCESSOR_H
