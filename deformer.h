#ifndef DEFORMER_H
#define DEFORMER_H

#include "mesh.h"
#include "mypolyhedron.h"

class Deformer
{
public:
//    Deformer(){}
    Deformer(MyPolyhedron &polyhedron):P(polyhedron){}

private:
  MyPolyhedron& P;
};

#endif // DEFORMER_H
