#ifndef CONNECTIVITYSURGEON_H
#define CONNECTIVITYSURGEON_H

#include "cgaltypedefs.h"
/*
 ConnectivitySurgeon operates on an already contracted mesh. In short it
 collapses edges using a greedy algorithm until no faces are left.
 It implements 4 tasks which are:
 1.Edge collapse
 2.Tunnel collapse prohibition
 3.Shape cost for each edge
 4.Sampling cost for each edge
 For more details see:
 http://visgraph.cse.ust.hk/projects/skeleton/skeleton_sig08.pdf

  */
class ConnectivitySurgeon {
public:
  // ConnectivitySurgeon();
  ConnectivitySurgeon(CGALSurfaceMesh contractedMesh);

private:
  CGALSurfaceMesh m_M; // the mesh on which the surgery is being applied
};

#endif // CONNECTIVITYSURGEON_H
