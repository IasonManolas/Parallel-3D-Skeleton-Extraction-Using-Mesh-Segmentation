#include "connectivitysurgeon.h"

// ConnectivitySurgeon::ConnectivitySurgeon() {}

ConnectivitySurgeon::ConnectivitySurgeon(CGALSurfaceMesh contractedMesh)
    : m_M(contractedMesh) {
  std::cout << "constructed connectivity surgeon" << std::endl;
}
