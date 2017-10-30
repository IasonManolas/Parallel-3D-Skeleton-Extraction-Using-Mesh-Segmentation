#ifndef DEBUG_MESHCONTRACTOR_H
#define DEBUG_MESHCONTRACTOR_H

std::pair<double, int> getMaximumAbsoluteDiagonalElement(SpMatrix M) {
  double maxNumber = 0;
  int maxIndex = -1;
  for (int index = 0; index < M.outerSize(); ++index) {
    double absValue = std::abs(M.coeff(index, index));
    if (absValue > maxNumber) {
      maxNumber = absValue;
      maxIndex = index;
    }
  }

  return std::make_pair(maxNumber, maxIndex);
}

bool hasNaN(EigenMatrix M) {
  for (int row = 0; row < M.rows(); row++) {
    for (int col = 0; col < M.cols(); col++) {
      if (std::isnan(M(row, col))) {
        std::cout << "Matrix has nan at: " << row << "," << col << std::endl;
        return true;
      }
    }
  }
  return false;
}

bool hasNaN(SpMatrix M) {
  for (int k = 0; k < M.outerSize(); ++k) {
    for (SpMatrix::InnerIterator it(M, k); it; ++it) {
      if (std::isnan(it.value())) {
        std::cout << "Matrix has nan at: " << it.row() << "," << it.col()
                  << std::endl;
        return true;
      }
    }
  }
  return false;
}

void printSparseMatrix(SpMatrix M, std::string matrixName) {
  for (int k = 0; k < M.outerSize(); ++k)
    for (SpMatrix::InnerIterator it(M, k); it; ++it) {
      std::cout << matrixName << "(" << it.row() << "," << it.col()
                << ")=" << it.value() << std::endl;
    }
}

void printMatrix(EigenMatrix M, std::string matrixName) {
  for (int row = 0; row < M.rows(); row++)
    for (int col = 0; col < M.cols(); col++)
      std::cout << matrixName << "(" << row << "," << col << ")=" << M(row, col)
                << std::endl;
}

void printVector(Vector V, std::string name) {
  for (int i = 0; i < V.size(); i++) {
    std::cout << name << "(" << i << ")=" << V(i) << std::endl;
  }
}

void printDiagonalElementsOfSparseMatrix(SpMatrix M, std::string matrixName) {

  for (int k = 0; k < M.outerSize(); ++k)

    for (SpMatrix::InnerIterator it(M, k); it; ++it) {
      if (it.row() != it.col())
        continue;
      std::cout << matrixName << "(" << it.row() << "," << it.col()
                << ")=" << it.value() << std::endl;
    }
}

void removeInfinity(SpMatrix &M) {
  double replaceWith = maxNumber;
  // double replaceWith = std::numeric_limits<double>::max();
  for (int k = 0; k < M.outerSize(); ++k)
    for (SpMatrix::InnerIterator it(M, k); it; ++it)
      if (it.value() == std::numeric_limits<double>::infinity())
        M.coeffRef(it.row(), it.col()) = replaceWith;
      else if (it.value() == -std::numeric_limits<double>::infinity())
        M.coeffRef(it.row(), it.col()) = -replaceWith;
}
void removeInfinity(EigenMatrix &M) {
  double replaceWith = maxNumber;
  // double replaceWith = std::numeric_limits<double>::max();
  for (int row = 0; row < M.rows(); row++)
    for (uint col = 0; col < M.cols(); col++)
      if (M(row, col) == std::numeric_limits<double>::infinity())
        M.coeffRef(row, col) = replaceWith;
      else if (M(row, col) == -std::numeric_limits<double>::infinity())
        M.coeffRef(row, col) = -replaceWith;
}

bool hasInfinity(SpMatrix M) {
  for (int k = 0; k < M.outerSize(); ++k) {
    for (SpMatrix::InnerIterator it(M, k); it; ++it) {
      if (std::isinf(it.value())) {
        std::cout << "Matrix has infinity at: " << it.row() << "," << it.col()
                  << std::endl;
        return true;
      }
    }
  }
  return false;
}

bool hasInfinity(EigenMatrix M) {
  for (uint row = 0; row < M.rows(); row++) {
    for (uint col = 0; col < M.cols(); col++) {
      if (std::isinf(M(row, col))) {
        return true;
      }
    }
  }
  return false;
}
// std::vector<size_t> MeshContractor::getVertexIndicesWithHighLaplacianValue()
// {
//  double threshold = 100;
//  std::vector<size_t> highLIndices;
//  for (size_t i = 0; i < m_M.number_of_vertices(); i++) {
//    if (m_L.coeff(i, i) > threshold)
//      highLIndices.push_back(i);
//  }
//  std::cout << "There are " << highLIndices.size() << " high vertices."
//            << std::endl;
//  return highLIndices;
//}
#include <igl/cotmatrix.h>
SpMatrix MeshContractor::computeLaplaceOperatorUsingIGL() {
  EigenMatrix V = constructVertexMatrix();
  SpMatrix igl_L;
  igl::cotmatrix(V, F, igl_L);
  return igl_L;
  // removeInfinity(m_L);
  // printSparseMatrix(m_L, "L");

  // printSparseMatrix(m_L, "L");
  // assert(!hasNaN(m_L));
  // if (hasNaN(m_L))
  //  fixNaN(m_L);
}
#endif // DEBUG_MESHCONTRACTOR_H
