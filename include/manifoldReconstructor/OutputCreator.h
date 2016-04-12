/*
 * OutputCreator.h
 *
 *  Created on: 29/giu/2015
 *      Author: andrea
 */

#ifndef OUTPUTCREATOR_H_
#define OUTPUTCREATOR_H_

#include <types_reconstructor.hpp>
#include <string>
#include <vector>

class OutputCreator {
public:

  // Hashing-related structs:
  struct HashVertHandle {
    size_t operator()(const Delaunay3::Vertex_handle x) const {
      return (size_t) (&(*x));
    } // use pointer to create hash
  };
  struct EqVertHandle {
    bool operator()(const Delaunay3::Vertex_handle x, const Delaunay3::Vertex_handle y) const {
      return x == y;
    }
  };

  OutputCreator(Delaunay3 &dt_);
  virtual ~OutputCreator();

  void setDt(const Delaunay3& dt) {
    dt_ = dt;
  }

  void setTh(float th) {
    th_ = th;
  }
  void writeOFF(const std::string filename, int lastCam = -1);
  void writeOFF(const std::string filename, std::vector<int> cams);
  void writeFreespaceOFF(const std::string filename);
  void writeUsedVOFF(const std::string filename);
  void writeNotUsedVOFF(const std::string filename);
  void writeBoundaryOFF(const std::string filename, const std::vector<Delaunay3::Cell_handle> &boundaryCells);

private:

  void tetrahedraToTriangles(std::vector<PointD3> & points, std::vector<PointD3> & tris, int lastCam = -1);
//  void tetrahedraBoundaryToTriangles(std::vector<PointD3> & points, std::vector<PointD3> & tris);
  void tetrahedraToTriangles(std::vector<PointD3> & points, std::vector<PointD3> & tris, std::vector<int> cams);
  void tetrahedraToTrianglesFreespace(std::vector<PointD3> & points, std::vector<PointD3> & tris, const float nVoteProbThresh, const bool weights = true);
  void facetToTriangle(const Delaunay3::Facet & f, std::vector<Delaunay3::Vertex_handle> & vecTri);
  Delaunay3& dt_;
  float th_;
};

#endif /* OUTPUTCREATOR_H_ */
