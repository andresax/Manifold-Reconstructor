

#ifndef MANIFOLDMANAGER_H_
#define MANIFOLDMANAGER_H_

//#include <Mesh.h>
#include <types_reconstructor.hpp>
#include <OutputCreator.h>
#include <fstream>
#include <iostream>
/**
* This class provides the basic tools to manage the actual manifold creation, 
* such as the region growing procedure, the manifoldness tests and the update of the
* tetrahedra-based boundary
* 
* */
class ManifoldManager {
public:
  ManifoldManager(Delaunay3 &dt,bool inverseConic_, float probabTh_);
  virtual ~ManifoldManager();

  /*Tell on which delaunay triangulation the class instatiation will apply the grow/shrink algorithms*/
  void setDt(Delaunay3 & dt) {
    dt_ = dt;
  }

  void setInverseConic(bool inverseConic) {
    inverseConic_ = inverseConic;
  }

  size_t getBoundarySize(){
    return boundaryCells_.size();
  }

  /*Grow the manifold from the startingcell */
  void regionGrowingBatch(Delaunay3::Cell_handle &startingCell);
  /*Grow the manifold from the cell including the point in position firstCamPosition*/
  void regionGrowingBatch(PointD3 firstCamPosition);
  /* Grow the manifold  one tet-at-once incrementally, bootstrapping from the current boundary inside and outside the manifold */
  void regionGrowing();

  /*Grow the manifold several-tet-at-one in order to handle the genus change It bootstraps from the boundary between inside and outside the manifold*/
  void growSeveralAtOnce();

  /*shrink the manifold such that all the space inside the sphere with center in camPosition
  and ray maxPointToPointDistance+maxPointToCamDistance is matter. In this way, our are 
  the able to add points seen from the cam in cam position 
  to the triangulation and retriangulate the space without breaking the delauna property*/
  void shrinkManifold(const PointD3 &camPosition, const float &maxPointToPointDistance, const float &maxPointToCamDistance);

  /*shrink the manifold several-tet-at-once in order to handle the genus change*/
  void shrinkSeveralAtOnce();

  const std::vector<Delaunay3::Cell_handle>& getBoundaryCells() const {
    return boundaryCells_;
  }

private:

 
  
  void regionGrowingProcedure();


/******************************************************/
/**************Manifold check functions****************/
/******************************************************/
  bool additionTest(Delaunay3::Cell_handle &i);
  bool subtractionTest(Delaunay3::Cell_handle &i);
  bool singleTetTest(Delaunay3::Cell_handle &i);
  bool checkManifoldness(Delaunay3::Cell_handle &cellToTest1, int idxNeigh);
  bool removeAndCheckManifoldness(Delaunay3::Cell_handle &cellToTest1, int idxNeigh);
  bool addAndCheckManifoldness(Delaunay3::Cell_handle &cellToTest1, int idxNeigh);
  bool isRegular(Delaunay3::Vertex_handle &v);

/******************************************************/
/************Boundary update functions*****************/
/******************************************************/
  
  bool isInBoundary(Delaunay3::Cell_handle &cellToTest);
  bool isInBoundary(Delaunay3::Cell_handle &cellToTest, std::vector<int> &neighNotManifold);
  bool insertInBoundary(Delaunay3::Cell_handle &cellToTest);
  bool removeFromBoundary(Delaunay3::Cell_handle &cellToTest);
  void addTetAndUpdateBoundary(Delaunay3::Cell_handle &cell);
  void subTetAndUpdateBoundary(Delaunay3::Cell_handle &cell);


  bool isFreespace(Delaunay3::Cell_handle &cell);

  std::vector<Delaunay3::Cell_handle> boundaryCells_;

  Delaunay3& dt_;
  OutputCreator *outputM_;
  bool inverseConic_;
  float probabTh_;

  std::ofstream fileOut_;
};

#endif /* MANIFOLDMANAGER_H_ */
