/*
 * ManifoldMeshReconstructor.h
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#ifndef MANIFOLDMESHRECONSTRUCTOR_H_
#define MANIFOLDMESHRECONSTRUCTOR_H_

#include <types_reconstructor.hpp>
#include <types_config.hpp>
#include <Logger.h>
#include <vector>
#include <ManifoldManager.h>
#include <OutputCreator.h>
#include <fstream>
#include <iostream>
/**
* This class provides the API to manage the 2-manifold creation as explained in the 
* paper:
* 
* Andrea Romanoni, Matteo Matteucci. Incremental Urban Manifold Reconstruction from a Sparse Edge-Points Cloud. 
* IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) 2015.
*
* The class builds a 3D Delaunay Triangulation, and keeps the information of the visibility updated incrementally.
* In the same time it provides the functions to incrementally estimate a manifold mesh out of the triangulation.
*
* The moving point related functions (as explained in the paper: A. Romanoni, M. Matteucci. 
* Efficient moving point handling for incremental 3D manifold reconstruction. 
* International Conference on Image Analysis and Processing (ICIAP) 2015.) have not been tested in this library yet
* 
*
*
*/
class ManifoldMeshReconstructor {
public:
  ManifoldMeshReconstructor(ManifoldReconstructionConfig conf);
  virtual ~ManifoldMeshReconstructor();

  /*Functions to add informations about the points, the cameras and visibility
  * Important Note: whenever you add here the points, they are not added automatically
  * to the Delaunay Triangulation. to actually add these points you need to call the 
  * insertNewPointsFromCam function which manages conveniently the manifold update 
  *
  **/
  void addPoint(float x, float y, float z);
  int addPointWhere(float x, float y, float z);
  void addCameraCenter(float x, float y, float z);
  void addVisibilityPair(int camIdx, int pointIdx);
  void movePoint(int idxPoint, float x, float y, float z);
  PointD3 movePointGetOld(int idxPoint, float x, float y, float z);

  //incremental reconstruction

  /**
  * Insert all the points visible from camera idxCam. Before calling this 
  * function you have to add the camera, the points and the proper visibility information.
  * If needed this function shrinks the manifold before point addition
  */
  void insertNewPointsFromCam(int idxCam, bool incremental = true);
  /**
  * Perform ray tracing to update the visibility information, of the tetrahedra,inducted by visibility 
  * rays starting from camera idxCam*/
  void rayTracingFromCam(int idxCam);
  /**
  * Estimate the manifold mesh bootstrapping from the most visible tetrahedron of the boundary if the manifold is initialized, otherwise
  * it boostraps from the most visible tetrahedron among all*/
  void growManifold();
  /**
  * Same as growManifold() but if the manifold is not initialized
  * it boostraps from the tetrahedron containing camera idxCam*/
  void growManifold(int idxCam);
  /**
  * Same as growManifold() but grows the mesh with the general manifold test, i.e., less efficient than growManifold() but 
  * manages the genus change. It is usually called after a call to growManifold(). 
  */
  void growManifoldSev();

  void clearLog();

  /*Saves the current manifold in OFF file format*/
  void saveManifold(const std::string filename);
  /*Saves the current boundary in OFF file format*/
  void saveBoundary(const std::string filename);
  /*Saves the old manifold with points up to cam  idx in OFF file format*/
  void saveOldManifold(const std::string filename, int idx);
  /*Saves the submap manifold specified by the vector idx in OFF file format*/
  void saveOldManifold(const std::string filename, std::vector<int> idx);
  /*Saves in OFF file format the boundary mesh between tetrahedra labelled as free space and those labelled as matter */
  void saveFreespace(const std::string filename);


  void setWeights(float w_1, float w_2, float w_3);

private:
  void shrinkManifold(const PointD3 &camCenter);
  /*create the initial grid of steiner points to avoid the infinite tetrahedra issue, while growing the mesh.
  * The parameters inside these function are also useful to avoid the creation of too big tetrahedra wich may cause
  * visual artifacts in the final mesh*/
  void createSteinerPointGridAndBound();
  /*add new point to the Delaunay Triangulation*/
  bool insertNewPoint(PointReconstruction &points);
  /*mark a tetrahedron with a visibility rays, it updates the information stored in the tetrahedron*/
  void markTetraedron(Delaunay3::Cell_handle & cell, const int camIndex, const int featureIndex, bool incrementCount = true);
  /*Traces the ray from a camera to a point and update the weights, i.e., the visibility infromation, soterd in the traversed tetrahedra
  * and in their neighbor. Here is implemented the Inverse Cone Heuristic (ICH)*/
  void rayTracing(int idxCam, int idxPoint, bool bOnlyMarkNew = false, bool incrementCount = true);

  /*Update the weights of the cellsToBeUpdates according to the values in the vecDistanceWeights. This implements the suboptimal policy*/
  void updateDistanceAndWeights(std::vector<Delaunay3::Cell_handle> &cellsToBeUpdated, const std::vector<DistanceWeight> &vecDistanceWeight);


  /*not tested yet*/
  int moveVertex(int idxPoint, int idxCam);
  /*not tested yet*/
  int moveVertex_WHeuristic(int idxPoint, int idxCam);


  /**test if the segment traverse two tetrahedra in facet f*/
  bool cellTraversalExitTest(int & f, int & fOld, const Delaunay3::Cell_handle& tetCur, const Delaunay3::Cell_handle &tetPrev, const Segment & constraint);

  ManifoldReconstructionConfig conf_;

  Delaunay3 dt_;
  utilities::Logger logger_;
  std::vector<PointReconstruction> points_;
  std::vector<CamReconstruction> cams_;
  std::vector<glm::vec3> camsPositions_;
  std::vector<int> curPointsVisible_;
  std::vector<Vertex3D_handle> vecVertexHandles_;
  std::vector<Delaunay3::Cell_handle> freeSpaceTets_;
  std::vector<int> idxPointsForRayTracing_;

  std::set<Delaunay3DCellInfo::FSConstraint, Delaunay3DCellInfo::LtFSConstraint> curConstraints_;
  std::vector<DistanceWeight> vecDistanceWeight_;


  ManifoldManager * manifoldManager_;
  OutputCreator *outputM_;
  std::vector<int> pointsMovedIdx_;

  float l_, stepX_, stepY_, stepZ_;

  std::ofstream fileOut_;
};

#endif /* MANIFOLDMESHRECONSTRUCTOR_H_ */
