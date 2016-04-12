/*
 * ManifoldMeshReconstructor.cpp
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#include <ManifoldMeshReconstructor.h>
#include <vector>
#include <utilities.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>

ManifoldMeshReconstructor::ManifoldMeshReconstructor(ManifoldReconstructionConfig conf) {
  conf_ = conf;
  manifoldManager_ = new ManifoldManager(dt_, conf_.inverseConicEnabled, conf_.probOrVoteThreshold);
  outputM_ = new OutputCreator(dt_);
  //fileOut_.open("ManifoldMeshReconstructor.log");
  l_ = 0;
  stepX_ = stepY_ = stepZ_ = -1;
}

ManifoldMeshReconstructor::~ManifoldMeshReconstructor() {
  delete (manifoldManager_);
  delete (outputM_);

}

void ManifoldMeshReconstructor::setWeights(float w_1, float w_2, float w_3) {
  conf_.w_1 = w_1;
  conf_.w_2 = w_2;
  conf_.w_3 = w_3;

}
void ManifoldMeshReconstructor::clearLog() {
  fileOut_.close();
  fileOut_.open("ManifoldMeshReconstructor.log");
}
void ManifoldMeshReconstructor::addPoint(float x, float y, float z) {
  PointReconstruction t;
  t.position = PointD3(x, y, z);
  points_.push_back(t);
}

void ManifoldMeshReconstructor::movePoint(int idxPoint, float x, float y, float z) {
  points_[idxPoint].position = PointD3(x, y, z);
  pointsMovedIdx_.push_back(idxPoint);
}

PointD3 ManifoldMeshReconstructor::movePointGetOld(int idxPoint, float x, float y, float z) {
  PointD3 old = points_[idxPoint].position;
  points_[idxPoint].position = PointD3(x, y, z);
  pointsMovedIdx_.push_back(idxPoint);
  return old;
}

int ManifoldMeshReconstructor::addPointWhere(float x, float y, float z) {
  PointReconstruction t;
  t.position = PointD3(x, y, z);
  points_.push_back(t);

  return points_.size() - 1;
}
void ManifoldMeshReconstructor::addCameraCenter(float x, float y, float z) {
  CamReconstruction t;
  t.position = PointD3(x, y, z);
  glm::vec3 pos = glm::vec3(x, y, z);

  cams_.push_back(t);
  camsPositions_.push_back(pos);
}

void ManifoldMeshReconstructor::addVisibilityPair(int camIdx, int pointIdx) {
  cams_[camIdx].visiblePoints.push_back(pointIdx);
  cams_[camIdx].newVisiblePoints.push_back(pointIdx);
  points_[pointIdx].viewingCams.push_back(camIdx);
}

void ManifoldMeshReconstructor::insertNewPointsFromCam(int camIdx, bool incremental) {
  if (dt_.number_of_vertices() == 0) {
    createSteinerPointGridAndBound();
  }
  if (incremental) {
    shrinkManifold(cams_[camIdx].position);
  }

  curConstraints_.clear();
  pointsMovedIdx_.clear();

  int count = 0;
  for (auto id : pointsMovedIdx_) {
    if (moveVertex(id, camIdx)) {
      count++;
    }
  }

  logger_.startEvent();

  int oldNumVertices = (int) vecVertexHandles_.size();
  idxPointsForRayTracing_.clear();
  std::vector<int> newPointsIdx;
  for (auto v : cams_[camIdx].newVisiblePoints) {
    if (points_[v].new_) {

      if (utilities::distanceEucl(points_[v].position, cams_[camIdx].position) < conf_.maxDistanceCamFeature) {

        vecDistanceWeight_.clear();

        if (insertNewPoint(points_[v])) {

          idxPointsForRayTracing_.push_back(v);
          points_[v].new_ = false;
          newPointsIdx.push_back(v);

          /* std::cout<< points_[v].vertexHandle->point().x()<<" ";
           std::cout<< points_[v].vertexHandle->point().y()<<" ";
           std::cout<< points_[v].vertexHandle->point().z()<<" "<<std::endl;
           std::cout<< points_[v].vertexHandle->info().getListViewingCam().size()<<" "<<std::endl;*/
          // std::cout<< points_[v].vertexHandle->info().getListViewingCam().size()<<" "<<std::endl;
          points_[v].vertexHandle->info().setLastCam(camIdx);
          points_[v].vertexHandle->info().setFirstCam(camIdx);
          /*for(auto c: points_[v].viewingCams){

            if(c<=camIdx){
            std::cout<< " "<<c<<" "<<std::endl;
              points_[v].vertexHandle->info().addCam(camIdx);
            }
          }*/

          /*std::cout<< points_[v].vertexHandle->point().x()<<" ";
           std::cout<< points_[v].vertexHandle->point().y()<<" ";
           std::cout<< points_[v].vertexHandle->point().z()<<" "<<std::endl;
           std::cout<< points_[v].vertexHandle->info().getListViewingCam().size()<<" "<<std::endl;*/


          if (conf_.enableSuboptimalPolicy) {
            std::vector<Delaunay3::Cell_handle> newCells;
            dt_.incident_cells(points_[v].vertexHandle, std::inserter(newCells, newCells.begin()));
            updateDistanceAndWeights(newCells, vecDistanceWeight_);
          }

        }
      }

    } else {
      idxPointsForRayTracing_.push_back(v);
      points_[v].vertexHandle->info().setLastCam(camIdx);
      for(auto c: points_[v].viewingCams){

        if(c<=camIdx){
          points_[v].vertexHandle->info().addCam(camIdx);
        }
      }
    }

  }

  if (!conf_.enableSuboptimalPolicy) {

    std::set<Delaunay3::Cell_handle> setNewCells;
    for (auto i : newPointsIdx) {
      dt_.incident_cells(points_[i].vertexHandle, std::inserter(setNewCells, setNewCells.begin()));

    }

    for (auto itConstraint : curConstraints_) {
      rayTracing(itConstraint.first, itConstraint.second, true);
    }

    for (std::set<Delaunay3::Cell_handle>::iterator itCell = setNewCells.begin(); itCell != setNewCells.end(); itCell++) {
      (*itCell)->info().markOld();
    }
  }
  logger_.endEventAndPrint("AddFeatures", true);

}

void ManifoldMeshReconstructor::rayTracingFromCam(int idxCam) {

  for (auto visPt : idxPointsForRayTracing_) {
    rayTracing(idxCam, visPt, false, true);
  }
}

void ManifoldMeshReconstructor::rayTracing(int idxCam, int idxPoint, bool bOnlyMarkNew, bool incrementCount) {
  Delaunay3::Cell_handle tetPrev;
  Delaunay3::Cell_handle tetCur;
  Delaunay3::Locate_type lt;
  int li, lj;
  PointD3 tmp = points_[idxPoint].position;
  PointD3 tmp2 = cams_[idxCam].position;
  Segment constraint = Segment(tmp, tmp2);
  Vertex3D_handle handlePoint = points_[idxPoint].vertexHandle;
//  std::cout<<handlePoint->point()<<std::endl;
  /*******************************************************************************************
   Look for the tetrahedron incident to Q intersected by the ray from camera O to point Q
   ******************************************************************************************/
  std::vector<Delaunay3::Cell_handle> qCells;
  //stores in the vector qCells the handle to the cells (=tetrahedra) incident to Q
  dt_.incident_cells(handlePoint, std::back_inserter(qCells));
  //fileOut_ <<" size conflictCells " <<qCells.size();

  for (auto cell : qCells) {
    if (dt_.side_of_cell(constraint.target(), cell, lt, li, lj) != CGAL::ON_UNBOUNDED_SIDE) {
      // the tetrahedron t contains O (NB:constraint.target() = O)
      if (!bOnlyMarkNew || cell->info().isNew() || conf_.enableSuboptimalPolicy) {
        markTetraedron(cell, idxCam, idxPoint, incrementCount);

      }
      // The ending of the ray is reached yet, so end the ray tracing
      return;
    }

    // Let f be the facet of t opposite to Q
    int f = cell->index(handlePoint); // this with the Cell_handle *itQCells defines the facet
    if (CGAL::do_intersect(dt_.triangle(cell, f), constraint)) {
      tetPrev = cell;
      tetCur = cell->neighbor(f); // t.actual = neighbour of t incident to facet f
      if (!bOnlyMarkNew || cell->info().isNew() || conf_.enableSuboptimalPolicy) {
        markTetraedron(cell, idxCam, idxPoint, incrementCount);

      }
      if (!bOnlyMarkNew || tetCur->info().isNew() || conf_.enableSuboptimalPolicy) {
        markTetraedron(tetCur, idxCam, idxPoint, incrementCount);
      }
      break;
    }
  }

  /**********************************************
   Follow the ray through the triangulation
   *********************************************/
  int f, fOld;

  // While t.actual doesn't contain O
  while (cellTraversalExitTest(f, fOld, tetCur, tetPrev, constraint)) {

    if (conf_.inverseConicEnabled) {
      for (int curidFac = 0; curidFac < 4; ++curidFac) {
        //increment weight of the neighbors not traversed by the ray
        if ((curidFac != f) && (curidFac != fOld)) {
          Delaunay3::Cell_handle nearCellNotIntersected = tetCur->neighbor(curidFac);
          //if (!bOnlyMarkNew || nearCellNotIntersected->info().isNew() || conf_.enableSuboptimalPolicy) {
          if (incrementCount == true) {
            nearCellNotIntersected->info().incrementVoteCountProb(conf_.w_2);
          } else {
            nearCellNotIntersected->info().decrementVoteCountProb(conf_.w_2);
          }
          //}
          if (!conf_.enableSuboptimalPolicy) {
//            if (!bOnlyMarkNew || nearCellNotIntersected->info().isNew()) {
            if (incrementCount == true) {
              nearCellNotIntersected->info().addIntersection(idxCam, idxPoint, conf_.w_2, points_[idxPoint].idVertex, points_, camsPositions_);
            } else {
              nearCellNotIntersected->info().removeIntersection(idxCam, idxPoint, conf_.w_2, points_, camsPositions_);
            }
//            }
          }

          for (int curidFacNear = 0; curidFacNear < 4; ++curidFacNear) {
            //increment weight of the neighbors of the neighbors
            if (curidFacNear != nearCellNotIntersected->index(tetCur)) {
              if (incrementCount == true) {
                nearCellNotIntersected->neighbor(curidFacNear)->info().incrementVoteCountProb(conf_.w_3);
              } else {
                nearCellNotIntersected->neighbor(curidFacNear)->info().decrementVoteCountProb(conf_.w_3);
              }
              if (!conf_.enableSuboptimalPolicy) {
                //if (!bOnlyMarkNew || nearCellNotIntersected->neighbor(curidFacNear)->info().isNew()) {
                if (incrementCount == true) {
                  nearCellNotIntersected->neighbor(curidFacNear)->info().addIntersection(idxCam, idxPoint, conf_.w_3, points_[idxPoint].idVertex, points_,
                      camsPositions_);
                } else {
                  nearCellNotIntersected->neighbor(curidFacNear)->info().removeIntersection(idxCam, idxPoint, conf_.w_3, points_, camsPositions_);

                }
              }
            }
          }
        }
      }
    }
    tetPrev = tetCur; // t.precedent = t.actual

    tetCur = tetCur->neighbor(f); // t.actual = neighbour of t.precedent(==t.actual) incident to facet f
    if (!bOnlyMarkNew || tetCur->info().isNew() || conf_.enableSuboptimalPolicy) {
      markTetraedron(tetCur, idxCam, idxPoint, incrementCount);
    }

  }

  qCells.clear();


}

void ManifoldMeshReconstructor::markTetraedron(Delaunay3::Cell_handle & cell, const int camIndex, const int featureIndex, bool incrementCount) {

  if (incrementCount) {
    cell->info().incrementVoteCount(1); // t.n++

    freeSpaceTets_.push_back(cell);

    cell->info().incrementVoteCountProb(conf_.w_1);
    cell->info().incrementVoteCount(1.0);

    if (!conf_.enableSuboptimalPolicy) {
      cell->info().addIntersection(camIndex, featureIndex, conf_.w_1, points_[featureIndex].idVertex, points_, camsPositions_);
    }
  } else {
    cell->info().decrementVoteCount(1.0); // t.n++
    cell->info().decrementVoteCountProb(conf_.w_1); // t.n++
    if (!conf_.enableSuboptimalPolicy) {
      cell->info().removeIntersection(camIndex, featureIndex, points_, camsPositions_);
    }
  }
}
void ManifoldMeshReconstructor::growManifold() {
  if (manifoldManager_->getBoundarySize() == 0) {
    std::sort(freeSpaceTets_.begin(), freeSpaceTets_.end(), sortTetByIntersection());
    for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
      itCell->info().setKeptManifold(false);
      for (int curV = 0; curV < 4; ++curV) {
        itCell->vertex(curV)->info().setUsed(0);
        itCell->vertex(curV)->info().setNotUsed(true);
      }
    }
    Delaunay3::Cell_handle startingCell = freeSpaceTets_[freeSpaceTets_.size() - 1];
    manifoldManager_->regionGrowingBatch(startingCell);
  } else {
    manifoldManager_->regionGrowing();
  }

}

void ManifoldMeshReconstructor::growManifold(int camIdx) {
  if (manifoldManager_->getBoundarySize() == 0) {
    std::sort(freeSpaceTets_.begin(), freeSpaceTets_.end(), sortTetByIntersection());
    for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
      itCell->info().setKeptManifold(false);
      for (int curV = 0; curV < 4; ++curV) {
        itCell->vertex(curV)->info().setUsed(0);
        itCell->vertex(curV)->info().setNotUsed(true);
      }

    }

    Delaunay3::Cell_handle startingCell = dt_.locate(cams_[camIdx].position);

    manifoldManager_->regionGrowingBatch(startingCell);
  } else {
    manifoldManager_->regionGrowing();
  }

}
void ManifoldMeshReconstructor::growManifoldSev() {
  manifoldManager_->growSeveralAtOnce();

}

void ManifoldMeshReconstructor::saveManifold(const std::string filename) {
  outputM_->writeOFF(filename);
}

void ManifoldMeshReconstructor::saveBoundary(const std::string filename) {

  outputM_->writeBoundaryOFF(filename, manifoldManager_->getBoundaryCells());
}





void ManifoldMeshReconstructor::saveOldManifold(const std::string filename, int idx) {
  outputM_->writeOFF(filename, idx);
}

void ManifoldMeshReconstructor::saveOldManifold(const std::string filename, std::vector<int> idx) {

  outputM_->writeOFF(filename, idx);
}

void ManifoldMeshReconstructor::saveFreespace(const std::string filename) {
  outputM_->writeFreespaceOFF(filename);
}

void ManifoldMeshReconstructor::shrinkManifold(const PointD3 &camCenter) {

  manifoldManager_->shrinkManifold(camCenter, l_, conf_.maxDistanceCamFeature);
//  saveBoundary("tempa.off");
//  saveManifold("tempa2.off");


  manifoldManager_->shrinkSeveralAtOnce();
//  saveBoundary("tempb.off");
//  saveManifold("tempb2.off");

  manifoldManager_->shrinkManifold(camCenter, l_, conf_.maxDistanceCamFeature);
//  saveBoundary("tempc.off");
//  saveManifold("tempc2.off");

  manifoldManager_->shrinkSeveralAtOnce();
//  saveBoundary("tempb.off");
//  saveManifold("tempb2.off");

  manifoldManager_->shrinkManifold(camCenter, l_, conf_.maxDistanceCamFeature);

}

void ManifoldMeshReconstructor::createSteinerPointGridAndBound() {
  std::vector<PointD3> vecPoint;
//temple
//  stepX_ = 1.000;
//  stepY_ = 1.000;
//  stepZ_ = 1.000;
//
//  float inX = -5;
//  float finX = 5;
//  float inY = -5;
//  float finY = 5;
//  float inZ = -5;
//  float finZ = 5;

//dino
//  stepX_ = 0.100;
//  stepY_ = 0.100;
//  stepZ_ = 0.100;
//
//  float inX = -2;
//  float finX = 2;
//  float inY = -1;
//  float finY = 1;
//  float inZ = -2;
//  float finZ = 2;


///castle - kitti
  stepX_ = 10.000;
  stepY_ = 10.000;
  stepZ_ = 10.000;

  float inX = -400;
  float finX = 400;
  float inY = -400;
  float finY = 400;
  float inZ = -400;
  float finZ = 400;

//  float inX = -300;
//  float finX = 300;
//  float inY = -300;
//  float finY = 300;
//  float inZ = -300;
//  float finZ = 300;

//
//  stepX_ = 550.0;
//  stepY_ = 550.0;
//  stepZ_ = 550.0;
//
//  float inX = -5000;
//  float finX = 5000;
//  float inY = -5000;
//  float finY = 5000;
//  float inZ = -5000;
//  float finZ = 5000;


  float x = inX;
  do {
    float y = inY;
    do {
      float z = inZ;
      do {
        vecPoint.push_back(PointD3(x, y, z));
        z = z + stepZ_;
      } while (z <= finZ);
      y = y + stepY_;
    } while (y <= finY);
    x = x + stepX_;
  } while (x <= finX);

  dt_.insert(vecPoint.begin(), vecPoint.end());

  l_ = sqrt(stepX_ * stepX_ + stepY_ * stepY_ + stepZ_ * stepZ_);
}

bool ManifoldMeshReconstructor::insertNewPoint(PointReconstruction &point) {
  // Locate the point
  Delaunay3::Locate_type lt;
  int li, lj;
  Delaunay3::Cell_handle c = dt_.locate(point.position, lt, li, lj);
  if (lt == Delaunay3::VERTEX) {
    return false;
  }

  // Get the cells that conflict with Q in a vector vecConflictCells, and a facet on the boundary of this hole in f.
  std::vector<Delaunay3::Cell_handle> vecConflictCells;
  Delaunay3::Facet f;
  dt_.find_conflicts(point.position, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

  bool toBeAdded = true;

  if (false) {    ///if shrink
    for (auto it : vecConflictCells) {
      if (it->info().isBoundary() || it->info().iskeptManifold()) {
        toBeAdded = false;
      }
    }
  }
  if (!toBeAdded) {
    return false;
  }

  // Get the constraints.
  if (!conf_.enableSuboptimalPolicy) {
    for (auto it : vecConflictCells) {

      curConstraints_.insert(it->info().getIntersections().begin(), it->info().getIntersections().end());
    }
  } else {
    for (auto it : vecConflictCells) {
      PointD3 temp = CGAL::barycenter(it->vertex(0)->point(), 1.0, it->vertex(1)->point(), 1.0, it->vertex(2)->point(), 1.0, it->vertex(3)->point(), 1.0);
      vecDistanceWeight_.push_back(DistanceWeight(temp, (float) it->info().getVoteCountProb()));
    }
  }

  Vertex3D_handle hndlQ = dt_.insert_in_hole(point.position, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);

  // Add Q's Vertex_handle to vecVertexHandles
  vecVertexHandles_.push_back(hndlQ);
  point.idVertex = vecVertexHandles_.size() - 1;
  point.vertexHandle = hndlQ;
  return true;

}

void ManifoldMeshReconstructor::updateDistanceAndWeights(std::vector<Delaunay3::Cell_handle> &cellsToBeUpdated,
    const std::vector<DistanceWeight> &vecDistanceWeight) {
  for (auto itNewCell : cellsToBeUpdated) {
    PointD3 curCenter = CGAL::barycenter(itNewCell->vertex(0)->point(), 1.0, itNewCell->vertex(1)->point(), 1.0, itNewCell->vertex(2)->point(), 1.0,
        itNewCell->vertex(3)->point(), 1.0);
    float voteCount = 0;

    if (conf_.suboptimalMethod == 0) {
      //mindist policy
      float minDist = 10000000000000.0;
      for (auto itWeight : vecDistanceWeight) {
        float dist = utilities::distanceEucl(curCenter.x(), itWeight.first.x(), curCenter.y(), itWeight.first.y(), curCenter.z(), itWeight.first.z());
        if (minDist > dist) {
          minDist = dist;
          voteCount = itWeight.second;
        }
      }
    } else if (conf_.suboptimalMethod == 1) {
      //Mean policy
      for (auto itWeight : vecDistanceWeight) {
        voteCount = voteCount + itWeight.second;
      }
      voteCount = voteCount / vecDistanceWeight.size();

    } else if (conf_.suboptimalMethod == 2) {
      //Weighted mean policy
      float totDist = 0;
      for (auto itWeight : vecDistanceWeight) {
        float dist = utilities::distanceEucl(curCenter.x(), itWeight.first.x(), curCenter.y(), itWeight.first.y(), curCenter.z(), itWeight.first.z());

        voteCount = voteCount + (1 / dist) * itWeight.second;
        totDist = totDist + (1 / dist);
      }
      voteCount = voteCount / totDist;

    } else {
      std::cerr << "updateDistanceAndWeight: you choose a wrong policy num" << std::endl;
    }

    itNewCell->info().incrementVoteCountProb(voteCount); // t.n++
    itNewCell->info().markOld();
  }
}

bool ManifoldMeshReconstructor::cellTraversalExitTest(int & f, int & fold, const Delaunay3::Cell_handle& tetCur, const Delaunay3::Cell_handle &tetPrev,
    const Segment & constraint) {

  // Let f be the entry face's index.
  for (int curNeig = 0; curNeig < 4; ++curNeig) {
    if (tetCur->neighbor(curNeig) == tetPrev)
      fold = curNeig;
  }

  int curF = 0;
  bool found = false;
  do {
    if (fold != curF && CGAL::do_intersect(dt_.triangle(tetCur, curF), constraint)) {
      f = curF;
      found = true;
    }
    curF++;
  } while (curF < 4 && !found);

  if (found)
    return true;
  else
    return false;
}

int ManifoldMeshReconstructor::moveVertex_WHeuristic(int idxPoint, int idxCam) {

  std::set<Delaunay3::Cell_handle> setNewCells;
  Delaunay3::Vertex_handle hndlQ = points_[idxPoint].vertexHandle;

  PointD3 initialPosition = hndlQ->point();
  PointD3 pd3NewPoint = points_[idxPoint].position;

  PointD3 camPosition = cams_[idxCam].position;

  if (utilities::distanceEucl(pd3NewPoint, camPosition) < conf_.maxDistanceCamFeature) {

    /********************** Step 1: find the cells incident to the vertex to be removed*****************/
    std::vector<Delaunay3::Cell_handle> setIncidentCells;
    dt_.incident_cells(hndlQ, std::back_inserter(setIncidentCells));

    //store their weights
    std::vector<std::pair<PointD3, float> > vecDistanceWeight, vecDistanceWeight2;
    for (auto it : setIncidentCells) {
      PointD3 temp = CGAL::barycenter(it->vertex(0)->point(), 1.0, it->vertex(1)->point(), 1.0, it->vertex(2)->point(), 1.0, it->vertex(3)->point(), 1.0);
      vecDistanceWeight.push_back(std::pair<PointD3, float>(temp, (float) it->info().getVoteCountProb()));
    }

    /**********************Step 2: raytracing to update the weights before the  point removal****************/
    for (int curCam : points_[idxPoint].viewingCams) {
      Segment QO = Segment(hndlQ->point(), cams_[curCam].position);
      rayTracing(curCam, idxPoint, false, false);
    }

    /************Step 3: remove the point and update the new cells weights according to the information on the
     * weights collected in the previous step****************/
    // stored previously
    std::vector<Delaunay3::Cell_handle> newCells;
    dt_.remove_and_give_new_cells(hndlQ, std::back_inserter(newCells));

    updateDistanceAndWeights(newCells, vecDistanceWeight);

    /***********Step 4: Locate the point and remove conflicting tetrahedra****************/
    Delaunay3::Locate_type lt;
    int li, lj;
    Delaunay3::Cell_handle c = dt_.locate(pd3NewPoint, lt, li, lj);
    if (lt == Delaunay3::VERTEX) {
      std::cerr << "Error in FreespaceDelaunayManifold::moveVertex(): Attempted to move a vertex to an already existing vertex location" << std::endl;
      return false;
    }

    // Get the cells that conflict in a vector vecConflictCells, and a facet on the boundary of this hole in f.
    std::vector<Delaunay3::Cell_handle> vecConflictCells;
    Delaunay3::Facet f;
    dt_.find_conflicts(pd3NewPoint, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

    for (auto it : vecConflictCells) {
      PointD3 temp = CGAL::barycenter(it->vertex(0)->point(), 1.0, it->vertex(1)->point(), 1.0, it->vertex(2)->point(), 1.0, it->vertex(3)->point(), 1.0);
      vecDistanceWeight2.push_back(std::pair<PointD3, float>(temp, (float) it->info().getVoteCountProb()));
    }

    /**********Step 5: Add the new (moved) tets****************/
    hndlQ = dt_.insert_in_hole(pd3NewPoint, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
    points_[idxPoint].vertexHandle = hndlQ;

    /**********Step 6 update the weights of the new tetrahedra****************/
    updateDistanceAndWeights(newCells, vecDistanceWeight2);

    /**********Step 7 raytracing to update the weights after point insertion****************/
    for (int curCam : points_[idxPoint].viewingCams) {
      Segment QO = Segment(hndlQ->point(), cams_[curCam].position);
      rayTracing(curCam, idxPoint, false, true);
    }

    return true;
  } else {
    return false;
  }
}
int ManifoldMeshReconstructor::moveVertex(int idxPoint, int idxCam) {

  std::set<Delaunay3::Cell_handle> setNewCells;
  Delaunay3::Vertex_handle hndlQ = points_[idxPoint].vertexHandle;

  SetConstraints setUnionedConstraints;

  PointD3 initialPosition = hndlQ->point();
  PointD3 pd3NewPoint = points_[idxPoint].position;

  /*check if the move will potentially destroys the manifoldness*/

  PointD3 camPosition = cams_[idxCam].position;

  if (utilities::distanceEucl(pd3NewPoint, camPosition) < conf_.maxDistanceCamFeature) {

    //*********** Step 1: find the cells incident to the vertex to be removed*****************/
    std::set<Delaunay3::Cell_handle> setIncidentCells;
    dt_.incident_cells(hndlQ, std::inserter(setIncidentCells, setIncidentCells.begin()));

    for (auto itCell : setIncidentCells) {
      setUnionedConstraints.insert(itCell->info().getIntersections().begin(), itCell->info().getIntersections().end());
    }

    // Step 2:
    dt_.remove(hndlQ);

    // Step 3:
    // Locate the point
    Delaunay3::Locate_type lt;
    int li, lj;
    Delaunay3::Cell_handle c = dt_.locate(pd3NewPoint, lt, li, lj);
    if (lt == Delaunay3::VERTEX) {
      std::cerr << "Error in FreespaceDelaunayAlgorithm::moveVertex(): Attempted to move a vertex to an already existing vertex location" << std::endl;
      return false;
    }

    // Get the cells that conflict in a vector vecConflictCells, and a facet on the boundary of this hole in f.
    std::vector<Delaunay3::Cell_handle> vecConflictCells;
    Delaunay3::Facet f;
    dt_.find_conflicts(pd3NewPoint, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

    // Get the partitioned unioned constraint sets of all the cells in vecConflictCells.
    for (auto it : vecConflictCells) {
      setUnionedConstraints.insert(it->info().getIntersections().begin(), it->info().getIntersections().end());
    }

    // Step 4
    hndlQ = dt_.insert_in_hole(pd3NewPoint, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
    points_[idxPoint].vertexHandle = hndlQ;

    // Step 6
    for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {
      if (itCell->info().isNew())
        setNewCells.insert(itCell);
      // Linear search:
      for (auto itDelete = itCell->info().getIntersections().begin(); itDelete != itCell->info().getIntersections().end();) {
        if (itDelete->second == idxPoint) {
          // invalidates iterator, so careful about incrementing it:
          std::set<Delaunay3DCellInfo::FSConstraint, Delaunay3DCellInfo::LtFSConstraint>::const_iterator itNext = itDelete;
          itNext++;
          itCell->info().removeIntersection(itDelete->first, itDelete->second, itDelete->vote, points_, camsPositions_);
          itCell->info().decrementVoteCount(1.0);
          itCell->info().decrementVoteCountProb(itDelete->vote);
          itDelete = itNext;
        } else
          itDelete++;
      }
    }

    // Step 7
    for (auto itConstraint : setUnionedConstraints) {
      Segment QO = Segment(points_[itConstraint.second].vertexHandle->point(), cams_[itConstraint.first].position);

      if (itConstraint.second == idxPoint) {
        rayTracing(itConstraint.first, itConstraint.second, false);
      } else {
        rayTracing(itConstraint.first, itConstraint.second, true);
      }

    }

    for (auto itCell : setNewCells)
      itCell->info().markOld();

    return true;
  } else {
    return false;
  }
  return 1;

}
