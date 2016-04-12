/*
 * PathCreator.cpp
 *
 *  Created on: 22/set/2014
 *      Author: andrea
 */


#include <PathCreator.h>
#include <boost/filesystem.hpp>

PathCreator::PathCreator(SpaceCarvingConfig spaceCarvingConfig, std::string prefix, std::string numKitti) {
  spaceCarvingConfig_ = spaceCarvingConfig;
  prefix_ = prefix;
  numKitti_ = numKitti;
  createDir();
}
PathCreator::PathCreator(std::string numKitti) {
  numKitti_ = numKitti;
}

PathCreator::~PathCreator() {

}

void PathCreator::createDir() {

  std::stringstream pathSave, pathFile, pathEnd;
  pathSave << prefix_ << numKitti_ << "_" << spaceCarvingConfig_.keyFramePeriod << "_" << spaceCarvingConfig_.keyFramePeriodIteration << "_"
      << spaceCarvingConfig_.cannyHighThreshold << "_" << spaceCarvingConfig_.downsamplePeriod << "_" << spaceCarvingConfig_.maxGaussNewtonIteration
      << "_" << spaceCarvingConfig_.minDistBetweenTrackedPoints << "_" << spaceCarvingConfig_.minTrackLength;
  if (spaceCarvingConfig_.edgePointEnabled==1) {
     pathSave << "EdgePoint";
   }  if (spaceCarvingConfig_.edgePointEnabled==2||spaceCarvingConfig_.edgePointEnabled == 0) {
     pathSave << "Harris";
   }  if (spaceCarvingConfig_.edgePointEnabled==3) {
     pathSave << "FAST";
   } else if (spaceCarvingConfig_.edgePointEnabled > 3){
    pathSave << "Nofeatureselected";
  }
  if (spaceCarvingConfig_.inverseConicEnabled) {
    pathSave << "_inverseConicEnabled";
    pathSave << "_" << spaceCarvingConfig_.probOrVoteThreshold;
  } else {
    pathSave << "_NotinverseConicEnabled";
  }
  if (spaceCarvingConfig_.enableSuboptimalPolicy) {
      pathSave << "_withSuboptimal"<< "_" << spaceCarvingConfig_.suboptimalMethod;
    } else {
      pathSave << "_withoutSuboptimal";
    }
  pathRoot_ = pathSave.str();

  pathEnd << prefix_Folder_ <<"/"<<pathSave.str() << "/";

  pathRootDir_ = pathEnd.str();
  boost::filesystem::path dir(pathRootDir_.c_str());
  boost::filesystem::create_directories(dir);
}

std::string PathCreator::getPathToSave() {

  std::stringstream pathFile;
  pathFile << pathRootDir_ << pathRoot_;

  return pathFile.str();
}

std::string PathCreator::getPathLog() {

  std::stringstream pathFile;
  pathFile << pathRootDir_ << pathRoot_<<".txt";

  return pathFile.str();
}

std::string PathCreator::getPathStats() {

  std::stringstream pathFile;
  pathFile << pathRootDir_ << pathRoot_<<"Stats.txt";

  return pathFile.str();
}
std::string PathCreator::getPathStatsManifold() {

  std::stringstream pathFile;
  pathFile << pathRootDir_ << pathRoot_<<"StatsManifold.txt";

  return pathFile.str();
}

std::string PathCreator::getPathLogPoints() {

  std::stringstream pathFile;
  pathFile << pathRootDir_ << pathRoot_<<"LogPoints.txt";

  return pathFile.str();
}

std::string PathCreator::getPathBundlerOut() {
  std::stringstream pathSave;
  pathSave << "examples/kitti_" << numKitti_ << ".out";

  return pathSave.str();
}
std::string PathCreator::getPathKittiOut() {
  std::stringstream pathSave;
  pathSave << "examples/kitti" << numKitti_ << "_rect.out";
  return pathSave.str();
}
std::string PathCreator::getPathFolderImage() {
  std::stringstream pathSave;
  pathSave << "examples/" << numKitti_ << "/image_00/data/";


  return pathSave.str();
}
