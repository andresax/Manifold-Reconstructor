//  Copyright 2015 Andrea Romanoni
//
//  This file is part of edgePointSpaceCarver.
//
//  edgePointSpaceCarver is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version see
//  <http://www.gnu.org/licenses/>..
//
//  edgePointSpaceCarver is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

/**
* Header-only file with various types, especially those related to the CGAL library
*/
#ifndef TYPES_MESH_SWEEPING_HPP_
#define TYPES_MESH_SWEEPING_HPP_

#include <sstream>
#include <string>

#include <types_config.hpp>
#include <glm.hpp>

typedef struct {
  std::string pathInitMesh;
  std::string pathCamsPose;
  float thresholdNCC;
  float kSweepingDistance;
  float numPlanes;
  int windowNCC;
  int windowLocalMaxima;
  int rowGrid;
  int colGrid;
  int maxPointsPerCell;
} SweepConfig;

typedef struct {
  std::string pathToSave;
  std::string pathFirstMesh;
  std::string nameFirstMesh;
  std::string nameMesh;
  std::string pathMesh;
  std::string nameDataset;
  std::string pathStatsManifold;
} OutputSweep;

typedef struct Config {
  VideoConfig videoConfig;
  SweepConfig sweepConfig;
  OutputSweep outputSweep;

  std::string toString() {
    std::stringstream totContent;

    totContent << "Video Config " << std::endl;
    totContent << "folderImage " << videoConfig.folderImage << std::endl;
    totContent << "baseNameImage " << videoConfig.baseNameImage << std::endl;
    totContent << "imageExtension " << videoConfig.imageExtension << std::endl;
    totContent << "idxFirstFrame " << videoConfig.idxFirstFrame << std::endl;
    totContent << "digitIdxLength " << videoConfig.digitIdxLength << std::endl;
    totContent << "idxLastFrame " << videoConfig.idxLastFrame << std::endl;
    totContent << "downsampleRate " << videoConfig.downsampleRate << std::endl;
    totContent << "imageH " << videoConfig.imageH << std::endl;
    totContent << "imageW " << videoConfig.imageW << std::endl;

    totContent << "Sweep Config " << std::endl;
    totContent << "pathCamsPose " << sweepConfig.pathCamsPose << std::endl;
    totContent << "thresholdNCC " << sweepConfig.thresholdNCC << std::endl;
    totContent << "kSweepingDistance " << sweepConfig.kSweepingDistance << std::endl;
    totContent << "numPlanes " << sweepConfig.numPlanes << std::endl;
    totContent << "windowNCC " << sweepConfig.windowNCC << std::endl;
    totContent << "windowLocalMaxima " << sweepConfig.windowLocalMaxima << std::endl;
    return totContent.str();
  }
} SweepConfiguration;

typedef struct {
  glm::vec3 point;
  float ncc;
  int idx1;
  int idx2;
}PointNccSweep;



#endif /* TYPES_HPP_ */
