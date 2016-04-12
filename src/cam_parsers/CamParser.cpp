//  Copyright 2014 Andrea Romanoni
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

#include <CamParser.h>

#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>
#include <sstream>
#include <utilities.hpp>

CamParser::CamParser(std::string fileName) : fileName_(fileName), numCameras_(0), numPoints_(0) {
  fileStream_.open(fileName_.c_str(),std::ios::in);
}

CamParser::~CamParser() {
}

bool CamParser::parseFile(const int &downsample) {

  std::string line;

  //read num cam
  std::getline(fileStream_, line);
  std::istringstream iss(line);
  iss >> numCameras_;

  for (int curCam = 0; curCam < numCameras_; curCam = curCam + downsample) {
    CameraType tempCamera;

    //discard the line of cams to be skipped
    for (int var = 0; var < downsample; ++var) {
      std::getline(fileStream_, line);
    }

    iss.str(line);

    char path[1000];
    //std::cout<<"tempCamera.cameraMatrix "<<iss.str()<<std::endl;
    sscanf(iss.str().c_str(),"%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ", path,
        &tempCamera.intrinsics[0][0] , &tempCamera.intrinsics[0][1] , &tempCamera.intrinsics[0][2] ,
        &tempCamera.intrinsics[1][0] , &tempCamera.intrinsics[1][1] , &tempCamera.intrinsics[1][2] ,
        &tempCamera.intrinsics[2][0] , &tempCamera.intrinsics[2][1] , &tempCamera.intrinsics[2][2] ,
        &tempCamera.rotation[0][0] , &tempCamera.rotation[0][1] , &tempCamera.rotation[0][2] ,
        &tempCamera.rotation[1][0] , &tempCamera.rotation[1][1] , &tempCamera.rotation[1][2] ,
        &tempCamera.rotation[2][0] , &tempCamera.rotation[2][1] , &tempCamera.rotation[2][2] ,
        &tempCamera.translation[0] , &tempCamera.translation[1] , &tempCamera.translation[2]);

    camerasPaths_.push_back(std::string(path));
    /*std::cout << "tempCamera.intrinsics" << std::endl;
    utils::printMatrix(tempCamera.intrinsics);
    std::cout << "tempCamera.rotation" << std::endl;
    utils::printMatrix( tempCamera.rotation);
    std::cout << "tempCamera.translation" << std::endl;
    utils::printMatrix( tempCamera.translation);*/


    glm::mat4 tempCameraExtrinsic(0.0);
    tempCameraExtrinsic[0][0] = tempCamera.rotation[0][0];
    tempCameraExtrinsic[0][1] = tempCamera.rotation[0][1];
    tempCameraExtrinsic[0][2] = tempCamera.rotation[0][2];
    tempCameraExtrinsic[1][0] = tempCamera.rotation[1][0];
    tempCameraExtrinsic[1][1] = tempCamera.rotation[1][1];
    tempCameraExtrinsic[1][2] = tempCamera.rotation[1][2];
    tempCameraExtrinsic[2][0] = tempCamera.rotation[2][0];
    tempCameraExtrinsic[2][1] = tempCamera.rotation[2][1];
    tempCameraExtrinsic[2][2] = tempCamera.rotation[2][2];

    tempCameraExtrinsic[0][3] = tempCamera.translation[0];
    tempCameraExtrinsic[1][3] = tempCamera.translation[1];
    tempCameraExtrinsic[2][3] = tempCamera.translation[2];

    glm::mat4 tempCameraIntrinsicH(0.0);
    tempCameraIntrinsicH[0][0] = tempCamera.intrinsics[0][0];
    tempCameraIntrinsicH[0][1] = tempCamera.intrinsics[0][1];
    tempCameraIntrinsicH[0][2] = tempCamera.intrinsics[0][2];
    tempCameraIntrinsicH[1][0] = tempCamera.intrinsics[1][0];
    tempCameraIntrinsicH[1][1] = tempCamera.intrinsics[1][1];
    tempCameraIntrinsicH[1][2] = tempCamera.intrinsics[1][2];
    tempCameraIntrinsicH[2][0] = tempCamera.intrinsics[2][0];
    tempCameraIntrinsicH[2][1] = tempCamera.intrinsics[2][1];
    tempCameraIntrinsicH[2][2] = tempCamera.intrinsics[2][2];
   /* std::cout << "tempCameraIntrinsicH" << std::endl;
    utils::printMatrix( tempCameraIntrinsicH);

    std::cout << "glm::transpose(tempCameraIntrinsicH) * glm::transpose(tempCameraExtrinsic)" << std::endl;
    utils::printMatrix( glm::transpose(tempCameraIntrinsicH) * glm::transpose(tempCameraExtrinsic));

    std::cout << " glm::transpose(tempCameraIntrinsicH*tempCameraExtrinsic)" << std::endl;
    utils::printMatrix( glm::transpose(tempCameraIntrinsicH*tempCameraExtrinsic));
    std::cout << "tempCameraIntrinsicH*tempCameraExtrinsic" << std::endl;
    utils::printMatrix( tempCameraIntrinsicH*tempCameraExtrinsic);*/
    tempCamera.cameraMatrix = glm::transpose(glm::transpose(tempCameraIntrinsicH) * glm::transpose(tempCameraExtrinsic));
   /* std::cout << "tempCamera.cameraMatrix" << std::endl;
    utils::printMatrix( tempCamera.cameraMatrix);*/

   // utils::printMatrix(tempCamera.cameraMatrix);

    tempCamera.center = - tempCamera.translation * glm::transpose(tempCamera.rotation);

    camerasList_.push_back(tempCamera);
  }

  return true;
}
