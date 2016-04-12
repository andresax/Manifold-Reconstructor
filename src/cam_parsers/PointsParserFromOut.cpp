/*
 * PointsParserFromOut.cpp
 *
 *  Created on: 15/apr/2015
 *      Author: andrea
 */

#include <PointsParserFromOut.h>

#include <types_reconstructor.hpp>

PointsParserFromOut::PointsParserFromOut(std::string path, int imageW, int imageH) {
  numPoints_ = -1;

  fileStream_.open(path.c_str(), std::ios::in);
  std::cout<<path.c_str()<<std::endl;
  imageWidth_ = imageW;
  imageHeight_ = imageH;

}

PointsParserFromOut::~PointsParserFromOut() {
}

void PointsParserFromOut::parse(bool filterUnstable) {
  std::string line;
  int numCameras;
  glm::vec3 max=glm::vec3(-10000000.0);
  glm::vec3 min=glm::vec3(10000000.0);

  std::istringstream iss;
  //read and discard the first line (# Bundle file v0.3)
  std::getline(fileStream_, line);
  iss.str(line);

  //read num cam
  std::getline(fileStream_, line);
  iss.str(line);
  iss >> numCameras >> numPoints_;

  camViewingPointN_.assign(numPoints_, std::vector<int>());
  point2DoncamViewingPoint_.assign(numPoints_, std::vector<glm::vec2>());
  pointsVisibleFromCamN_.assign(numCameras, std::vector<int>());

  for (int curPoint = 0; curPoint < numPoints_; ++curPoint) {
    int lengthList, tempKey, tempIdx;
    float tempX, tempY;
    PointParser tempPoint;

    //point's position <x, y, z>
    std::getline(fileStream_, line);
    iss.clear();
    iss.str(line);
    iss >> tempPoint.x >> tempPoint.y >> tempPoint.z;

    if (tempPoint.x < min.x)
      min.x = tempPoint.x;
    if (tempPoint.y < min.y)
      min.y = tempPoint.y;
    if (tempPoint.z < min.z)
      min.z = tempPoint.z;
    if (tempPoint.x > max.x)
      max.x = tempPoint.x;
    if (tempPoint.y > max.y)
      max.y = tempPoint.y;
    if (tempPoint.z > max.z)
      max.z = tempPoint.z;

    std::getline(fileStream_, line);
    iss.clear();
    iss.str(line);
    //view list
    std::getline(fileStream_, line);
    iss.clear();
    iss.str(line);
    iss >> lengthList;
    for (int curViewingCamera = 0; curViewingCamera < lengthList; ++curViewingCamera) {
      iss >> tempIdx;

      camViewingPointN_[curPoint].push_back(tempIdx);
      pointsVisibleFromCamN_[tempIdx].push_back(curPoint);

      iss >> tempKey;
      iss >> tempX;
      tempPoint.viewingCamerasX.push_back(static_cast<float>(imageWidth_) / 2 + tempX);
      iss >> tempY;
      tempPoint.viewingCamerasY.push_back(static_cast<float>(imageHeight_) / 2 - tempY);
      glm::vec2 temp;
      temp.x = tempX;
      temp.y = tempY;
      point2DoncamViewingPoint_[curPoint].push_back(temp);
    }
    glm::vec3 temp;
    temp.x = tempPoint.x;
    temp.y = tempPoint.y;
    temp.z = tempPoint.z;
    points_.push_back(temp);

  }

  std::cout <<"MIN: ("<<min.x<<"," << min.y<<", "<<min.z<<"); MAX: ("<<max.x<<", "<<max.y<<", "<<max.z<<")"<<std::endl;
}
