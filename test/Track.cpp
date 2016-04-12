/*
 * Track.cpp
 *
 *  Created on: 14/gen/2015
 *      Author: andrea
 */

#include "Track.h"

Track::Track() {
  idInTriangulation_ = -1;
  notYetEstimated = true;
  eta_ = 0;
  lastEstimateDone_ = false;
  trackEnded_ = false;
  inserted_ = false;
}
Track::Track(cv::Point2f point, int camId, int idInTriangulation){

  idInTriangulation_ = idInTriangulation;
  track_.push_back(std::pair<cv::Point2f, int>(point, camId));
  notYetEstimated = true;
  eta_ = 0;
  lastEstimateDone_ = false;
  trackEnded_ = false;
  inserted_ = false;
}

Track::Track(cv::Point2f point, int camId){

  idInTriangulation_ = -1;
  track_.push_back(std::pair<cv::Point2f, int>(point, camId));
  notYetEstimated = true;
  eta_ = 0;
  lastEstimateDone_ = false;
  trackEnded_ = false;
  inserted_ = false;
}

Track::~Track() {
}

size_t Track::getTrackLength() {

  return track_.size();
}

void Track::add2DMeasure(cv::Point2f point, int camId) {

  track_.push_back(std::pair<cv::Point2f, int>(point, camId));
}

std::pair<cv::Point2f, int> Track::getLastFrameMeasure() {

  return track_.back();
}

std::pair<cv::Point2f, int> Track::getMeasure(int i) {
  return track_.at(i);
}

std::pair<cv::Point2f, int> Track::getMeasureCam(int i) {
  std::pair<cv::Point2f, int> temp;
  //TODO
  return temp;
}

std::string Track::toString() {
  std::stringstream s;

  s <<"Id " << idInTriangulation_;
  for (auto it = track_.begin(); it != track_.end(); ++it) {

    s <<"; Camera num " << it->second;
    s <<", point x " << it->first.x;
    s <<", point y " << it->first.y;
  }

  s <<", is not estimated yet " << notYetEstimated;
  s <<", is trackEnded_ " << trackEnded_;
  s <<", is lastEstimateDone_ " << lastEstimateDone_;
  s <<", is inserted_ " << inserted_;


  return s.str();
}
