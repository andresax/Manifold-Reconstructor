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

#include "VideoSimulator.h"
#include <iostream>

VideoSimulator::VideoSimulator(VideoConfig videoConfig) {
  folderImage_    = videoConfig.folderImage;
  baseNameImage_  = videoConfig.baseNameImage;
  firstIdx_       = videoConfig.idxFirstFrame;
  curVideoIndex_  = firstIdx_;
  imageExtension_ = videoConfig.imageExtension;
  digitIdxLength_ = videoConfig.digitIdxLength;
  idxLastFrame_   = videoConfig.idxLastFrame;
  downsampleRate_   = videoConfig.downsampleRate;
}

VideoSimulator::~VideoSimulator() {
}

std::string VideoSimulator::getFrameNumber(int curFrame) {
  std::ostringstream curNumber;
  if (digitIdxLength_ > 0) {
    int n = curFrame;
    int curNumOfDigit = curFrame== 0 ? 1 : 0;
    while (n > 0) {
      n /= 10;
      ++curNumOfDigit;
    }
    while (curNumOfDigit < digitIdxLength_) {
      curNumber << "0";
      curNumOfDigit++;
    }
  }
  curNumber << curFrame;
  return curNumber.str();
}
bool VideoSimulator::getNextFrame(cv::Mat &frame) {
  bool curFrameExists = getFrame(curVideoIndex_, frame);
  curVideoIndex_++;
  return curFrameExists;
}

bool VideoSimulator::getFrame(int numFrame, cv::Mat &frame) {
  if (numFrame <= idxLastFrame_) {
    std::stringstream pathCurImage;
    pathCurImage << folderImage_ << "" << baseNameImage_ << getFrameNumber(numFrame) <<"."<< imageExtension_;
    std::cout<<pathCurImage.str()<<std::endl;
    frame = cv::imread(pathCurImage.str());
    return true;
  } else {
    frame = cv::Mat();
    return false;
  }
}
