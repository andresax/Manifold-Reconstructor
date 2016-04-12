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


#ifndef VIDEOSIMULATOR_H_
#define VIDEOSIMULATOR_H_

// This class provides a friendly interface to deal with a list of images that
// represent a video sequence.

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <types_reconstructor.hpp>
#include <types_config.hpp>

class VideoSimulator {
  public:
    VideoSimulator(VideoConfig videoConfig);
    virtual ~VideoSimulator();

    bool getNextFrame(cv::Mat &frame);
    bool getFrame(int numFrame, cv::Mat &frame);

    int getCurVideoIndex() const {
      return curVideoIndex_;
    }

    int getFirstIdx() const {
      return firstIdx_;
    }

    int getIdxLastFrame() const {
      return idxLastFrame_;
    }

//TODO add color management
  private:
    std::string folderImage_;
    std::string baseNameImage_;
    std::string imageExtension_;
    int curVideoIndex_;
    int firstIdx_;
    int digitIdxLength_;
    int idxLastFrame_;
    int downsampleRate_;
    std::string getFrameNumber( int curFrame );
};

#endif /* VIDEOSIMULATOR_H_ */
