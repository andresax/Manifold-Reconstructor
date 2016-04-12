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

#include "EdgePointSpaceCarver.h"
#include <iostream>
#include <list>
#include <fstream>
#include <drawingUtilities.hpp>
#include <conversionUtilities.hpp>
#include <omp.h>
#include <sys/time.h>
#include <sstream>
#include <opencv2/core/eigen.hpp>
#include <utilities.hpp>

// Domain

EdgePointSpaceCarver::EdgePointSpaceCarver(Configuration configuration, std::vector<CameraRect> cameras) :
    video_(configuration.videoConfig) {
  configuration_ = configuration;

  ManifoldReconstructionConfig confManif;
  confManif.enableSuboptimalPolicy = configuration.spaceCarvingConfig.enableSuboptimalPolicy;
  confManif.inverseConicEnabled = configuration.spaceCarvingConfig.inverseConicEnabled;
  confManif.maxDistanceCamFeature = configuration.spaceCarvingConfig.maxDistanceCamFeature;
  confManif.probOrVoteThreshold = configuration.spaceCarvingConfig.probOrVoteThreshold;
  confManif.suboptimalMethod = configuration.spaceCarvingConfig.suboptimalMethod;
  confManif.w_1 = configuration.spaceCarvingConfig.w_1;
  confManif.w_2 = configuration.spaceCarvingConfig.w_2;
  confManif.w_3 = configuration.spaceCarvingConfig.w_3;

  manifRec_ = new ManifoldMeshReconstructor(confManif);

  cameras_ = cameras;

  imageW_ = configuration.videoConfig.imageW;
  imageH_ = configuration.videoConfig.imageH;

  for (std::vector<CameraRect>::iterator curCam = cameras_.begin(); curCam != cameras_.end(); ++curCam) {
    cv::Mat K, E;
    kittiRectParamToCalibMatrix(*curCam, K, E);
    //in this case K is the identity matrix and E encodes the whole projection matrix
    camerasK_.push_back(K);
    camerasE_.push_back(E);
    camerasP_.push_back(E);
  }
  keyFramePeriod_ = configuration.spaceCarvingConfig.keyFramePeriod;
  keyFramePeriodIteration_ = configuration.spaceCarvingConfig.keyFramePeriodIteration;
  cannyHighThreshold_ = configuration.spaceCarvingConfig.cannyHighThreshold;
  downsamplePeriod_ = configuration.spaceCarvingConfig.downsamplePeriod;
  minDistBetweenTrackedPoints_ = configuration.spaceCarvingConfig.minDistBetweenTrackedPoints;
  minTrackLength_ = configuration.spaceCarvingConfig.minTrackLength;
  currentFrameNumber_ = 0;
  idPoint_ = 0;
  minCam_ = video_.getIdxLastFrame();    //last cameras without current tracked points

#ifdef LOGPOINTS
  fileOut.open(configuration.outputSpaceCarving.pathLogPoints.c_str());
#endif
#ifdef LOGGING
  fileLog.open(configuration.outputSpaceCarving.pathLog.c_str());
  fileStats_.open(configuration.outputSpaceCarving.pathStats.c_str());
#endif
#ifdef LOGGING_VERBOSE
  fileOutVerbose.open("logPointsVerbose");
#endif

  lastCam_ = -1;    //tracks the last cameras without current tracked points

  totLen_ = 0;
  numLen_ = 0;
#ifdef LOGGING_DISTRIBUTION
  std::vector<int> temp(5, 0);
  grid_.assign(3, temp);
#endif
}

EdgePointSpaceCarver::~EdgePointSpaceCarver() {
  fileOut.close();
  fileOutVerbose.close();
}

void EdgePointSpaceCarver::bundlerParamToCalibMatrix(SensorParser camBundler, cv::Mat &K, cv::Mat &E) {
  E =
      (cv::Mat_<float>(3, 4) << camBundler.R(0, 0), camBundler.R(0, 1), camBundler.R(0, 2), camBundler.t(0), camBundler.R(1, 0), camBundler.R(1, 1), camBundler.R(
          1, 2), camBundler.t(1), camBundler.R(2, 0), camBundler.R(2, 1), camBundler.R(2, 2), camBundler.t(2));
  assert(imageH_ != 0);

  K = (cv::Mat_<float>(3, 3) << -camBundler.f, 0, imageW_ / 2, 0, camBundler.f, imageH_ / 2, 0, 0, 1);
}

void EdgePointSpaceCarver::kittiParamToCalibMatrix(Camera camBundler, cv::Mat &K, cv::Mat &E) {
  E =
      (cv::Mat_<float>(3, 4) << camBundler.R(0, 0), camBundler.R(0, 1), camBundler.R(0, 2), camBundler.t(0), camBundler.R(1, 0), camBundler.R(1, 1), camBundler.R(
          1, 2), camBundler.t(1), camBundler.R(2, 0), camBundler.R(2, 1), camBundler.R(2, 2), camBundler.t(2));

  K = (cv::Mat_<float>(3, 3) << camBundler.fx, 0, camBundler.cx, 0, camBundler.fy, camBundler.cy, 0, 0, 1);

}

void EdgePointSpaceCarver::kittiRectParamToCalibMatrix(CameraRect camBundler, cv::Mat &K, cv::Mat &E) {
  E =
      (cv::Mat_<float>(3, 4) << camBundler.P(0, 0), camBundler.P(0, 1), camBundler.P(0, 2), camBundler.P(0, 3), camBundler.P(1, 0), camBundler.P(1, 1), camBundler.P(
          1, 2), camBundler.P(1, 3), camBundler.P(2, 0), camBundler.P(2, 1), camBundler.P(2, 2), camBundler.P(2, 3));

  //std::cout << E << std::endl;
  K = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

}

void EdgePointSpaceCarver::printTracksStuff() {
  int countLastEstimate = 0, notYetEstimatedcount = 0, trackEndedcount = 0;
  for (auto t : activeTracks_) {
    if (t.isLastEstimateDone()) {
      countLastEstimate++;
    }
    if (t.isNotYetEstimated()) {
      notYetEstimatedcount++;
    }
    if (t.isTrackEnded()) {
      trackEndedcount++;
    }
  }
  std::cout << "active: " << activeTracks_.size();
  std::cout << " countLastEstimate " << countLastEstimate;
  std::cout << " notYetEstimatedcount " << notYetEstimatedcount;
  std::cout << " trackEndedcount " << trackEndedcount << std::endl;
}
void EdgePointSpaceCarver::trackPointsFrameByFrame() {
  std::vector<unsigned char> statusReal;
  std::vector<float> err;
#ifdef LOGGING
  struct timeval start, end;
  gettimeofday(&start, NULL);
  std::cout << "Track " << activeTracks_.size() << " points with KLT...";
  fileLog << "Tracking " << activeTracks_.size() << " ";
  fileStats_ << "Before Tracking(active) " << activeTracks_.size() << std::endl;
  std::cout << "Before Tracking(active) " << std::endl;
  // printTracksStuff();
#endif

  prevTrackedPoints_.clear();
  curTrackedPoints_.clear();
  if (activeTracks_.size() > 0) {
    for (std::vector<Track>::iterator curActiveTrack = activeTracks_.begin(); curActiveTrack != activeTracks_.end(); ++curActiveTrack) {
      if (curActiveTrack->isTrackEnded() == false) {
        prevTrackedPoints_.push_back(curActiveTrack->getLastFrameMeasure().first);
      }
    }
    std::cout << "prevTrackedPoints_ " << prevTrackedPoints_.size() << " points with KLT...";

    fileStats_ << "Before Tracking(prevTrackedPoints_) " << prevTrackedPoints_.size() << std::endl;

    if (prevTrackedPoints_.size() > 0)
      cv::calcOpticalFlowPyrLK(previousFrame_, currentFrame_, prevTrackedPoints_, curTrackedPoints_, statusReal, err, cv::Size(31, 31), 4,
          cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.1));
#ifdef LOGGING
    gettimeofday(&end, NULL);
    float delta = ((end.tv_sec - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
    std::cout << "DONE. (" << delta << "s )" << std::endl;
    fileLog << delta << std::endl;
    int countOk1 = 0;
    for (size_t curP = 0; curP < statusReal.size(); ++curP) {
      if (statusReal[curP] == 1)
        countOk1++;
    }
    fileStats_ << "After Tracking(curTrackedPoints_) " << curTrackedPoints_.size() << std::endl;

    std::cout << "After Tracking " << std::endl;
    //printTracksStuff();
#endif

#ifdef DRAW_TRACKING
    //drawingUtilities::drawTrackedPoint(previousFrame_, prevTrackedPoints_, currentFrame_, curTrackedPoints_,statusReal);
#endif

    trackRefinementWithEpipolar(statusReal);
    int countOk = 0;
    size_t curP = 0;
    for (size_t curT = 0; curT < activeTracks_.size(); ++curT) {
      if (!activeTracks_[curT].isTrackEnded()) {
        if (activePointsIndices_[curP] == 0) {
          activeTracks_[curT].setTrackEnded(true);
        } else {
          activeTracks_[curT].add2DMeasure(curTrackedPoints_[curP], currentFrameNumber_);
          ++countOk;
        }
        ++curP;
      }
    }

    fileStats_ << "After Filtering(curTrackedPoints_) " << countOk << std::endl;
    std::cout << "After Filtering " << std::endl;
    //printTracksStuff();
#ifdef DRAW_TRACKING
    std::string path;
    path = utilities::getFrameNumber(currentFrameNumber_,10)+".png";
    drawingUtilities::drawCorrespondences(previousFrame_, prevTrackedPoints_, currentFrame_, curTrackedPoints_, activePointsIndices_,path);
#endif
    // drawingUtilities::drawCorrespondences(previousFrame_, prevTrackedPoints_, currentFrame_, curTrackedPoints_, activePointsIndices_);

    //trackRefinementWithBacktrackingAndFiltering(statusReal);

    //drawingUtilities::drawCorrespondences(previousFrame_, prevTrackedPoints_, currentFrame_, curTrackedPoints_, activePointsIndices_);
  }
}
void EdgePointSpaceCarver::trackRefinementWithEpipolar(std::vector<unsigned char> status) {
  if (status.size() > 0) {
    std::vector<cv::Vec<float, 3> > epilines;
    std::vector<float> epipolarOk(status.size(), 0.0);
#ifdef LOGGING
    struct timeval start, end;
    gettimeofday(&start, NULL);
    std::cout << "Filter points tracked with KLT with epipolar constraint ...";
    fileLog << "Filtering " << activeTracks_.size() << " ";
#endif
    cv::Mat F, P1, P2;
    cv::eigen2cv(cameras_[currentFrameNumber_ - 1].P, P1);
    cv::eigen2cv(cameras_[currentFrameNumber_].P, P2);

    fromPtoF(P2, P1, F);
    cv::computeCorrespondEpilines(prevTrackedPoints_, 1, F, epilines); //Index starts with 1

    activePointsIndices_ = status;

#ifdef OPEN_MP_ENABLED
#pragma omp parallel for
#endif
    for (size_t curStatus = 0; curStatus < activePointsIndices_.size(); ++curStatus) {
      cv::Point2f curPrevTrackedPoints = prevTrackedPoints_[curStatus];
      cv::Point2f curCurTrackedPoints = curTrackedPoints_[curStatus];

      epipolarOk[curStatus] = drawingUtilities::distancePointLine(curCurTrackedPoints, epilines[curStatus]);

      activePointsIndices_[curStatus] = 0;
      float distanceBetweenPt = sqrt(
          (curCurTrackedPoints.x - curPrevTrackedPoints.x) * (curCurTrackedPoints.x - curPrevTrackedPoints.x)
              + (curCurTrackedPoints.y - curPrevTrackedPoints.y) * (curCurTrackedPoints.y - curPrevTrackedPoints.y));
      //float coeff = distanceBetweenPt / 10;
      if (status[curStatus] == 1 && epipolarOk[curStatus] < configuration_.spaceCarvingConfig.maxEpipolarDist) {

        if (distanceBetweenPt > minDistBetweenTrackedPoints_) {
          activePointsIndices_[curStatus] = 1;
        }
      }
    }

#ifdef LOGGING
    gettimeofday(&end, NULL);
    float delta = ((end.tv_sec - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
    fileLog << delta << std::endl;
    std::cout << "DONE. (" << delta << " sec)" << std::endl;
#endif

    //#ifdef DRAW
    // drawingUtilities::drawTrackedPoints(previousFrame_, prevTrackedPoints_, currentFrame_, curTrackedPoints_,activePointsIndices_,err);
    //#endif

    //drawingUtilities::myDrawEpipolarLinesfloat("Epipolar Lines", F1, previousFrame_, currentFrame_, prevTrackedPoints_, curTrackedPoints_);
#ifdef DRAW
    //cv::Matx<float, 3, 3> F1 = F;
    //cv::Matx<float, 3, 3> F1 = F;

    //drawingUtilities::myDrawEpipolarLines("Epipolar Lines", F1, previousFrame_, currentFrame_, prevTrackedPoints_, curTrackedPoints_);
    //drawingUtilities::drawEpipolarLines("Epipolar Lines2",F1, previousFrame_, currentFrame_, prevTrackedPoints_, curTrackedPoints_);
    cv::Mat matTemp;
    //drawingUtilities::drawFilteredPointWithEpipolar(previousFrame_, prevTrackedPoints_, currentFrame_, curTrackedPoints_, matTemp, epipolarOk);

    drawingUtilities::drawCorrespondences(previousFrame_, prevTrackedPoints_, currentFrame_, curTrackedPoints_, activePointsIndices_);
#endif
  }
}
void EdgePointSpaceCarver::trackRefinementWithBacktrackingAndFiltering(std::vector<unsigned char> statusReal) {
  std::vector<cv::Point2f> testTrackedPoints;
  std::vector<unsigned char> status, statusTemp, statusTemp2, epipolarOk;
  std::vector<float> err;
#ifdef LOGGING
  struct timeval start, end;
  gettimeofday(&start, NULL);
  std::cout << "Filter points tracked with KLT with backtracking...";
  fileLog << "Filtering " << activeTracks_.size() << " ";
#endif
  cv::Mat F, P1, P2;

  cv::calcOpticalFlowPyrLK(currentFrame_, previousFrame_, curTrackedPoints_, testTrackedPoints, status, err, cv::Size(31, 31), 4,
      cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.1));

  activePointsIndices_ = statusReal;
  statusTemp = statusReal;
  statusTemp2 = statusReal;
#ifdef OPEN_MP_ENABLED
#pragma omp parallel for
#endif
  for (size_t curStatus = 0; curStatus < activePointsIndices_.size(); ++curStatus) {
    cv::Point2f curPrevTrackedPoints = prevTrackedPoints_[curStatus];
    cv::Point2f curCurTrackedPoints = curTrackedPoints_[curStatus];
    cv::Point2f curTestTrackedPoints = testTrackedPoints[curStatus];

    activePointsIndices_[curStatus] = 0;
    statusTemp[curStatus] = 0;
    statusTemp2[curStatus] = 0;
    if (status[curStatus] == 1 && statusReal[curStatus] == 1) {
      float distanceBackpr = sqrt(
          (curTestTrackedPoints.x - curPrevTrackedPoints.x) * (curTestTrackedPoints.x - curPrevTrackedPoints.x)
              + (curTestTrackedPoints.y - curPrevTrackedPoints.y) * (curTestTrackedPoints.y - curPrevTrackedPoints.y));
      if (distanceBackpr < 1) {

        float distanceBetweenPt = sqrt(
            (curCurTrackedPoints.x - curPrevTrackedPoints.x) * (curCurTrackedPoints.x - curPrevTrackedPoints.x)
                + (curCurTrackedPoints.y - curPrevTrackedPoints.y) * (curCurTrackedPoints.y - curPrevTrackedPoints.y));
        if (distanceBetweenPt > minDistBetweenTrackedPoints_) {
          activePointsIndices_[curStatus] = 1;
          statusTemp2[curStatus] = 1;
          statusTemp[curStatus] = 1;
        }
      } else {
        float distanceBetweenPt = sqrt(
            (curCurTrackedPoints.x - curPrevTrackedPoints.x) * (curCurTrackedPoints.x - curPrevTrackedPoints.x)
                + (curCurTrackedPoints.y - curPrevTrackedPoints.y) * (curCurTrackedPoints.y - curPrevTrackedPoints.y));
        if (distanceBetweenPt > minDistBetweenTrackedPoints_) {
          statusTemp2[curStatus] = 1;
        }
      }
    } else {
      if (statusReal[curStatus] == 1) {
        float distanceBetweenPt = sqrt(
            (curCurTrackedPoints.x - curPrevTrackedPoints.x) * (curCurTrackedPoints.x - curPrevTrackedPoints.x)
                + (curCurTrackedPoints.y - curPrevTrackedPoints.y) * (curCurTrackedPoints.y - curPrevTrackedPoints.y));
        if (distanceBetweenPt > minDistBetweenTrackedPoints_) {
          statusTemp2[curStatus] = 1;
        }
      }
    }
  }

#ifdef LOGGING
  gettimeofday(&end, NULL);
  float delta = ((end.tv_sec - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
  fileLog << delta << std::endl;
  std::cout << "DONE. (" << delta << " sec)" << std::endl;
#endif
  //#ifdef DRAW
  //  cv::Mat matTemp;
  //  drawingUtilities::drawTrackedPoints2(previousFrame_, prevTrackedPoints_, currentFrame_, curTrackedPoints_,matTemp,statusTemp2,err, statusReal);
  //#endif
  //#ifdef DRAW
  //  drawingUtilities::drawTrackedPoints3(previousFrame_, prevTrackedPoints_, currentFrame_, curTrackedPoints_,matTemp,statusTemp,err,statusReal);
  //#endif
}

void EdgePointSpaceCarver::updateTracks() {

#ifdef LOGGING
  struct timeval start, end;
  gettimeofday(&start, NULL);
  std::cout << "Look for ended tracks ...";
  fileLog << "EndedTracks ";

  std::cout << "Before update " << std::endl;
  //printTracksStuff();
#endif
  for (std::vector<Track>::iterator curActiveTrack = activeTracks_.begin(); curActiveTrack != activeTracks_.end();) {
    //std::cout << "updateTracks::CurTracked PointID: " << curTrackedPointID << std::endl;
    if (curActiveTrack->isTrackEnded() && (curActiveTrack->isLastEstimateDone() || curActiveTrack->track_.size() < (size_t) minTrackLength_)) {
      curActiveTrack = activeTracks_.erase(curActiveTrack);
    } else {
      ++curActiveTrack;
    }
  }
  std::cout << "After update " << std::endl;
  //printTracksStuff();

#ifdef LOGGING
  gettimeofday(&end, NULL);
  float delta = ((end.tv_sec - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
  std::cout << "DONE. (" << endedTracks_.size() << " tracks ended). (" << delta << " sec)" << std::endl;
  fileLog << endedTracks_.size() << " " << delta << std::endl;
#endif
}

void EdgePointSpaceCarver::estimate3Dpositions() {
#ifdef LOGGING
  std::cout << "Estimating the 3D positions...";
  fileLog << "Estimating3Dpositions ";
  struct timeval start, end;
  gettimeofday(&start, NULL);
  int countPt = 0;
#endif
  minCam_ = video_.getIdxLastFrame();
#ifdef OPEN_MP_ENABLED
  // #pragma omp parallel for
  //TODO remove comment from parallel for
#endif
  for (size_t curTrackNumber = 0; curTrackNumber < activeTracks_.size(); ++curTrackNumber) {
    //I don't use iterators to enable a simpler management of the OpenMP parallel for
    Track *currentTrack = &activeTracks_.at(curTrackNumber);
    //if (curTrackNumber%10 == 0) std::cout<<currentTrack->toString()<<std::endl;
    if (currentTrack->getTrackLength() >= (size_t) minTrackLength_
#ifdef WITHOUT_MOVING_POINTS
        && currentTrack->isTrackEnded()
#endif
        ) {
      //***********************************************TRIANGULATE**********************************************//
      cv::Vec4f triangulated3DPointInitTemp;
      cv::Point3f triangulated3DPointInit, triangulated3DPoint;
      std::vector<cv::Mat> curCams;
      std::vector<cv::Point2f> curPoints;
      int firstCamIdx, lastCamIdx;
      cv::vector<cv::Point2f> firstPositionVec, lastPositionVec;
      cv::Point2f firstPosition, secondPosition;

      firstCamIdx = currentTrack->getMeasure(0).second;
      firstPositionVec.push_back(currentTrack->getMeasure(0).first);

      lastCamIdx = currentTrack->getLastFrameMeasure().second;
      lastPositionVec.push_back(currentTrack->getLastFrameMeasure().first);
      cv::triangulatePoints(camerasP_[firstCamIdx], camerasP_[lastCamIdx], firstPositionVec, lastPositionVec, triangulated3DPointInitTemp);
      triangulated3DPointInit.x = triangulated3DPointInitTemp[0] / triangulated3DPointInitTemp[3];
      triangulated3DPointInit.y = triangulated3DPointInitTemp[1] / triangulated3DPointInitTemp[3];
      triangulated3DPointInit.z = triangulated3DPointInitTemp[2] / triangulated3DPointInitTemp[3];

      //Pack the information to start the Gauss Newton algorithm
      for (auto it = currentTrack->track_.begin(); it != currentTrack->track_.end(); ++it) {
        /* std::cout<<"Point " << it->first;
         std::cout<<"; corresponding Cam " << it->second<<std::endl;*/
        cv::Mat temp;
        camerasP_[it->second].convertTo(temp, CV_32F);
        curCams.push_back(temp);
        curPoints.push_back(it->first);
        if (it->second < minCam_) {
          minCam_ = it->second;
        }
      }
      //***********************************************Gauss-Newton**********************************************//
      int resGN = GaussNewton(curCams, curPoints, triangulated3DPointInit, triangulated3DPoint);
      if (currentTrack->isTrackEnded()) {
        currentTrack->setLastEstimateDone(true);
      }
      if (resGN != -1) {

        //**************3D point Insertion/update in the space carver**********************************/
        //*NB: actually, at this stage we do not add the points to the 3D triangulation***************//
        //*but we only tell the space carver that new points or moved points are available************//
        //*and where they are located*****************************************************************//

#ifndef WITHOUT_MOVING_POINTS
        if (currentTrack->isNotYetEstimated()) {
          //}//remove this braket, needed only for atuo-indentation reasons

#endif
#ifdef WITHOUT_MOVING_POINTS
        if (currentTrack->isTrackEnded()) {
#endif
#ifdef OPEN_MP_ENABLED
#pragma omp critical
#endif
          {
            /*Point "insertion"*/
            int r = manifRec_->addPointWhere(triangulated3DPoint.x, triangulated3DPoint.y, triangulated3DPoint.z);
            currentTrack->setIdInTriangulation(r);
            ++idPoint_;

            for (auto it = currentTrack->track_.begin(); it != currentTrack->track_.end(); ++it) {
              manifRec_->addVisibilityPair(it->second, currentTrack->getIdInTriangulation());
            }
            countPt++;
          }

          currentTrack->setInserted(true);
          currentTrack->setWorldPosition(triangulated3DPoint);
        }
#ifndef WITHOUT_MOVING_POINTS
        else {
          msc::Matrix old = spaceCarver.movePointGetOld(conversionUtilities::toLoviMatrix(triangulated3DPoint), currentTrack->getIdInTriangulation());

#ifdef LOGPOINTSMOVE
          fileOut2 << triangulated3DPoint.x << " " << triangulated3DPoint.y
          << " " << triangulated3DPoint.z << " "
          << old[0] << " " << old[1]
          << " " << old[2] <<std::endl;

          fileOut2 << std::endl;
#endif
          ///is it needed?
          spaceCarver.addVisibilityPair(currentTrack->track_.back().second, currentTrack->getIdInTriangulation());

          currentTrack->incrementEta();
          currentTrack->setWorldPosition(triangulated3DPoint);
        }
#endif

        if (currentTrack->isTrackEnded()) {
          currentTrack->setLastEstimateDone(true);
        }
#ifdef LOGPOINTS
        fileOut << triangulated3DPoint.x << ", " << triangulated3DPoint.y
        << ", " << triangulated3DPoint.z << ", 1.0 ";
        for (auto it = currentTrack->track_.begin(); it != currentTrack->track_.end(); ++it) {
          fileOut << it->second + 1 << ", ";
        }

        fileOut << std::endl;
#endif

#ifdef LOGGING_VERBOSE
        fileOutVerbose << "Point num. " << idPoint_ <<std::endl;
        fileOutVerbose << "Triangulated3DPointInit: " << std::endl;
        fileOutVerbose << "x: "<< triangulated3DPointInit.x;
        fileOutVerbose << ", y: "<< triangulated3DPointInit.y;
        fileOutVerbose << ", z: "<< triangulated3DPointInit.z << std::endl;
        fileOutVerbose << "Triangulated3DPoint: " << std::endl;
        fileOutVerbose << "x: "<< triangulated3DPoint.x;
        fileOutVerbose << ", y: "<< triangulated3DPoint.y;
        fileOutVerbose << ", z: "<< triangulated3DPoint.z << std::endl;
        fileOutVerbose << "Measures. " << std::endl;
        for (Track::iterator it = currentTrack->begin(); it != currentTrack->end(); ++it) {
          fileOutVerbose << it->second << ": (" << it->first.x << ", " << it->first.y <<")" << std::endl;
        }
        fileOutVerbose << std::endl;
#endif

      } else {
#ifdef WITHOUT_MOVING_POINTS
        currentTrack->setLastEstimateDone(true);
#endif
      }
    } else {

      currentTrack->setInserted(false);
    }

    //if (curTrackNumber%10 == 0) std::cout<<currentTrack->toString()<<std::endl;
  }

#ifdef LOGGING
  gettimeofday(&end, NULL);
  float delta = ((end.tv_sec - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
  std::cout << "DONE (" << delta << "sec)." << std::endl;
  fileLog << delta << " " << countPt << std::endl;
  fileStats_ << "3DPointEstimate " << countPt << " / " << endedTracks_.size() << std::endl;
#endif
}

void EdgePointSpaceCarver::extractEdgePoints() {
  cv::Mat currentEdgesImage, currentFrameSmoothed;
  std::vector<std::vector<cv::Point> > connectedEdges;
#ifdef DRAW
  std::vector<cv::Point2f> downsampledEdges;
#endif
#ifdef LOGGING
  std::cout << "Extract edge-points: ";
  clock_t begin_extractPoints = clock();
  fileLog << "ExtractEdgePoints ";
#endif
  cv::GaussianBlur(currentFrame_, currentFrameSmoothed, cv::Size(7, 7), sqrt(2));
  cv::Canny(currentFrameSmoothed, currentEdgesImage, 0.4 * cannyHighThreshold_, cannyHighThreshold_);
  cv::findContours(currentEdgesImage, connectedEdges, cv::noArray(), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

#ifdef LOGGING_VERBOSE
  int curcontourId = 0;
#endif

  for (std::vector<std::vector<cv::Point> >::iterator curContour = connectedEdges.begin(); curContour != connectedEdges.end(); ++curContour) {
    int curpointN = 0;
#ifdef LOGGING_VERBOSE
    fileOutVerbose << "Contour: "<<curcontourId;
#endif
    for (std::vector<cv::Point>::iterator curPoint = curContour->begin(); curPoint != curContour->end(); ++curPoint) {

      if (curpointN % downsamplePeriod_ == 0) {
#ifdef DRAW
        downsampledEdges.push_back(*curPoint);
#endif

        activeTracks_.push_back(Track(*curPoint, currentFrameNumber_));
        //++idPoint_;
#ifdef LOGGING_VERBOSE
        fileOutVerbose << " point: "<<curpointN;
        fileOutVerbose << "  ("<<curPoint->x<<", "<<curPoint->y << "); ";
#endif

      }

      ++curpointN;
    }

#ifdef LOGGING_VERBOSE
    fileOutVerbose << std::endl;
    ++curcontourId;
#endif
  }

#ifdef LOGGING
  fileLog << float(clock() - begin_extractPoints) / CLOCKS_PER_SEC << std::endl;
  std::cout << "DONE (takes " << float(clock() - begin_extractPoints) / CLOCKS_PER_SEC << "sec," << connectedEdges.size() << " points extracted). "
      << std::endl;

#endif
#ifdef DRAW
  drawingUtilities::drawPoints(currentEdgesImage, downsampledEdges);
#endif
}

void EdgePointSpaceCarver::extractEdgePointsSimple() {
#ifdef DRAW
  std::vector<cv::Point2f> downsampledEdges;
#endif
#ifdef LOGGING
  std::cout << "Extract edge-points: ";
  clock_t begin_extractPoints = clock();
  fileLog << "ExtractEdgePoints ";
#endif
  cv::Mat curFrameBW;
  cv::Mat currentEdgesImage;

  int curpointN = 0;
  if (configuration_.spaceCarvingConfig.edgePointEnabled == 1) {

    cv::Mat currentFrameSmoothed;
    std::vector<std::vector<cv::Point> > connectedEdges;

    cv::GaussianBlur(currentFrame_, currentFrameSmoothed, cv::Size(7, 7), sqrt(2));
    cv::Canny(currentFrameSmoothed, currentEdgesImage, 0.4 * cannyHighThreshold_, cannyHighThreshold_);
    cv::Mat nonZeroCoordinates;
    cv::findNonZero(currentEdgesImage, nonZeroCoordinates);

    for (size_t i = 0; i < nonZeroCoordinates.total(); i = i + downsamplePeriod_) {
      cv::Point curPoint(nonZeroCoordinates.at<cv::Point>(i).x, nonZeroCoordinates.at<cv::Point>(i).y);

      activeTracks_.push_back(Track(curPoint, currentFrameNumber_));
      //++idPoint_;
#ifdef DRAW
      downsampledEdges.push_back(curPoint);
#endif
      ++curpointN;
    }
#ifdef LOGGING_DISTRIBUTION

    int numr = grid_.size();
    int numc = grid_[0].size();

    for (size_t i = 0; i < nonZeroCoordinates.total(); i = i + downsamplePeriod_) {

      int positionX = numc * nonZeroCoordinates.at<cv::Point>(i).x / imageW_;
      int positionY = numr * nonZeroCoordinates.at<cv::Point>(i).y / imageH_;
      grid_[positionY][positionX]++;

    }
#endif

#ifdef DRAW

    cv::cvtColor(currentFrame_, curFrameBW, CV_BGR2GRAY);
//    curFrameBW =currentEdgesImage;
#endif
  } else {
    std::vector<cv::Point2f> corners;
    std::vector<cv::KeyPoint> cornersKP;
    float qualityLevel = 0.001;
    float minDistance = 2;
    int blockSize = 3;
    bool useHarrisDetector = true;
    float k = 0.04;
    int maxCorners = 8000;

    cv::cvtColor(currentFrame_, curFrameBW, CV_BGR2GRAY);

    /// Apply corner detection
    if (configuration_.spaceCarvingConfig.edgePointEnabled == 2 || configuration_.spaceCarvingConfig.edgePointEnabled == 0) {
      cv::goodFeaturesToTrack(curFrameBW, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector, k);
    } else if (configuration_.spaceCarvingConfig.edgePointEnabled == 3) {

      cv::FAST(curFrameBW, cornersKP, 25);
      cv::KeyPoint::convert(cornersKP, corners);
    }

    for (size_t i = 0; i < corners.size(); ++i) {
      activeTracks_.push_back(Track(corners[i], currentFrameNumber_));

#ifdef DRAW
      downsampledEdges.push_back(corners[i]);
#endif
      ++curpointN;
    }
#ifdef LOGGING_DISTRIBUTION

    int numr = grid_.size();
    int numc = grid_[0].size();

    for (size_t i = 0; i < corners.size(); ++i) {

      int positionX = numc * corners[i].x / imageW_;
      int positionY = numr * corners[i].y / imageH_;
      grid_[positionY][positionX]++;
    }

#endif

  }
#ifdef LOGGING
  fileLog << float(clock() - begin_extractPoints) / CLOCKS_PER_SEC << " " << curpointN << std::endl;
  std::cout << "DONE (takes " << float(clock() - begin_extractPoints) / CLOCKS_PER_SEC << "sec," << curpointN << " edge-points extracted). " << std::endl;
#endif
#ifdef DRAW
  drawingUtilities::drawPoints(curFrameBW, downsampledEdges);
  std::cout<<"NumPoints "<<downsampledEdges.size();
#endif

}

void EdgePointSpaceCarver::fromPtoF(cv::Mat P1, cv::Mat P2, cv::Mat &F) {
  cv::Mat X1 = P1(cv::Range(1, 3), cv::Range::all());
  cv::Mat X2 = cv::Mat(2, 4, X1.type());
  P1.row(2).copyTo(X2.row(0));
  P1.row(0).copyTo(X2.row(1));
  cv::Mat X3 = P1(cv::Range(0, 2), cv::Range::all());
  cv::Mat Y1 = P2(cv::Range(1, 3), cv::Range::all());
  cv::Mat Y2 = cv::Mat(2, 4, Y1.type());
  P2.row(2).copyTo(Y2.row(0));
  P2.row(0).copyTo(Y2.row(1));
  cv::Mat Y3 = P2(cv::Range(0, 2), cv::Range::all());

  cv::Mat F00_, F01_, F02_, F10_, F11_, F12_, F20_, F21_, F22_;

  cv::vconcat(X1, Y1, F00_);
  cv::vconcat(X2, Y1, F01_);
  cv::vconcat(X3, Y1, F02_);
  cv::vconcat(X1, Y2, F10_);
  cv::vconcat(X2, Y2, F11_);
  cv::vconcat(X3, Y2, F12_);
  cv::vconcat(X1, Y3, F20_);
  cv::vconcat(X2, Y3, F21_);
  cv::vconcat(X3, Y3, F22_);

  F =
      (cv::Mat_<float>(3, 3) << cv::determinant(F00_), cv::determinant(F01_), cv::determinant(F02_), cv::determinant(F10_), cv::determinant(F11_), cv::determinant(
          F12_), cv::determinant(F20_), cv::determinant(F21_), cv::determinant(F22_));

}

int EdgePointSpaceCarver::run() {

#ifdef LOGGING
  std::cout << "Start to carve!!!" << std::endl;
  struct timeval start, end;
  gettimeofday(&start, NULL);
#endif

  while (video_.getNextFrame(currentFrame_)) {

#ifdef LOGGING
    std::cout << "Frame num. " << currentFrameNumber_ << std::endl;
    fileLog << "Frame " << currentFrameNumber_ << std::endl;
    std::cout << "Add camera to space carver" << std::endl;
#endif
    manifRec_->addCameraCenter(cameras_[currentFrameNumber_].center.x(), cameras_[currentFrameNumber_].center.y(), cameras_[currentFrameNumber_].center.z());

    if (!previousFrame_.empty()) {  //**********ALL FRAMES AFTER THE FIRST ONE**********//
      trackPointsFrameByFrame();

      /*no more fram, end all the tracks*/
      if (currentFrameNumber_ >= video_.getIdxLastFrame() - 1) {
        for (std::vector<unsigned char>::iterator curActiveTrackId = activePointsIndices_.begin(); curActiveTrackId != activePointsIndices_.end();
            ++curActiveTrackId) {
          *curActiveTrackId = 0;
        }
        //minCam_ = video_.getIdxLastFrame();
      }

      //endedTracks_.clear();

#ifndef WITHOUT_MOVING_POINTS
      if (currentFrameNumber_ != video_.getIdxLastFrame()) {

#endif
      updateTracks();

      int curC = currentFrameNumber_;

      if (curC % keyFramePeriod_ == 0) {
#ifdef LOGPOINTSMOVE
        std::stringstream s;
        s << configuration_.outputSpaceCarving.pathLogPoints.c_str() << curC << ".txt";
        fileOut2.open(s.str().c_str());
#endif
        estimate3Dpositions();
#ifdef LOGPOINTSMOVE
        fileOut2.close();
#endif
      }

#ifdef WITHOUT_MOVING_POINTS
      if (minCam_ - 1 > lastCam_ && minCam_ != video_.getIdxLastFrame()) {
        if (lastCam_ >= 0)
          lastCam_ = lastCam_ - 1;
        for (int curC = lastCam_ + 1; curC < minCam_; ++curC) {
          if (curC % keyFramePeriod_ == 0) {
#endif

#ifdef LOGGING
            struct timeval startCarv, endCarv;
            gettimeofday(&startCarv, NULL);
            std::cout << "Let's carve!!! cam " << curC << " ";
            fileLog << "carve " << curC << " ";
#endif
            bool incrementalEnabled = curC > configuration_.spaceCarvingConfig.firstGrowingFrame
                && curC % configuration_.spaceCarvingConfig.manifoldPeriod == 0;

            manifRec_->insertNewPointsFromCam(curC, incrementalEnabled);

            manifRec_->rayTracingFromCam(curC);

#ifndef WITHOUT_MOVING_POINTS
            if (curC % keyFramePeriod_ == 0 ) {
              for (int cc = curC-keyFramePeriod_-10>=0?curC-keyFramePeriod_-10:0; cc < curC; ++cc) {
                //for (int cc = curC-keyFramePeriod_; cc < curC; ++cc) {

                manifRec_->insertNewPointsFromCam(curC, incrementalEnabled);
                manifRec_->rayTracingFromCam(curC);

              }
            }
#endif

            if (incrementalEnabled) {
              std::stringstream s;
              s << "ManifoldBefore" << curC << ".off";
              //manifRec_->saveManifold(s.str());
              manifRec_->growManifold();
              manifRec_->growManifoldSev();
              manifRec_->growManifold();
            }

            if (curC == configuration_.spaceCarvingConfig.firstGrowingFrame) {

              manifRec_->growManifold(curC);
              manifRec_->growManifoldSev();
              manifRec_->growManifold();

              if (configuration_.outputSpaceCarving.enableSaveReconstr) {
                std::stringstream s;
                s << "Manif.off";
                manifRec_->saveManifold(s.str());
                s << "Freesp.off";
                manifRec_->saveFreespace(s.str());
              }
            }

#ifdef LOGGING
            gettimeofday(&endCarv, NULL);
            float delta = ((endCarv.tv_sec - startCarv.tv_sec) * 1000000u + endCarv.tv_usec - startCarv.tv_usec) / 1.e6;
            std::cout << "DONE. (" << delta << " sec)" << std::endl;
            fileLog << delta << std::endl;
#endif
#ifdef WITHOUT_MOVING_POINTS
          }
#endif

#ifdef WITHOUT_MOVING_POINTS
        }
        lastCam_ = minCam_;
#endif
      } else if (currentFrameNumber_ == video_.getIdxLastFrame()) {
        std::stringstream s;
        s << "ManifInit.off";
        //manifRec_->saveManifold(s.str());
        s << "FreespInit.off";
        //manifRec_->saveFreespace(s.str());

        /*
         for (int curC = lastCam_ + 1; curC < video_.getIdxLastFrame(); ++curC) {
         if (curC % keyFramePeriod_ == 0) {

         #ifdef LOGGING
         struct timeval startCarv, endCarv;
         gettimeofday(&startCarv, NULL);
         std::cout << "Let's carve!!! cam " << curC << " ";
         fileLog << "carve " << curC << " ";
         #endif

         if (curC % configuration_.spaceCarvingConfig.manifoldPeriod == 0 && curC > configuration_.spaceCarvingConfig.firstGrowingFrame) {


         manifRec_->insertNewPointsFromCam(curC, incrementalEnabled);


         manifRec_->rayTracingFromCam(curC);
         manifRec_->growManifold();
         *manifRec_->growManifoldSev();
         manifRec_->growManifold();

         if(configuration_.outputSpaceCarving.enableSaveReconstr){
         std::stringstream s;
         s << "AfterRegrow_" << curC;
         saveCurrentManifold(s.str());
         }
         }

         #ifdef LOGGING
         gettimeofday(&endCarv, NULL);
         float delta = ((endCarv.tv_sec - startCarv.tv_sec) * 1000000u + endCarv.tv_usec - startCarv.tv_usec) / 1.e6;
         std::cout << "DONE. (" << delta << " sec)" << std::endl;
         fileLog << delta << std::endl;
         #endif
         }
         }*/
      }
    }

    if (currentFrameNumber_ % keyFramePeriodIteration_ == 0) {  //**********KEYFRAME************//
#ifdef LOGGING
      std::cout << "Keyframe Processing: " << std::endl;

#endif
      extractEdgePointsSimple();
    }

#ifdef LOGGING
    std::cout << "DONE. " << std::endl;
#endif
    currentFrame_.copyTo(previousFrame_);
    ++currentFrameNumber_;
  }

  std::cout << "DONE " << std::endl;

  gettimeofday(&end, NULL);

  float delta = ((end.tv_sec - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
  std::cout << "DONE. (" << delta << " sec)" << std::endl;
  fileLog << "TotTime " << delta << std::endl;

  manifRec_->saveManifold("Manif.off");
  manifRec_->saveFreespace("Freesp.off");
  //printGrid();

  return 1;
}

void EdgePointSpaceCarver::printGrid() {

  std::cout << "Grid:" << std::endl;
  for (auto itY : grid_) {
    for (auto itGint : itY) {
      std::cout << itGint << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

}

void EdgePointSpaceCarver::saveCurrentManifold(std::string suffix) {

  std::stringstream namefileFreespace, namefileBound, nameFIleManif;
  namefileBound << configuration_.outputSpaceCarving.pathToSave.c_str() << "Boundary" << suffix << ".obj";
  namefileFreespace << configuration_.outputSpaceCarving.pathToSave.c_str() << "FreespaceinverseConicEnabled_" << suffix << ".obj";
  nameFIleManif << configuration_.outputSpaceCarving.pathToSave.c_str() << "ManifinverseConicEnabled_" << suffix << ".obj";
}

int EdgePointSpaceCarver::point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian) {

  int numMeasures = cameras.size();
  cv::Mat cur3DPointHomog = cv::Mat(4, 1, CV_32F);
  ;
  cur3DPointHomog.at<float>(0, 0) = cur3Dpoint.at<float>(0, 0);
  cur3DPointHomog.at<float>(1, 0) = cur3Dpoint.at<float>(1, 0);
  cur3DPointHomog.at<float>(2, 0) = cur3Dpoint.at<float>(2, 0);
  cur3DPointHomog.at<float>(3, 0) = 1.0;

  J = cv::Mat(2 * numMeasures, 3, CV_32FC1);  //2 rows for each point: one for x, the other for y
  hessian = cv::Mat(3, 3, CV_32FC1);

  /*std::cout << "gdevre" <<std::endl;
   std::cout << cameras[0] <<std::endl;*/
  for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
    cv::Mat curReproj = cameras[curMeas] * cur3DPointHomog;
    float xH = curReproj.at<float>(0, 0);
    float yH = curReproj.at<float>(1, 0);
    float zH = curReproj.at<float>(2, 0);
    float p00 = cameras[curMeas].at<float>(0, 0);
    float p01 = cameras[curMeas].at<float>(0, 1);
    float p02 = cameras[curMeas].at<float>(0, 2);
    float p10 = cameras[curMeas].at<float>(1, 0);
    float p11 = cameras[curMeas].at<float>(1, 1);
    float p12 = cameras[curMeas].at<float>(1, 2);
    float p20 = cameras[curMeas].at<float>(2, 0);
    float p21 = cameras[curMeas].at<float>(2, 1);
    float p22 = cameras[curMeas].at<float>(2, 2);

    //d(P*X3D)/dX
    J.at<float>(2 * curMeas, 0) = (p00 * zH - p20 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 0) = (p10 * zH - p20 * yH) / (zH * zH);

    //d(P*X3D)/dY
    J.at<float>(2 * curMeas, 1) = (p01 * zH - p21 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 1) = (p11 * zH - p21 * yH) / (zH * zH);

    //d(P*X3D)/dZ
    J.at<float>(2 * curMeas, 2) = (p02 * zH - p22 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 2) = (p12 * zH - p22 * yH) / (zH * zH);
  }

  hessian = J.t() * J;
  float d;
  d = cv::determinant(hessian);
  if (d < 0.00001) {
    //printf("doh");
    return -1;
  } else {
    return 1;
  }
}

int EdgePointSpaceCarver::GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint,
    cv::Point3f &optimizedPoint) {
  int numMeasures = points.size();
  cv::Mat r = cv::Mat(numMeasures * 2, 1, CV_32F);

  cv::Mat curEstimate3DPoint = cv::Mat(3, 1, CV_32F);
  cv::Mat curEstimate3DPointH = cv::Mat(4, 1, CV_32F);
  curEstimate3DPoint.at<float>(0, 0) = init3Dpoint.x;
  curEstimate3DPoint.at<float>(1, 0) = init3Dpoint.y;
  curEstimate3DPoint.at<float>(2, 0) = init3Dpoint.z;

  cv::Mat J, H;
  float last_mse = 0;
  int i;
  for (i = 0; i < configuration_.spaceCarvingConfig.maxGaussNewtonIteration; i++) {

    float mse = 0;
    //compute residuals
    for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
      curEstimate3DPointH.at<float>(0, 0) = curEstimate3DPoint.at<float>(0, 0);
      curEstimate3DPointH.at<float>(1, 0) = curEstimate3DPoint.at<float>(1, 0);
      curEstimate3DPointH.at<float>(2, 0) = curEstimate3DPoint.at<float>(2, 0);
      curEstimate3DPointH.at<float>(3, 0) = 1.0;
      cv::Mat cur2DpositionH = cameras[curMeas] * curEstimate3DPointH;

      r.at<float>(2 * curMeas, 0) = ((points[curMeas].x - cur2DpositionH.at<float>(0, 0) / cur2DpositionH.at<float>(2, 0)));
      mse += r.at<float>(2 * curMeas, 0) * r.at<float>(2 * curMeas, 0);

      r.at<float>(2 * curMeas + 1, 0) = ((points[curMeas].y - cur2DpositionH.at<float>(1, 0) / cur2DpositionH.at<float>(2, 0)));
      mse += r.at<float>(2 * curMeas + 1, 0) * r.at<float>(2 * curMeas + 1, 0);
#ifdef DEBUG_OPTIMIZATION_VERBOSE
      std::cout<<"CurMeas: "<<curMeas<<std::endl<<"curEstimate3DPointH="<< curEstimate3DPointH.t()<<std::endl;
      std::cout<<"CurCam"<<cameras[curMeas]<<std::endl;
      std::cout<<"cur2DpositionH: "<<cur2DpositionH.at<float>(0, 0)/cur2DpositionH.at<float>(2, 0)<<", "<<cur2DpositionH.at<float>(1, 0)/cur2DpositionH.at<float>(2, 0)<<std::endl;
      std::cout<<"points[curMeas]: "<<points[curMeas]<<std::endl;
      std::cout<<"residual on x: "<<r.at<float>(2 * curMeas, 0)<<std::endl;
      std::cout<<"residual on y: "<<r.at<float>(2 * curMeas + 1 , 0)<<std::endl;
      std::cout<<std::endl;
#endif
    }
    //mse = sqrt(mse)/(numMeasures*2);

    if (abs(mse / (numMeasures * 2) - last_mse) < 0.0000005) {
      break;
    }
    last_mse = mse / (numMeasures * 2);

    if (point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1)
      return -1;
#ifdef DEBUG_OPTIMIZATION_VERBOSE
    std::cout<<"J: "<<J<<std::endl;
    std::cout<<"H: "<<H<<std::endl;
#endif

    curEstimate3DPoint += H.inv() * J.t() * r;

#ifdef DEBUG_OPTIMIZATION
    printf("%d %f\n", i, last_mse);
#endif
  }
  if (last_mse < 9/*3 pixels*/) {
    optimizedPoint.x = curEstimate3DPoint.at<float>(0, 0);
    optimizedPoint.y = curEstimate3DPoint.at<float>(1, 0);
    optimizedPoint.z = curEstimate3DPoint.at<float>(2, 0);
    return 1;
  } else {
    return -1;
  }
}

