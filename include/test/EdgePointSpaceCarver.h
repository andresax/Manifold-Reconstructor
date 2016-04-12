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
//
//  Implementation described in the paper:
//
//  Romanoni, A., Matteucci, M.. "Incremental Reconstruction of Urban Environments by Edge-Points Delaunay Triangulation."
//  Intelligent Robots and Systems (IROS 2015), 2015 IEEE/RSJ International Conference on. IEEE, 2015.


#ifndef EDGEPOINTSPACECARVER_H_
#define EDGEPOINTSPACECARVER_H_


// This class implements the Edge-Point approach to space carving. Each function
// implementd a different stage: edge-point extraction, edge-point tracking,
// 3D point positions estimation, optimization algorithm (via Gauss-Newton)
// and the run function calls the space carver (FreespaceDelaunayAlgorithm library)

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_3.h>

#include <ManifoldMeshReconstructor.h>

#include <KittiCamParser.h>

#include <types_reconstructor.hpp>
#include <types_config.hpp>
#include <VideoSimulator.h>
#include <Track.h>

//#define DEBUG_OPTIMIZATION_VERBOSE
//#define DEBUG_OPTIMIZATION

//#define DRAW
//#define DRAW_TRACKING

#define LOGGING
#define LOGGING_DISTRIBUTION
//#define LOGPOINTS
//#define LOGPOINTSMOVE
//#define LOGGING_VERBOSE
#define OPEN_MP_ENABLED
#define WITHOUT_MOVING_POINTS


class EdgePointSpaceCarver {
  public:
    EdgePointSpaceCarver(Configuration configuration, std::vector<CameraRect> cameras);
    virtual ~EdgePointSpaceCarver();

    int run();
  private:
    Configuration configuration_;
    cv::Mat currentFrame_;
    cv::Mat previousFrame_;
    cv::Mat lastKeyFrame_;
    VideoSimulator video_;
    int keyFramePeriod_;
    int keyFramePeriodIteration_;
    float cannyHighThreshold_;
    int downsamplePeriod_;
    std::vector<CameraRect> cameras_;
    std::vector<cv::Mat> camerasP_;
    std::vector<cv::Mat> camerasK_;
    std::vector<cv::Mat> camerasE_;
    int imageW_, imageH_;
    int currentFrameNumber_;
    int idPoint_;
    std::vector<cv::Point2f> prevTrackedPoints_, curTrackedPoints_;
    std::vector<unsigned char> activePointsIndices_;
    std::vector<Track> activeTracks_;
    std::vector<Track> endedTracks_;


    ManifoldMeshReconstructor *manifRec_;

    int minCam_, lastCam_;
    int newMinCam_;
    std::ofstream fileOut;
    std::ofstream fileLog;
    std::ofstream fileStats_;
    std::ofstream fileOutVerbose;
    int minDistBetweenTrackedPoints_;
    int minTrackLength_;
    std::ofstream fileOut2;


    std::vector<std::vector<int> > grid_;

    int totLen_, numLen_, meanTrackLen_;//to be removed after the test is gone

    void bundlerParamToCalibMatrix(SensorParser camBundler, cv::Mat &K, cv::Mat &E);
    void kittiParamToCalibMatrix(Camera camBundler, cv::Mat &K, cv::Mat &E);
    void kittiRectParamToCalibMatrix(CameraRect camBundler, cv::Mat &K, cv::Mat &E);

    void trackPointsFrameByFrame();
    void trackRefinementWithEpipolar(std::vector<unsigned char> status);
    void trackRefinementWithBacktrackingAndFiltering(std::vector<unsigned char> statusReal);

    void updateTracks();
    void estimate3Dpositions();
    void extractEdgePoints();
    void extractEdgePointsSimple();

    void fromPtoF(cv::Mat P1, cv::Mat P2,  cv::Mat &F);


    void printTracksStuff();
    int GaussNewton(const std::vector<cv::Mat> &cameras,
        const std::vector<cv::Point2f> &points,cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint);

    int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint,
        cv::Mat &jacobian, cv::Mat &hessian);

    void printGrid();


    void saveCurrentManifold(std::string suffix);




    //    float backprojectionError(const cv::Point3f &input, const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points);
};

#endif /* EDGEPOINTSPACECARVER_H_ */
