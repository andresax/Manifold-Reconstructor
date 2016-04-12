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

#ifndef DRAWINGUTILITIES_HPP_
#define DRAWINGUTILITIES_HPP_

// This headers provides some useful functions that draw the extracted Edge-Points
// and their tracks

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>       /* fabs */
#include <iostream>

#define STORE_IMG

namespace drawingUtilities {
int drawPoints(cv::Mat &image, std::vector<cv::Point2f> pointList, cv::Mat &outImage, std::string pathImage) {

  image.copyTo(outImage);
  outImage = outImage * 2;
  cv::cvtColor(outImage, outImage, CV_GRAY2RGB);
  //draws points
  for (std::vector<cv::Point2f>::iterator curPoint = pointList.begin(); curPoint != pointList.end(); ++curPoint) {
    cv::circle(outImage, *curPoint, 1, cv::Scalar(255, 0, 0), 2);
  }
  cv::imshow(pathImage.c_str(), outImage);
  cv::imwrite("imgPoint.png", outImage);
  int key = cv::waitKey(0);

  if (key == 27) {
    return -1;
  } else {
    return 1;
  }
}
int drawPoints(cv::Mat &image, std::vector<cv::Point2f> pointList, cv::Mat &outImage) {
  return drawPoints(image, pointList, outImage, std::string("EdgeImageWithColors"));
}
int drawPoints(cv::Mat &image, std::vector<cv::Point2f> pointList) {
  cv::Mat outImage;
  return drawPoints(image, pointList, outImage, std::string("EdgeImageWithColors"));
}
int drawPoints(cv::Mat &image, std::vector<cv::Point2f> pointList, std::string pathImage) {
  cv::Mat outImage;
  return drawPoints(image, pointList, outImage, pathImage);
}

int drawTrackedPoints(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev, cv::Mat &imageCur, std::vector<cv::Point2f> pointListCur,
    cv::Mat &outImage, cv::vector<unsigned char> status, std::vector<float> err, std::string pathImage) {
  cv::Mat imagePrevRGB, imageCurRGB, outImageTMP;
  //outImage = imagePrev/2 + imageCur/2;
  cv::Mat outImaget, outImaget2;
  // cv::cvtColor(imagePrev, outImaget, CV_GRAY2RGB);
  outImaget = imagePrev * 2;
  for (size_t curLine = 0; curLine < pointListPrev.size(); ++curLine) {
    if (status[curLine] == 1 && err[curLine] < 10) {
      cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(255, 0, 0, 0), 2, 8, 0);
      //cv::circle(outImaget,pointListPrev[curLine],2,cv::Scalar(255,0,255,0),1,8,0);
      //cv::circle(outImaget,pointListCur[curLine],2,cv::Scalar(255,125,255,0),1,8,0);
    } else {
      //cv::line(outImaget,pointListPrev[curLine],pointListCur[curLine],cv::Scalar(0,0,255,0),1,8,0);
      //cv::circle(outImaget,pointListPrev[curLine],2,cv::Scalar(255,0,255,0),1,8,0);
      //cv::circle(outImaget,pointListCur[curLine],2,cv::Scalar(255,125,255,0),1,8,0);
    }

  }

  cv::imshow("res", outImaget);
  cv::imwrite(pathImage.c_str(), outImaget);
  int key = cv::waitKey(0);
  if (key == 27) {
    return -1;
  } else {
    return 1;
  }

  return 1;
}

int drawTrackedPoints(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev, cv::Mat &imageCur, std::vector<cv::Point2f> pointListCur,
    cv::Mat &outImage, cv::vector<unsigned char> status, std::vector<float> err) {
  return drawTrackedPoints(imagePrev, pointListPrev, imageCur, pointListCur, outImage, status, err, std::string("imgPointres.png"));
}

int drawFilteredPointWithEpipolar(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev, cv::Mat &imageCur,
    std::vector<cv::Point2f> pointListCur, cv::Mat &outImage, cv::vector<float> status, float scaleCoeff = 10.0) {
  cv::Mat imagePrevRGB, imageCurRGB, outImageTMP;
  //outImage = imagePrev/2 + imageCur/2;
  cv::Mat outImaget, outImaget2;
  // cv::cvtColor(imagePrev, outImaget, CV_GRAY2RGB);
  outImaget = imagePrev * 2;
  for (size_t curLine = 0; curLine < pointListPrev.size(); ++curLine) {
    float colorcoeff = status[curLine] / scaleCoeff;
    if (colorcoeff < 1) {
      cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(colorcoeff * 255, 0, 0, 0), 2, 8, 0);
    } else {
      cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(0, 0, 255, 0), 1, 8, 0);
    }
  }

  cv::imshow("EpipolarTest", outImaget);
  /* cv::imwrite("imgPointres.png", outImaget);*/
  int key = cv::waitKey(0);
  if (key == 27) {
    return -1;
  } else {
    return 1;
  }

  return 1;
}

int drawCorrespondences(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev, cv::Mat &imageCur, std::vector<cv::Point2f> pointListCur,
    cv::vector<unsigned char> status, std::string curPath) {
  cv::Mat imagePrevRGB, imageCurRGB, outImageTMP;
  //outImage = imagePrev/2 + imageCur/2;
  cv::Mat outImaget, outImaget2;
  // cv::cvtColor(imagePrev, outImaget, CV_GRAY2RGB);
  outImaget = imagePrev * 2;
  for (size_t curLine = 0; curLine < pointListPrev.size(); ++curLine) {
    if (status[curLine] == 1) {
      cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(255, 0, 0, 0), 2, 8, 0);
    } else {
      cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(0, 0, 255, 0), 1, 8, 0);
    }
  }

  cv::imshow("drawCorrespondences", outImaget);
  cv::imwrite(curPath.c_str(), outImaget);
  int key = cv::waitKey(12);
  if (key == 27) {
    return -1;
  } else {
    return 1;
  }

  return 1;
}


int drawCorrespondences(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev, cv::Mat &imageCur, std::vector<cv::Point2f> pointListCur,
    cv::vector<unsigned char> status) {
  cv::Mat imagePrevRGB, imageCurRGB, outImageTMP;
  //outImage = imagePrev/2 + imageCur/2;
  cv::Mat outImaget, outImaget2;
  // cv::cvtColor(imagePrev, outImaget, CV_GRAY2RGB);
  outImaget = imagePrev * 2;
  for (size_t curLine = 0; curLine < pointListPrev.size(); ++curLine) {
    if (status[curLine] == 1) {
      cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(255, 0, 0, 0), 2, 8, 0);
    } else {
      cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(0, 0, 255, 0), 1, 8, 0);
    }
  }

  cv::imshow("drawCorrespondences", outImaget);
  int key = cv::waitKey(0);
  if (key == 27) {
    return -1;
  } else {
    return 1;
  }

  return 1;
}

int drawTrackedPoints2(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev, cv::Mat &imageCur, std::vector<cv::Point2f> pointListCur,
    cv::Mat &outImage, cv::vector<unsigned char> status, std::vector<float> err, cv::vector<unsigned char> statusR) {
  cv::Mat imagePrevRGB, imageCurRGB, outImageTMP;
  //outImage = imagePrev/2 + imageCur/2;
  cv::Mat outImaget, outImaget2;
  // cv::cvtColor(imagePrev, outImaget, CV_GRAY2RGB);
  outImaget = imagePrev * 2;
  for (size_t curLine = 0; curLine < pointListPrev.size(); ++curLine) {
    if (statusR[curLine] == 1 && err[curLine] < 10) {
      if (status[curLine] == 1) {
        cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(255, 0, 0, 0), 2, 8, 0);
        //      cv::circle(outImaget,pointListPrev[curLine],2,cv::Scalar(255,0,255,0),3,8,0);
        //      cv::circle(outImaget,pointListCur[curLine],2,cv::Scalar(255,125,255,0),3,8,0);
      } else {
        cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(0, 0, 255, 0), 1, 8, 0);
        //            cv::circle(outImaget,pointListPrev[curLine],2,cv::Scalar(255,0,255,0),1,8,0);
        //            cv::circle(outImaget,pointListCur[curLine],2,cv::Scalar(255,125,255,0),1,8,0);
      }
    }
  }

  cv::imshow("res2", outImaget);
  cv::imwrite("imgPointres2.png", outImaget);
  int key = cv::waitKey(0);
  if (key == 27) {
    return -1;
  } else {
    return 1;
  }

  return 1;
}

int drawTrackedPoints3(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev, cv::Mat &imageCur, std::vector<cv::Point2f> pointListCur,
    cv::Mat &outImage, cv::vector<unsigned char> status, std::vector<float> err, cv::vector<unsigned char> statusR) {
  cv::Mat imagePrevRGB, imageCurRGB, outImageTMP;
  //outImage = imagePrev/2 + imageCur/2;
  cv::Mat outImaget, outImaget2;
  // cv::cvtColor(imagePrev, outImaget, CV_GRAY2RGB);
  outImaget = imagePrev * 2;
  for (size_t curLine = 0; curLine < pointListPrev.size(); ++curLine) {
    if (statusR[curLine] == 1 && err[curLine] < 10) {
      if (status[curLine] == 1) {
        cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(255, 0, 0, 0), 2, 8, 0);
        //      cv::circle(outImaget,pointListPrev[curLine],2,cv::Scalar(255,0,255,0),3,8,0);
        //      cv::circle(outImaget,pointListCur[curLine],2,cv::Scalar(255,125,255,0),3,8,0);
      } else {
        cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(0, 0, 255, 0), 1, 8, 0);
        //            cv::circle(outImaget,pointListPrev[curLine],2,cv::Scalar(255,0,255,0),1,8,0);
        //            cv::circle(outImaget,pointListCur[curLine],2,cv::Scalar(255,125,255,0),1,8,0);
      }
    }
  }

  cv::imshow("res3", outImaget);
  cv::imwrite("imgPointres3.png", outImaget);
  int key = cv::waitKey(0);
  if (key == 27) {
    return -1;
  } else {
    return 1;
  }

  return 1;
}
//int drawTrackedPoints(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev,
//    cv::Mat &imageCur, std::vector<cv::Point2f> pointListCur ,cv::vector<unsigned char> status,std::vector<float> err){
//  cv::Mat outImage;
//  cv::Mat imagePrevRGB,imageCurRGB, outImageTMP;
//  return drawTrackedPoints(imagePrev, pointListPrev, imageCur, pointListCur, outImage,status, err);
//}

int drawTrackedPoints(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev, cv::Mat &imageCur, std::vector<cv::Point2f> pointListCur,
    cv::vector<unsigned char> status, std::vector<float> err) {
  cv::Mat outImage;
  cv::Mat imagePrevRGB, imageCurRGB, outImageTMP;
  return drawTrackedPoints(imagePrev, pointListPrev, imageCur, pointListCur, outImage, status, err);
}

int drawTrackedPoint(cv::Mat &imagePrev, std::vector<cv::Point2f> pointListPrev, cv::Mat &imageCur, std::vector<cv::Point2f> pointListCur,
    cv::vector<unsigned char> status) {

  cv::Mat outImaget = imagePrev / 2 + imageCur / 2;
  for (size_t curLine = 0; curLine < pointListPrev.size(); curLine = curLine + 1) {
    if (status[curLine] == 1) {
      cv::line(outImaget, pointListPrev[curLine], pointListCur[curLine], cv::Scalar(255, 0, 0, 0), 1, 8, 0);
      cv::circle(outImaget, pointListCur[curLine], 2, cv::Scalar(255, 0, 255, 0), 1, 8, 0);
    }
  }

  cv::imshow("drawTrackedPoint", outImaget);
  int key = cv::waitKey(7);
  if (key == 27) {
    return -1;
  } else {
    return 1;
  }

  return 1;
}

template<typename T>
static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T, 3>& line) {
  //Line is given as a*x + b*y + c = 0
  return std::abs(line(0) * point.x + line(1) * point.y + line(2)) / std::sqrt(line(0) * line(0) + line(1) * line(1));
}

/**
 * \brief Compute and draw the epipolar lines in two images
 *      associated to each other by a fundamental matrix
 *
 * \param title     Title of the window to display
 * \param F         Fundamental matrix
 * \param img1      First image
 * \param img2      Second image
 * \param points1   Set of points in the first image
 * \param points2   Set of points in the second image matching to the first set
 * \param inlierDistance      Points with a high distance to the epipolar lines are
 *                not displayed. If it is negative, all points are displayed
 **/
template<typename T1, typename T2>
static void drawEpipolarLines(const std::string& title, const cv::Matx<T1, 3, 3> F, const cv::Mat& img1, const cv::Mat& img2,
    const std::vector<cv::Point_<T2> > points1, const std::vector<cv::Point_<T2> > points2, const float inlierDistance = -1) {

  CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
  cv::Mat outImg(img1.rows, img1.cols * 2, CV_8UC3);
  cv::Rect rect1(0, 0, img1.cols, img1.rows);
  cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
  /*
   * Allow color drawing
   */
  if (img1.type() == CV_8U) {
    cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
    cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
  } else {
    img1.copyTo(outImg(rect1));
    img2.copyTo(outImg(rect2));
  }
  std::vector<cv::Vec<T2, 3> > epilines1, epilines2;
  cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
  cv::computeCorrespondEpilines(points2, 2, F, epilines2);

  CV_Assert(points1.size() == points2.size() && points2.size() == epilines1.size() && epilines1.size() == epilines2.size());

  cv::RNG rng(0);
  for (size_t i = 0; i < points1.size(); i++) {
    if (inlierDistance > 0) {
      if (distancePointLine(points1[i], epilines2[i]) > inlierDistance || distancePointLine(points2[i], epilines1[i]) > inlierDistance) {
        //The point match is no inlier
        continue;
      }
    }
    /*
     * Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
     */
    cv::Scalar color(rng(256), rng(256), rng(256));
    cv::Mat temp = outImg.clone();
    cv::Mat tmp2 = outImg(rect2);
    cv::Mat tmp1 = outImg(rect1);
    //outImg.copyTo(temp);

    cv::line(tmp2, cv::Point(0, -epilines1[i][2] / epilines1[i][1]),
        cv::Point(img1.cols, -(epilines1[i][2] + epilines1[i][0] * img1.cols) / epilines1[i][1]), color);
    cv::circle(tmp1, points1[i], 3, color, -1, CV_AA);

    cv::line(tmp1, cv::Point(0, -epilines2[i][2] / epilines2[i][1]),
        cv::Point(img2.cols, -(epilines2[i][2] + epilines2[i][0] * img2.cols) / epilines2[i][1]), color);
    cv::circle(tmp2, points2[i], 3, color, -1, CV_AA);
  }
  cv::imshow(title, outImg);
  cv::waitKey(1);
}

template<typename T1, typename T2>
static void myDrawEpipolarLines(const std::string& title, const cv::Matx<T1, 3, 3> F, const cv::Mat& img1, const cv::Mat& img2,
    const std::vector<cv::Point_<T2> > points1, const std::vector<cv::Point_<T2> > points2) {

  cv::Mat outImg(img1.rows, img1.cols, CV_8UC3);

  cv::Mat outImgInit(img1.rows, img1.cols, CV_8UC3);

  if (img1.type() == CV_8U) {
    cv::cvtColor(img1, outImg, CV_GRAY2BGR);
  } else {
    img1.copyTo(outImg);
  }

  outImg.copyTo(outImgInit);
  std::vector<cv::Vec<T2, 3> > epilines;
  cv::computeCorrespondEpilines(points1, 1, F, epilines);

  for (size_t i = 0; i < points1.size(); i++) {

    cv::Mat outImg2(img1.rows, img1.cols, CV_8UC3);
    outImgInit.copyTo(outImg2);

    cv::line(outImg2, cv::Point(0, -epilines[i][2] / epilines[i][1]),
        cv::Point(img1.cols, -(epilines[i][2] + epilines[i][0] * img1.cols) / epilines[i][1]), cv::Scalar(255, 0, 0));
    cv::circle(outImg2, points2[i], 3, cv::Scalar(0, 255, 255), 2);
    cv::line(outImg, cv::Point(0, -epilines[i][2] / epilines[i][1]),
        cv::Point(img1.cols, -(epilines[i][2] + epilines[i][0] * img1.cols) / epilines[i][1]), cv::Scalar(255, 0, 0));
    cv::circle(outImg, points2[i], 3, cv::Scalar(0, 255, 255), 2);

    std::cout << "Current Distance to point" << points2[i] << ": " << distancePointLine(points2[i], epilines[i]) << std::endl;
    cv::imshow(title, outImg2);
    cv::waitKey(0);
  }
  cv::imshow(title, outImg);
  cv::waitKey(0);
}

template<typename T1, typename T2>
static void myDrawEpipolarLinesDouble(const std::string& title, const cv::Matx<T1, 3, 3> F, const cv::Mat& img1, const cv::Mat& img2,
    const std::vector<cv::Point_<T2> > points1, const std::vector<cv::Point_<T2> > points2) {
  cv::Mat outImg(img1.rows, img1.cols * 2, CV_8UC3);
  cv::Mat outImgInit(img1.rows, img1.cols * 2, CV_8UC3);
  cv::Rect rect1(0, 0, img1.cols, img1.rows);
  cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
  /*
   * Allow color drawing
   */
  if (img1.type() == CV_8U) {
    cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
    cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
  } else {
    img1.copyTo(outImg(rect1));
    img2.copyTo(outImg(rect2));
  }
  outImg.copyTo(outImgInit);
  std::vector<cv::Vec<T2, 3> > epilines1, epilines2;
  cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
  cv::computeCorrespondEpilines(points2, 2, F, epilines2);

  CV_Assert(points1.size() == points2.size() && points2.size() == epilines1.size() && epilines1.size() == epilines2.size());

  cv::RNG rng(0);
  for (size_t i = 0; i < points1.size(); i++) {

    outImg.copyTo(outImgInit);
    cv::Scalar color(rng(256), rng(256), rng(256));
    cv::Mat temp = outImgInit.clone();
    cv::Mat tmp2 = outImgInit(rect2);
    cv::Mat tmp1 = outImgInit(rect1);
    //outImg.copyTo(temp);

    cv::line(tmp2, cv::Point(0, -epilines1[i][2] / epilines1[i][1]),
        cv::Point(img1.cols, -(epilines1[i][2] + epilines1[i][0] * img1.cols) / epilines1[i][1]), cv::Scalar(0, 255, 0));
    cv::circle(tmp2, points1[i], 3, cv::Scalar(255, 0, 0), -1, CV_AA);

    cv::line(tmp1, cv::Point(0, -epilines2[i][2] / epilines2[i][1]),
        cv::Point(img2.cols, -(epilines2[i][2] + epilines2[i][0] * img2.cols) / epilines2[i][1]), cv::Scalar(0, 255, 0));
    cv::circle(tmp1, points2[i], 3, cv::Scalar(255, 0, 0), -1, CV_AA);

    cv::imshow(title, outImgInit);
    cv::waitKey(0);
  }
}

}  // namespace drawingUtilities

#endif /* DRAWINGUTILITIES_HPP_ */
