#include <conversionUtilities.hpp>



Eigen::Matrix4f conversionUtilities::toEigenMatrix(glm::mat4 mat) {
  Eigen::Matrix4f matOut;

  for (int curR = 0; curR < 4; ++curR) {
    for (int curC = 0; curC < 4; ++curC) {
      matOut(curR, curC) = mat[curR][curC];
    }
  }

  return matOut;

}

cv::Mat conversionUtilities::toOpenCVMatrix(glm::mat4 mat, bool cameraMat4x4) {
  int maxR;
  if (cameraMat4x4)
    maxR = 4;
  else
    maxR = 3;

  cv::Mat matOut = cv::Mat(maxR, 4, CV_32F);

  for (int curR = 0; curR < maxR; ++curR) {
    for (int curC = 0; curC < 4; ++curC) {
      matOut.at<float>(curR, curC) = mat[curR][curC];
    }
  }

  return matOut;

}
