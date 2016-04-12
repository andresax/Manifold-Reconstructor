/*
 * Track.h
 *
 *  Created on: 14/gen/2015
 *      Author: andrea
 */

#ifndef TRACK_H_
#define TRACK_H_

#include <opencv2/core/core.hpp>
/*
 * It models one track of measures, its current 3D position */
class Track {
  public:
    Track();
    Track(cv::Point2f point, int camId, int idInTriangulation);
    Track(cv::Point2f point, int camId);
    virtual ~Track();

    int getIdInTriangulation() const {
      return idInTriangulation_;
    }

    void setIdInTriangulation(int idInTriangulation) {
      idInTriangulation_ = idInTriangulation;
    }

    std::vector<std::pair<cv::Point2f, int> > getTrack() const {
      return track_;
    }

    void setTrack(const std::vector<std::pair<cv::Point2f, int> >& track) {
      track_ = track;
    }

    const cv::Point3f& getWorldPosition() const {
      return worldPosition_;
    }

    void setWorldPosition(const cv::Point3f& worldPosition) {
      worldPosition_ = worldPosition;
      notYetEstimated = false;
    }

    size_t getTrackLength();
    void add2DMeasure(cv::Point2f point, int camId);
    std::pair<cv::Point2f, int> getLastFrameMeasure();
    std::pair<cv::Point2f, int> getMeasure(int i);
    std::pair<cv::Point2f, int> getMeasureCam(int i);

    bool isNotYetEstimated() const {
      return notYetEstimated;
    }

    std::string toString();

    int getEta() const {
      return eta_;
    }

    void setEta(int eta) {
      this->eta_ = eta;
    }
    void incrementEta() {
      eta_++;
    }
    bool isLastEstimateDone() const {
      return lastEstimateDone_;
    }

    void setLastEstimateDone(bool lastEstimateDone) {
      lastEstimateDone_ = lastEstimateDone;
    }

    bool isTrackEnded() const {
      return trackEnded_;
    }

    void setTrackEnded(bool trackEnded) {
      trackEnded_ = trackEnded;
    }

    bool isInserted() const {
      return inserted_;
    }

    void setInserted(bool inserted) {
      inserted_ = inserted;
    }

    void setNotYetEstimated(bool notYetEstimated) {
      this->notYetEstimated = notYetEstimated;
    }

    std::vector<std::pair<cv::Point2f, int> > track_;

  private:
    int idInTriangulation_;
    cv::Point3f worldPosition_;
    bool notYetEstimated;
    int eta_;//how many times the point has been moved
    bool trackEnded_;
    bool lastEstimateDone_;
    bool inserted_;
};

#endif /* TRACK_H_ */
