/*
 * EdgePointConfigurator.cpp
 *
 *  Created on: 02/lug/2015
 *      Author: andrea
 */

#include <EdgePointConfigurator.h>

EdgePointConfigurator::EdgePointConfigurator(const std::string &path) {
  file_.open(path.c_str());

}

EdgePointConfigurator::~EdgePointConfigurator() {
  // TODO Auto-generated destructor stub
}

Configuration EdgePointConfigurator::parseConfigFile() {

  Configuration myConf;
  std::string line;


  std::getline(file_, line);
  std::getline(file_, line);

  utilities::readLineAndStore(file_, myConf.videoConfig.baseNameImage);
  utilities::readLineAndStore(file_, myConf.videoConfig.imageExtension);
  utilities::readLineAndStore(file_, myConf.videoConfig.idxFirstFrame);
  utilities::readLineAndStore(file_, myConf.videoConfig.digitIdxLength);
  utilities::readLineAndStore(file_, myConf.videoConfig.idxLastFrame);

  utilities::readLineAndStore(file_, myConf.videoConfig.imageH);
  utilities::readLineAndStore(file_, myConf.videoConfig.imageW);

  std::getline(file_, line);

  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.keyFramePeriod);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.keyFramePeriodIteration);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.cannyHighThreshold);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.downsamplePeriod);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.maxGaussNewtonIteration);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.minDistBetweenTrackedPoints);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.maxEpipolarDist);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.minTrackLength);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.manifoldPeriod);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.inverseConicEnabled);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.probOrVoteThreshold);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.edgePointEnabled);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.firstGrowingFrame);
  utilities::readLineAndStore(file_, myConf.outputSpaceCarving.enableSaveReconstr);
  utilities::readLineAndStore(file_, myConf.outputSpaceCarving.enableSaveShrink);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.maxDistanceCamFeature);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.enableSuboptimalPolicy);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.suboptimalMethod);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.w_1);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.w_2);
  utilities::readLineAndStore(file_, myConf.spaceCarvingConfig.w_3);

  config_ = myConf;
  return myConf;
}
