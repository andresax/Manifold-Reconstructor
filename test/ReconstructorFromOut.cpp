/*
 * ReconstructorFromOut.cpp
 *
 *  Created on: 13/lug/2015
 *      Author: andrea
 */

#include <ReconstructorFromOut.h>
#include <conversionUtilities.hpp>

#include <utilities.hpp>
#include <types_reconstructor.hpp>

ReconstructorFromOut::ReconstructorFromOut() {

//  pointP = new PointsParserFromOut("/home/andrea/Documenti/PhD/parserbundler/herzu_15_04_07_points.out");
////  fileCams_.open("/home/andrea/Scrivania/Datasets/EpflDataset/herzjesu_dense/centers.txt");
  pointP = new PointsParserFromOut("/home/andrea/workspaceC/ggpostica-urban-reconstruction-9f96d7f92733/output/bundler/points_200K.out");
  fileCams_.open("/home/andrea/workspaceC/ggpostica-urban-reconstruction-9f96d7f92733/output/bundler/cams_out_0095.txt");
  /*pointP = new PointsParserFromOut("/home/andrea/workspaceC/manifoldReconstructor/examples/temple/templeRing_points.out");
  fileCams_.open("/home/andrea/workspaceC/manifoldReconstructor/examples/temple/templeRingCenters.txt");*/
  parseCenters();
  pointP->parse();

  ManifoldReconstructionConfig confManif;
  confManif.inverseConicEnabled = true;
  confManif.maxDistanceCamFeature = 100.0;
  confManif.probOrVoteThreshold = 1.0;
  confManif.enableSuboptimalPolicy = false;
  confManif.suboptimalMethod = 0;
  confManif.w_1 = 1.0;
  confManif.w_2 = 0.10;
  confManif.w_3 = 0.00;

  manifRec_ = new ManifoldMeshReconstructor(confManif);
  utilities::saveVisibilityPly(camCenters_, pointP->getPoints(), pointP->getPointsVisibleFromCamN());
}

ReconstructorFromOut::~ReconstructorFromOut() {
}

void ReconstructorFromOut::run() {

  for (auto camC : camCenters_) {

    manifRec_->addCameraCenter(camC.x, camC.y, camC.z);
  }
  for (auto curPt : pointP->getPoints()) {

    manifRec_->addPoint(curPt.x, curPt.y, curPt.z);
  }

  for (int curCamIdx = 0; curCamIdx < camCenters_.size()-200; curCamIdx += 5) {

    //std::cout << "spaceCarver_->addVisibilityPair cam:" << curCamIdx << " ";
    for (auto curPtIdx : pointP->getPointsVisibleFromCamN(curCamIdx)) {

      if (pointP->getCamViewingPointN(curPtIdx).size() >= 1)
        manifRec_->addVisibilityPair(curCamIdx, curPtIdx);
    }
  }

  for (int curCamIdx = 0; curCamIdx < camCenters_.size()-200; curCamIdx += 5) {

    manifRec_->insertNewPointsFromCam(curCamIdx, false);
    //manifRec_->saveManifold("prova.off");
    manifRec_->rayTracingFromCam(curCamIdx);
    std::cout << "Cam " << curCamIdx << " done" << std::endl;
  }

  //spaceCarver_->BatchTetrahedronMethod(0);
  utilities::Logger log;
  log.startEvent();
  int curIt = 0;
  std::stringstream s, s2, s3,s4,s5,s6,s7,s8;
  s << "ManifoldBefore" << curIt << ".off";
  s2 << "ManifoldAfter" << curIt << ".off";
  s4 << "ManifoldAfter1" << curIt << ".off";
  s5 << "ManifoldAfter2" << curIt << ".off";
  s6 << "ManifoldAfter3" << curIt << ".off";
  s7 << "ManifoldAfter4" << curIt << ".off";
  s8 << "ManifoldAfter5" << curIt << ".off";
  s3 << "Free" << curIt << ".off";
  manifRec_->growManifold();
  manifRec_->saveManifold(s.str());
  manifRec_->growManifoldSev();
  manifRec_->saveManifold(s2.str());
  manifRec_->growManifold();
  manifRec_->saveManifold(s4.str());




  //manifRec_->saveFreespace(s3.str());
  log.endEventAndPrint("Grow ", true);
  std::cout << "DONE" << std::endl;
}

void ReconstructorFromOut::parseCenters() {
  int numCams;
  fileCams_ >> numCams;

  for (int curCam = 0; curCam < numCams; ++curCam) {
    float curX, curY, curZ;
    fileCams_ >> curX >> curY >> curZ;

    camCenters_.push_back(glm::vec3(curX, curY, curZ));
    std::cout << "CENTERS " << curX << " " << curY << " " << curZ << std::endl;
  }
}

