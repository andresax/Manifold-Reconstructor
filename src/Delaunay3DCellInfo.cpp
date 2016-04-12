/*
 * Delaunay3DCellInfo.cpp
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#include <Delaunay3DCellInfo.h>

Delaunay3DCellInfo::Delaunay3DCellInfo() {
  m_voteCount = 0;
      m_voteCountProb = 0;
      m_bNew = true;
      boundary = false;
      keepManifold = false;
      toBeTested_ = false;
      shrinked_ = true;
      Galpha_ = false;
      temporary_Inside_ = false;

}
Delaunay3DCellInfo::Delaunay3DCellInfo(const Delaunay3DCellInfo & ref) {
    setVoteCount(ref.getVoteCount());
    setBoundary(ref.isBoundary());
    setIntersections(ref.getIntersections());
    if (!ref.isNew())
    markOld();
  }


Delaunay3DCellInfo::~Delaunay3DCellInfo() {
}
