/*
 * Delaunay3DVertexInfo.cpp
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#include <Delaunay3DVertexInfo.h>

Delaunay3DVertexInfo::Delaunay3DVertexInfo() {
  used_ = 0;
  notUsed_ = true;
  lastCam_ = -1;

}

Delaunay3DVertexInfo::~Delaunay3DVertexInfo() {
}

