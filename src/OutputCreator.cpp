/*
 * OutputCreator.cpp
 *
 *  Created on: 29/giu/2015
 *      Author: andrea
 */

#include <OutputCreator.h>
#include <ext/hash_map>
#include <fstream>
#include <iostream>

OutputCreator::OutputCreator(Delaunay3 &dt) :
    dt_(dt) {
  th_ = 1.0;
}

OutputCreator::~OutputCreator() {
}

void OutputCreator::tetrahedraToTriangles(std::vector<PointD3>& points, std::vector<PointD3>& tris, int lastCam) {
  std::vector<Delaunay3::Vertex_handle> vecBoundsHandles;
  std::vector<Delaunay3::Vertex_handle> vecVertexHandles;
  __gnu_cxx ::hash_map<Delaunay3::Vertex_handle, int, HashVertHandle, EqVertHandle> hmapVertexHandleToIndex;
  int numKept = 0, numNotKept = 0;
  int numManifold = 0, numNotManifold = 0;

  for (Delaunay3::Cell_iterator itCell = dt_.cells_begin(); itCell != dt_.cells_end(); itCell++) {
    if ((*itCell).info().iskeptManifold()) {
      ++numManifold;
    } else {
      ++numNotManifold;
    }
  }
  std::cout << "numManifold: " << numManifold << ", tets" << std::endl;
  std::cout << "numNotManifold: " << numNotManifold << ", tets" << std::endl;

  // Initialize points and tris as empty:
  if (!points.empty())
    points.clear();
  if (!tris.empty())
    tris.clear();

  // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
  dt_.incident_vertices(dt_.infinite_vertex(), std::back_inserter(vecBoundsHandles));

  // Populate the model's point list, create a list of finite non-bounding vertex handles, and
  // create a useful associative maps (handle->point list index).
  for (Delaunay3::Finite_vertices_iterator itVert = dt_.finite_vertices_begin(); itVert != dt_.finite_vertices_end(); itVert++) {
    std::vector<Delaunay3::Vertex_handle>::iterator itBounds;
    for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
      if ((*itBounds) == ((Delaunay3::Vertex_handle) itVert))
        break;
    }
    if (itBounds == vecBoundsHandles.end() && (lastCam < 0 || itVert->info().getLastCam() < lastCam)) { // the vertex is not a bounding vertex, so add it
      PointD3 tmp = itVert->point();
      points.push_back(tmp);
      vecVertexHandles.push_back(itVert);
      hmapVertexHandleToIndex[itVert] = points.size() - 1;

    }
  }

  // Iterate over finite facets
  for (Delaunay3::Finite_facets_iterator itFacet = dt_.finite_facets_begin(); itFacet != dt_.finite_facets_end(); itFacet++) {
    // If one adjacent cell is empty, and the other is not, and if the facet contains no vertex from the bounding vertices, then add a triangle to the mesh (w/ correct orientation).
    bool bFacetCellKept = itFacet->first->info().iskeptManifold();

    bool bMirrorCellKept = itFacet->first->neighbor(itFacet->second)->info().iskeptManifold();
    if ((bFacetCellKept != bMirrorCellKept)) {
      numKept++;
    } else {
      numNotKept++;
    }
    if (!bFacetCellKept && bMirrorCellKept) {
      bool bContainsBoundsVert = false;
      for (int i = 0; i < 4; i++) {
        if (i == itFacet->second)
          continue;
        if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
          bContainsBoundsVert = true;
          break;

        }
      }
      if (!bContainsBoundsVert) {
        Delaunay3::Facet fTmp = dt_.mirror_facet(*itFacet); // The normal points inward so mirror the facet
        PointD3 tmpTri;
        std::vector<Delaunay3::Vertex_handle> vecTri;

        facetToTriangle(fTmp, vecTri);
        tmpTri = PointD3(hmapVertexHandleToIndex[vecTri[0]], hmapVertexHandleToIndex[vecTri[1]], hmapVertexHandleToIndex[vecTri[2]]);

        tris.push_back(tmpTri);
      }
    } else if (!bMirrorCellKept && bFacetCellKept) {
      bool bContainsBoundsVert = false;
      for (int i = 0; i < 4; i++) {
        if (i == itFacet->second)
          continue;
        if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
          bContainsBoundsVert = true;
          break;
        }
      }
      if (!bContainsBoundsVert) {
        Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
        PointD3 tmpTri;
        std::vector<Delaunay3::Vertex_handle> vecTri;

        facetToTriangle(fTmp, vecTri);
        tmpTri = PointD3(hmapVertexHandleToIndex[vecTri[0]], hmapVertexHandleToIndex[vecTri[1]], hmapVertexHandleToIndex[vecTri[2]]);
        tris.push_back(tmpTri);
      }
    }
  }
  std::cout << "num facets kept: " << numKept << ", num facets not kept: " << numNotKept << std::endl;
  std::cout << "tris created: " << tris.size() << ", points : " << points.size() << std::endl;
}

/*


void OutputCreator::tetrahedraBoundaryToTriangles(std::vector<PointD3> & points, std::vector<PointD3> & tris){
  std::vector<Delaunay3::Vertex_handle> vecBoundsHandles;
    std::vector<Delaunay3::Vertex_handle> vecVertexHandles;
    __gnu_cxx ::hash_map<Delaunay3::Vertex_handle, int, HashVertHandle, EqVertHandle> hmapVertexHandleToIndex;
    int numKept = 0, numNotKept = 0;
    int numManifold = 0, numNotManifold = 0;

    for (Delaunay3::Cell_iterator itCell = dt_.cells_begin(); itCell != dt_.cells_end(); itCell++) {
      if ((*itCell).info().iskeptManifold()) {
        ++numManifold;
      } else {
        ++numNotManifold;
      }
    }
    std::cout << "numManifold: " << numManifold << ", tets" << std::endl;
    std::cout << "numNotManifold: " << numNotManifold << ", tets" << std::endl;

    // Initialize points and tris as empty:
    if (!points.empty())
      points.clear();
    if (!tris.empty())
      tris.clear();

    // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
    dt_.incident_vertices(dt_.infinite_vertex(), std::back_inserter(vecBoundsHandles));

    // Populate the model's point list, create a list of finite non-bounding vertex handles, and
    // create a useful associative maps (handle->point list index).
    for (Delaunay3::Finite_vertices_iterator itVert = dt_.finite_vertices_begin(); itVert != dt_.finite_vertices_end(); itVert++) {
      std::vector<Delaunay3::Vertex_handle>::iterator itBounds;
      for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
        if ((*itBounds) == ((Delaunay3::Vertex_handle) itVert))
          break;
      }
      if (itBounds == vecBoundsHandles.end() && (lastCam < 0 || itVert->info().getLastCam() < lastCam)) { // the vertex is not a bounding vertex, so add it
        PointD3 tmp = itVert->point();
        points.push_back(tmp);
        vecVertexHandles.push_back(itVert);
        hmapVertexHandleToIndex[itVert] = points.size() - 1;

      }
    }

    // Iterate over finite facets
    for (Delaunay3::Finite_facets_iterator itFacet = dt_.finite_facets_begin(); itFacet != dt_.finite_facets_end(); itFacet++) {
      // If one adjacent cell is empty, and the other is not, and if the facet contains no vertex from the bounding vertices, then add a triangle to the mesh (w/ correct orientation).
      bool bFacetCellKept = itFacet->first->info().iskeptManifold();

      bool bMirrorCellKept = itFacet->first->neighbor(itFacet->second)->info().iskeptManifold();
      if ((bFacetCellKept != bMirrorCellKept)) {
        numKept++;
      } else {
        numNotKept++;
      }
      if (!bFacetCellKept && bMirrorCellKept) {
        bool bContainsBoundsVert = false;
        for (int i = 0; i < 4; i++) {
          if (i == itFacet->second)
            continue;
          if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
            bContainsBoundsVert = true;
            break;

          }
        }
        if (!bContainsBoundsVert) {
          Delaunay3::Facet fTmp = dt_.mirror_facet(*itFacet); // The normal points inward so mirror the facet
          PointD3 tmpTri;
          std::vector<Delaunay3::Vertex_handle> vecTri;

          facetToTriangle(fTmp, vecTri);
          tmpTri = PointD3(hmapVertexHandleToIndex[vecTri[0]], hmapVertexHandleToIndex[vecTri[1]], hmapVertexHandleToIndex[vecTri[2]]);

          tris.push_back(tmpTri);
        }
      } else if (!bMirrorCellKept && bFacetCellKept) {
        bool bContainsBoundsVert = false;
        for (int i = 0; i < 4; i++) {
          if (i == itFacet->second)
            continue;
          if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
            bContainsBoundsVert = true;
            break;
          }
        }
        if (!bContainsBoundsVert) {
          Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
          PointD3 tmpTri;
          std::vector<Delaunay3::Vertex_handle> vecTri;

          facetToTriangle(fTmp, vecTri);
          tmpTri = PointD3(hmapVertexHandleToIndex[vecTri[0]], hmapVertexHandleToIndex[vecTri[1]], hmapVertexHandleToIndex[vecTri[2]]);
          tris.push_back(tmpTri);
        }
      }
    }
    std::cout << "num facets kept: " << numKept << ", num facets not kept: " << numNotKept << std::endl;
    std::cout << "tris created: " << tris.size() << ", points : " << points.size() << std::endl;
}*/
void OutputCreator::tetrahedraToTriangles(std::vector<PointD3> & points, std::vector<PointD3> & tris, std::vector<int> cams) {
  std::vector<Delaunay3::Vertex_handle> vecBoundsHandles;
  std::vector<Delaunay3::Vertex_handle> vecVertexHandles;
  __gnu_cxx ::hash_map<Delaunay3::Vertex_handle, int, HashVertHandle, EqVertHandle> hmapVertexHandleToIndex;
  int numKept = 0, numNotKept = 0;
  int numManifold = 0, numNotManifold = 0;

  for (Delaunay3::Cell_iterator itCell = dt_.cells_begin(); itCell != dt_.cells_end(); itCell++) {
    if ((*itCell).info().iskeptManifold()) {
      ++numManifold;
    } else {
      ++numNotManifold;
    }
  }
  std::cout << "numManifold: " << numManifold << ", tets" << std::endl;
  std::cout << "numNotManifold: " << numNotManifold << ", tets" << std::endl;

  // Initialize points and tris as empty:
  if (!points.empty())
    points.clear();
  if (!tris.empty())
    tris.clear();

  //std::ofstream s("out2.txt");
  // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
  dt_.incident_vertices(dt_.infinite_vertex(), std::back_inserter(vecBoundsHandles));

  // Populate the model's point list, create a list of finite non-bounding vertex handles, and
  // create a useful associative maps (handle->point list index).
  for (Delaunay3::Finite_vertices_iterator itVert = dt_.finite_vertices_begin(); itVert != dt_.finite_vertices_end(); itVert++) {
    std::vector<Delaunay3::Vertex_handle>::iterator itBounds;
    for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
      if ((*itBounds) == ((Delaunay3::Vertex_handle) itVert))
        break;
    }

    bool found = false;
    for (auto cam : cams) {
      /*s<<"cams "<<cam<<" ";
       s<<std::endl;
       s<< itVert->info().getListViewingCam().size()<<" "<<std::endl;
       s<< itVert->info().getLastCam()<<" "<<std::endl;
       for(auto cc:itVert->info().getListViewingCam()){

       s<<" "<<cc<<" ";
       }
       s<<std::endl;*/
      if (std::find(itVert->info().getListViewingCam().begin(), itVert->info().getListViewingCam().end(), cam) != itVert->info().getListViewingCam().end()) {
        found = true;
      }
      //if (itVert->info().getFirstCam() <= cam && cam <= itVert->info().getLastCam()) {
        if (0 <= cam && cam <= itVert->info().getLastCam()) {
        found = true;
      }

    }

    if (itBounds == vecBoundsHandles.end() && found) { // the vertex is not a bounding vertex, so add it
      PointD3 tmp = itVert->point();
      points.push_back(tmp);
      vecVertexHandles.push_back(itVert);
      hmapVertexHandleToIndex[itVert] = points.size() - 1;

    }
  }

  //s.close();

  // Iterate over finite facets
  for (Delaunay3::Finite_facets_iterator itFacet = dt_.finite_facets_begin(); itFacet != dt_.finite_facets_end(); itFacet++) {
    // If one adjacent cell is empty, and the other is not, and if the facet contains no vertex from the bounding vertices, then add a triangle to the mesh (w/ correct orientation).
    bool bFacetCellKept = itFacet->first->info().iskeptManifold();

    bool bMirrorCellKept = itFacet->first->neighbor(itFacet->second)->info().iskeptManifold();
    if ((bFacetCellKept != bMirrorCellKept)) {
      numKept++;
    } else {
      numNotKept++;
    }
    if (!bFacetCellKept && bMirrorCellKept) {
      bool bContainsBoundsVert = false;
      for (int i = 0; i < 4; i++) {
        if (i == itFacet->second)
          continue;
        if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
          bContainsBoundsVert = true;
          break;

        }
      }
      if (!bContainsBoundsVert) {
        Delaunay3::Facet fTmp = dt_.mirror_facet(*itFacet); // The normal points inward so mirror the facet
        PointD3 tmpTri;
        std::vector<Delaunay3::Vertex_handle> vecTri;

        facetToTriangle(fTmp, vecTri);
        tmpTri = PointD3(hmapVertexHandleToIndex[vecTri[0]], hmapVertexHandleToIndex[vecTri[1]], hmapVertexHandleToIndex[vecTri[2]]);

        tris.push_back(tmpTri);
      }
    } else if (!bMirrorCellKept && bFacetCellKept) {
      bool bContainsBoundsVert = false;
      for (int i = 0; i < 4; i++) {
        if (i == itFacet->second)
          continue;
        if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
          bContainsBoundsVert = true;
          break;
        }
      }
      if (!bContainsBoundsVert) {
        Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
        PointD3 tmpTri;
        std::vector<Delaunay3::Vertex_handle> vecTri;

        facetToTriangle(fTmp, vecTri);
        tmpTri = PointD3(hmapVertexHandleToIndex[vecTri[0]], hmapVertexHandleToIndex[vecTri[1]], hmapVertexHandleToIndex[vecTri[2]]);
        tris.push_back(tmpTri);
      }
    }
  }
  std::cout << "num facets kept: " << numKept << ", num facets not kept: " << numNotKept << std::endl;
  std::cout << "tris created: " << tris.size() << ", points : " << points.size() << std::endl;
}

void OutputCreator::tetrahedraToTrianglesFreespace(std::vector<PointD3>& points, std::vector<PointD3>& tris, const float nVoteProbThresh, const bool weights) {
  std::vector<Delaunay3::Vertex_handle> vecBoundsHandles;
  std::vector<Delaunay3::Vertex_handle> vecVertexHandles;
  __gnu_cxx ::hash_map<Delaunay3::Vertex_handle, int, HashVertHandle, EqVertHandle> hmapVertexHandleToIndex;
  PointD3 matTmpPoint;

  int numFreeSpaceTets = 0;
  for (Delaunay3::Cell_iterator itCell = dt_.cells_begin(); itCell != dt_.cells_end(); itCell++) {
    bool value;
    if (!weights) {
      value = !itCell->info().isKeptByVoteCount(nVoteProbThresh);
    } else {
      value = !itCell->info().isKeptByVoteCountProb(nVoteProbThresh);
    }
    if (value) {
      ++numFreeSpaceTets;
    }
  }
  std::cout << "numFreeSpaceTets: " << numFreeSpaceTets << ", tets" << std::endl;

  // Initialize points and tris as empty:
  if (!points.empty())
    points.clear();
  if (!tris.empty())
    tris.clear();

  // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
  dt_.incident_vertices(dt_.infinite_vertex(), std::back_inserter(vecBoundsHandles));

  // Populate the model's point list, create a list of finite non-bounding vertex handles, and
  // create a useful associative maps (handle->point list index).
  for (Delaunay3::Finite_vertices_iterator itVert = dt_.finite_vertices_begin(); itVert != dt_.finite_vertices_end(); itVert++) {
    std::vector<Delaunay3::Vertex_handle>::iterator itBounds;
    for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
      if ((*itBounds) == ((Delaunay3::Vertex_handle) itVert))
        break;
    }
    if (itBounds == vecBoundsHandles.end()) { // the vertex is not a bounding vertex, so add it
      matTmpPoint = itVert->point();
      points.push_back(matTmpPoint);
      vecVertexHandles.push_back(itVert);
      hmapVertexHandleToIndex[itVert] = points.size() - 1;
    }
  }

  // Iterate over finite facets
  for (Delaunay3::Finite_facets_iterator itFacet = dt_.finite_facets_begin(); itFacet != dt_.finite_facets_end(); itFacet++) {
    // If one adjacent cell is empty, and the other is not, and if the facet contains no vertex from the bounding vertices, then add a triangle to the mesh (w/ correct orientation).
    bool bFacetCellKept = itFacet->first->info().isKeptByVoteCount(nVoteProbThresh);
    bool bMirrorCellKept = itFacet->first->neighbor(itFacet->second)->info().isKeptByVoteCount(nVoteProbThresh);
    bool toCheck2Manifold = false;
    if (bFacetCellKept && !bMirrorCellKept) {
      toCheck2Manifold = true;
    }
    for (int cur = 0; cur < 4; cur++) {
      if (bFacetCellKept && !bMirrorCellKept) {
        if (cur != itFacet->second) {
          if (itFacet->first->neighbor(cur)->info().isKeptByVoteCount(nVoteProbThresh)) {
            toCheck2Manifold = false;
          }
        }
      }

    }

    if (bFacetCellKept && !bMirrorCellKept) {

      bool bContainsBoundsVert = false;
      for (int i = 0; i < 4; i++) {
        if (i == itFacet->second)
          continue;
        if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
          bContainsBoundsVert = true;
          break;
        }
      }
      if (!bContainsBoundsVert) {
        Delaunay3::Facet fTmp = dt_.mirror_facet(*itFacet); // The normal points inward so mirror the facet
        PointD3 tmpTri;
        std::vector<Delaunay3::Vertex_handle> vecTri;

        facetToTriangle(fTmp, vecTri);
        tmpTri = PointD3(hmapVertexHandleToIndex[vecTri[0]], hmapVertexHandleToIndex[vecTri[1]], hmapVertexHandleToIndex[vecTri[2]]);
        tris.push_back(tmpTri);
      }
    } else if (bMirrorCellKept && !bFacetCellKept) {
      bool bContainsBoundsVert = false;
      for (int i = 0; i < 4; i++) {
        if (i == itFacet->second)
          continue;
        if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
          bContainsBoundsVert = true;
          break;
        }
      }
      if (!bContainsBoundsVert) {
        Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
        PointD3 tmpTri;
        std::vector<Delaunay3::Vertex_handle> vecTri;

        facetToTriangle(fTmp, vecTri);
        tmpTri = PointD3(hmapVertexHandleToIndex[vecTri[0]], hmapVertexHandleToIndex[vecTri[1]], hmapVertexHandleToIndex[vecTri[2]]);
        tris.push_back(tmpTri);
      }
    }
  }
}

void OutputCreator::facetToTriangle(const Delaunay3::Facet& f, std::vector<Delaunay3::Vertex_handle>& vecTri) {
  if (f.second == 0) {
    // Vertex handle order: 3 2 1
    vecTri.push_back(f.first->vertex(3));
    vecTri.push_back(f.first->vertex(2));
    vecTri.push_back(f.first->vertex(1));
  } else if (f.second == 1) {
    // Vertex handle order: 0 2 3
    vecTri.push_back(f.first->vertex(0));
    vecTri.push_back(f.first->vertex(2));
    vecTri.push_back(f.first->vertex(3));
  } else if (f.second == 2) {
    // Vertex handle order: 3 1 0
    vecTri.push_back(f.first->vertex(3));
    vecTri.push_back(f.first->vertex(1));
    vecTri.push_back(f.first->vertex(0));
  } else { // f->second == 3
    // Vertex handle order: 0 1 2
    vecTri.push_back(f.first->vertex(0));
    vecTri.push_back(f.first->vertex(1));
    vecTri.push_back(f.first->vertex(2));
  }
}

void OutputCreator::writeBoundaryOFF(const std::string filename, const std::vector<Delaunay3::Cell_handle> &boundaryCells) {
  std::ofstream outfile;

  std::vector<PointD3> points;
  std::vector<PointD3> tris;

  int countWrong = 0, countNotManif = 0, count = 0;
  for (auto itCellBoundary : boundaryCells) {
    if (dt_.is_cell(itCellBoundary)) {

      if (!itCellBoundary->info().iskeptManifold()) {
        countNotManif++;
      } else {

        for (int curNeigh = 0; curNeigh < 4; ++curNeigh) {
          if (!itCellBoundary->neighbor(curNeigh)->info().iskeptManifold()) {
            PointD3 tmptris;
            PointD3 tmpPoint;

            for (int curVertex = 0; curVertex < 4; ++curVertex) {
              if (curVertex != curNeigh) {
                tmpPoint = PointD3(itCellBoundary->vertex(curVertex)->point().x(), itCellBoundary->vertex(curVertex)->point().y(),
                    itCellBoundary->vertex(curVertex)->point().z());
                points.push_back(tmpPoint);

              }
            }
            tmptris = PointD3(points.size() - 3, points.size() - 2, points.size() - 1);
            tris.push_back(tmptris);
          }

        }
      }
    } else {
      countWrong++;
    }count++;

  }

  std::cout << "writeBoundaryOFF num wrong " << countWrong << " countNotManif: " << countNotManif << "total: "<<count<<std::endl;
 // tetrahedraToTrianglesFreespace(points, tris, 1);

  // Open file
  outfile.open(filename.c_str());
  if (!outfile.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return;
  }

  int numTr = tris.size();

  outfile << "OFF" << std::endl;
  outfile << points.size() << " " << numTr << " 0" << std::endl;

  // Write out lines one by one.
  for (auto itPoints : points)
    outfile << static_cast<float>(itPoints.x()) << " " << static_cast<float>(itPoints.y()) << " " << static_cast<float>(itPoints.z()) << " " << std::endl;
  for (auto itTris : tris) {
    int idxTr0 = static_cast<int>(round(itTris.x()));
    int idxTr1 = static_cast<int>(round(itTris.y()));
    int idxTr2 = static_cast<int>(round(itTris.z()));

    outfile << "3 " << idxTr0 << " " << idxTr1 << " " << idxTr2 << std::endl;
  }
  // Close the file and return
  outfile.close();
}
void OutputCreator::writeOFF(const std::string filename, int lastCam) {

  std::ofstream outfile;

  std::vector<PointD3> points;
  std::vector<PointD3> tris;
  tetrahedraToTriangles(points, tris, lastCam);

// Open file
  outfile.open(filename.c_str());
  if (!outfile.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return;
  }

  int numTr = tris.size();

  outfile << "OFF" << std::endl;
  outfile << points.size() << " " << numTr << " 0" << std::endl;

// Write out lines one by one.
  for (auto itPoints : points)
    outfile << static_cast<float>(itPoints.x()) << " " << static_cast<float>(itPoints.y()) << " " << static_cast<float>(itPoints.z()) << " " << std::endl;
  for (auto itTris : tris) {
    int idxTr0 = static_cast<int>(round(itTris.x()));
    int idxTr1 = static_cast<int>(round(itTris.y()));
    int idxTr2 = static_cast<int>(round(itTris.z()));

    outfile << "3 " << idxTr0 << " " << idxTr1 << " " << idxTr2 << std::endl;
  }
// Close the file and return
  outfile.close();
}

void OutputCreator::writeOFF(const std::string filename, std::vector<int> cams) {

  std::ofstream outfile;

  std::vector<PointD3> points;
  std::vector<PointD3> tris;
  tetrahedraToTriangles(points, tris, cams);

// Open file
  outfile.open(filename.c_str());
  if (!outfile.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return;
  }

  int numTr = tris.size();

  outfile << "OFF" << std::endl;
  outfile << points.size() << " " << numTr << " 0" << std::endl;

// Write out lines one by one.
  for (auto itPoints : points)
    outfile << static_cast<float>(itPoints.x()) << " " << static_cast<float>(itPoints.y()) << " " << static_cast<float>(itPoints.z()) << " " << std::endl;
  for (auto itTris : tris) {
    int idxTr0 = static_cast<int>(round(itTris.x()));
    int idxTr1 = static_cast<int>(round(itTris.y()));
    int idxTr2 = static_cast<int>(round(itTris.z()));

    outfile << "3 " << idxTr0 << " " << idxTr1 << " " << idxTr2 << std::endl;
  }
// Close the file and return
  outfile.close();
}

void OutputCreator::writeFreespaceOFF(const std::string filename) {

  std::ofstream outfile;

  std::vector<PointD3> points;
  std::vector<PointD3> tris;
  tetrahedraToTrianglesFreespace(points, tris, th_);

// Open file
  outfile.open(filename.c_str());
  if (!outfile.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return;
  }

  int numTr = tris.size();

  outfile << "OFF" << std::endl;
  outfile << points.size() << " " << numTr << " 0" << std::endl;

// Write out lines one by one.
  for (auto itPoints : points)
    outfile << static_cast<float>(itPoints.x()) << " " << static_cast<float>(itPoints.y()) << " " << static_cast<float>(itPoints.z()) << " " << std::endl;
  for (auto itTris : tris) {
    int idxTr0 = static_cast<int>(round(itTris.x()));
    int idxTr1 = static_cast<int>(round(itTris.y()));
    int idxTr2 = static_cast<int>(round(itTris.z()));

    outfile << "3 " << idxTr0 << " " << idxTr1 << " " << idxTr2 << std::endl;
  }
// Close the file and return
  outfile.close();
}

void OutputCreator::writeUsedVOFF(const std::string filename) {

  std::ofstream outfile;

  std::vector<PointD3> points;
  std::vector<PointD3> tris;

  for (auto it = dt_.vertices_begin(); it != dt_.vertices_end(); it++) {
    if (it->info().isUsed())
      points.push_back(it->point());
  }

// Open file
  outfile.open(filename.c_str());
  if (!outfile.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return;
  }

  int numTr = tris.size();

  outfile << "OFF" << std::endl;
  outfile << points.size() << " " << numTr << " 0" << std::endl;

// Write out lines one by one.
  for (auto itPoints : points)
    outfile << static_cast<float>(itPoints.x()) << " " << static_cast<float>(itPoints.y()) << " " << static_cast<float>(itPoints.z()) << " " << std::endl;
  for (auto itTris : tris) {
    int idxTr0 = static_cast<int>(round(itTris.x()));
    int idxTr1 = static_cast<int>(round(itTris.y()));
    int idxTr2 = static_cast<int>(round(itTris.z()));

    outfile << "3 " << idxTr0 << " " << idxTr1 << " " << idxTr2 << std::endl;
  }
// Close the file and return
  outfile.close();
}

void OutputCreator::writeNotUsedVOFF(const std::string filename) {

  std::ofstream outfile;

  std::vector<PointD3> points;
  std::vector<PointD3> tris;

  for (auto it = dt_.vertices_begin(); it != dt_.vertices_end(); it++) {
    if (it->info().isNotUsed())
      points.push_back(it->point());
  }

// Open file
  outfile.open(filename.c_str());
  if (!outfile.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return;
  }

  int numTr = tris.size();

  outfile << "OFF" << std::endl;
  outfile << points.size() << " " << numTr << " 0" << std::endl;

// Write out lines one by one.
  for (auto itPoints : points)
    outfile << static_cast<float>(itPoints.x()) << " " << static_cast<float>(itPoints.y()) << " " << static_cast<float>(itPoints.z()) << " " << std::endl;
  for (auto itTris : tris) {
    int idxTr0 = static_cast<int>(round(itTris.x()));
    int idxTr1 = static_cast<int>(round(itTris.y()));
    int idxTr2 = static_cast<int>(round(itTris.z()));

    outfile << "3 " << idxTr0 << " " << idxTr1 << " " << idxTr2 << std::endl;
  }
// Close the file and return
  outfile.close();
}

