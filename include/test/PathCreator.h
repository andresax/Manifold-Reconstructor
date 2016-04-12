/*
 * PathCreator.h
 *
 *  Created on: 22/set/2014
 *      Author: andrea
 */

#ifndef PATHCREATOR_H_
#define PATHCREATOR_H_



#include <string>
#include <iostream>
#include <types_reconstructor.hpp>
#include <types_config.hpp>

class PathCreator {
  public:
    PathCreator(SpaceCarvingConfig spaceCarvingConfig, std::string prefix, std::string numKitti);
    PathCreator( std::string numKitti);
    virtual ~PathCreator();

    std::string getPathToSave();

    std::string getPathLog();
    std::string getPathLogPoints();
    std::string getPathBundlerOut();
    std::string getPathKittiOut();
    std::string getPathFolderImage();
    std::string getPathStats();
    std::string getPathStatsManifold();

    const std::string& getNumKitti() const {
      return numKitti_;
    }

    void setNumKitti(const std::string& numKitti) {
      numKitti_ = numKitti;
    }

    void setPrefixAndSpaceC(const std::string& prefix,const std::string& prefixFolder, const SpaceCarvingConfig& spaceCarvingConfig) {
      prefix_Folder_ = prefixFolder;
      prefix_ = prefix;

      spaceCarvingConfig_ = spaceCarvingConfig;
      createDir();
    }


  private:
    SpaceCarvingConfig spaceCarvingConfig_;
    std::string prefix_;
    std::string prefix_Folder_;
    std::string numKitti_;
    std::string pathRootDir_;
    std::string pathRoot_;



    void createDir();
};


#endif /* PATHCREATOR_H_ */
