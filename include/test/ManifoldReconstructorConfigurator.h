/*
 * ManifoldReconstructorConfigurator.h
 *
 *  Created on: 15/apr/2015
 *      Author: andrea
 */

#ifndef MANIFOLDRECONSTRUCTORCONFIGURATOR_H_
#define MANIFOLDRECONSTRUCTORCONFIGURATOR_H_

#include <string>
#include <fstream>
#include <iostream>
#include <types_reconstructor.hpp>
#include <types_config.hpp>

class ManifoldReconstructorConfigurator {
public:
  ManifoldReconstructorConfigurator(const std::string &path);
  virtual ~ManifoldReconstructorConfigurator();
  ManifoldConfig parseConfigFile();
private:
  ManifoldConfig config_;
    std::ifstream file_;
};

#endif /* MANIFOLDRECONSTRUCTORCONFIGURATOR_H_ */
