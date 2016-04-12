# DEPENDENCIES #
### OpenCV  (> 2.4.x) ###
sudo apt-get install libopencv-dev 

### CGAL (>4.3) ###
download from https://gforge.inria.fr/frs/?group_id=52 the version 4.5, 

extract it into a folder, let's call it "FOLDER" 

open the file CGAL-4.5/include/CGAL/Triangulation_hierarchy_3.h

change line 654 from  CGAL_triangulation_precondition(!is_infinite(v)); to  CGAL_triangulation_precondition(!this->is_infinite(v));

open terminal and go into FOLDER/CGAL-4.5 (*cd FOLDER/CGAL-4.5*)

then on terminal:

*mkdir build

cd build

cmake ..

make

sudo make install*

then compile the lib and install

### Eigen3 (> 3.0.5) ###
sudo apt-get install libeigen3-dev

### gmp ###
sudo apt-get install libgmp-dev


# EXAMPLES #
In data folder we put three examples: templeRing and dinoRing comes as part of the Middelbury dataset (http://vision.middlebury.edu/mview/); while 0095 is one of the KITTI sequences (raw data from http://www.cvlibs.net/datasets/kitti/).

We provide only the structure from motion output in a json file, following the specs of OpenMVG. We do not provide the image sequences to avoid copyright infringements. Anyway you can easily download them from the websites.



