# Fast Change Detection

## Description

The program allows to identify, in real-time, changes on a 3D model from a sequence of images.

The idea is to first detect inconsistencies between pairs of images by reprojecting an image onto another one by passing through the 3D model. Ambiguities about possible inconsistencies resulting from this process are then resolved by combining multiple images. Finally, the 3D location of the change is estimated by projecting in 3D these inconsistencies.

Check out the video:

[![Fast Change Detection Video](http://img.youtube.com/vi/DEkOYf4Zzh4/0.jpg)](https://www.youtube.com/watch?v=DEkOYf4Zzh4&feature=youtu.be "Fast Change Detection Video")

For further details, see the paper ["Fast Image-Based Geometric Change Detection Given a 3D Model"](http://www.ipb.uni-bonn.de/pdfs/palazzolo2018icra.pdf).

## Key contributors

Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)

## Key assumptions

* The input images are calibrated and registered w.r.t. the 3D model

## Related publications

If you use this code for your research, please cite:

E. Palazzolo and C. Stachniss, “Fast Image-Based Geometric Change Detection Given a 3D Model”, in _Proceedings of the IEEE Int. Conf. on Robotics & Automation (ICRA)_, 2018. [PDF](http://www.ipb.uni-bonn.de/pdfs/palazzolo2018icra.pdf)

BibTeX:
```
@InProceedings{palazzolo2018icra,
Title = {{Fast Image-Based Geometric Change Detection Given a 3D Model}},
Author = {E. Palazzolo and C. Stachniss},
Booktitle = {Proceedings of the IEEE Int. Conf. on Robotics & Automation (ICRA)},
Year = {2018}
}
```

## Dependencies

* catkin
* Eigen >= 3.2
* boost >= 1.54
* OpenCV >= 2.4
* QT >= 5.2
* OpenGL >= 3.3
* [glow](https://github.com/jbehley/glow) (catkin package)
* (optional) Doxygen >= 1.8.11

## Installation guide

On Ubuntu 16.04, most of the dependencies can be installed from the package manager:
```bash
sudo apt install git libeigen3-dev libboost-all-dev qtbase5-dev libglew-dev libopencv-dev catkin
```

Additionally, make sure you have [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) and the [fetch](https://github.com/Photogrammetry-Robotics-Bonn/catkin_tools_fetch) verb installed:
```bash
sudo apt install python-pip
sudo pip install catkin_tools catkin_tools_fetch empy
```

Finally, if you also want to build the documentation you need Doxygen installed (tested only with Doxygen 1.8.11):
```bash
sudo apt install doxygen
```

If you do not have a catkin workspace already, create one:
```bash
cd
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin init
cd src
git clone https://github.com/ros/catkin.git
```
Clone the repository in your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/Photogrammetry-Robotics-Bonn/fast_change_detection.git
```
Download the additional dependencies:
```bash
catkin deps fetch
```
Then, build the project:
```bash
catkin build fast_change_detection
```
Now the project root directory (e.g. `~/catkin_ws/src/fast_change_detection`) should contain a `bin` directory containing an example binary and, if Doxygen is installed, a `docs` directory containing the documentation.

## How to use it

The `ChangeDetector` class is the core of the program. Its constructor requires
the 3D model of the environment and the options. Use the `AddImage` member function to
add an image and compute the inconsistencies with the others. Use the `GetChanges`
member function to compute and get the changes in the form of mean position and 
covariance of the points of the 3D regions.

Refer to the documentation and to the source code for further details. An example
that illustrates how to use the library is located in `src/example.cpp`.

## Examples / datafiles

After the build process, the `bin` directory in the project root directory (e.g. `~/catkin_ws/src/fast_change_detection`) will contain an example binary.
To run it execute from the command line:
```bash
cd ~/catkin_ws/src/fast_change_detection/bin
./fastcd_example DATASET_PATH
```
where `DATASET_PATH` is the path to the directory of a dataset (e.g. `~/changedetection2017/statue`).
Some example datasets can be found [here](http://www.ipb.uni-bonn.de/data/changedetection2017/).

Alternatively, the `example` directory contains some scripts to immediately test the library.
Make sure you have `wget` installed to download the dataset and `zip` to uncompress it:
```bash
sudo apt install wget zip
```
Download and uncompress the dataset:
```bash
cd ~/catkin_ws/src/fast_change_detection/example
./download_dataset.sh
```
Execute the example program:
```bash
./run_example.sh
```

A 3D visualization of the model will appear. The changes are represented by the blue ellipsoids.
Rotate the view with the mouse while the right button is pressed. The keyboard is mapped as following:
* W/A/S/D -> Move around
* Space -> Move up vertically
* C -> Move down vertically
* Shift -> Keep pressed to move faster
* 1-0 -> Teleport the camera to the pose of one of the first 10 cameras

## License

This project is licensed under the FreeBSD License. See the LICENSE.txt file for details.

## Acknowledgments

This work has partly been supported by the DFG under the grant number FOR~1505: Mapping on Demand.
