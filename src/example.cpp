// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#include <QApplication>
#include <sstream>
#include <cstring>
#include <vector>
#include "fastcd/change_detector.h"
#include "visualizer/visualizer.h"
#include "utils/obj_reader.h"

struct Dataset {
  bool loaded = false;
  Mesh model;
  std::vector<fastcd::Image> images;
};

Dataset LoadDataset(char *path, int n_images) {
  Dataset data;
  std::ostringstream filename;
  if (path[strlen(path) - 1] != '/') {
    filename << path << "/";
  } else {
    filename << path;
  }
  std::string basepath = filename.str();

  std::cout << "Loading model... " << std::flush;
  filename << "model.obj";
  try {
    data.model = ObjReader::FromFile(filename.str());
  } catch (...) {
    std::cout << "Error reading Obj file!" << std::endl;
    return Dataset();
  }
  std::cout << "Done." << std::endl;

  std::cout << "Loading images... " << std::flush;
  for (int i = 0; i < n_images; i++) {
    fastcd::Camera cam;
    filename.str("");
    filename.clear();
    filename << basepath << "cameras.xml";
    try {
      cam.ReadCalibration(filename.str(), i);
    } catch (...) {
      std::cout << "Error reading calibration!" << std::endl;
      return Dataset();
    }

    fastcd::Image img;
    filename.str("");
    filename.clear();
    filename << basepath << "images/Image" << i << ".JPG";
    if (!img.LoadImage(filename.str(), cam)) {
      std::cout << "Error reading images!" << std::endl;
      return Dataset();
    }
    data.images.push_back(img);
  }
  std::cout << "Done." << std::endl;
  data.loaded = true;
  return data;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "Usage: fastcd_example DATASET_PATH" << std::endl;
    return -1;
  }

  // Initialize the viewer
  QApplication app(argc, argv);
  fastcd::Visualizer visualizer;

  // Load data
  Dataset data = LoadDataset(argv[1], 5);
  if (!data.loaded) return -1;

  // Initialize Change Detector
  fastcd::ChangeDetector::ChangeDetectorOptions cd_opts;
  cd_opts.cache_size = 10;
  cd_opts.max_comparisons = 4;
  cd_opts.threshold_change_area = 50;
  cd_opts.rescale_width = 500;
  cd_opts.chi_square2d = 3.219;
  cd_opts.chi_square3d = 4.642;
  fastcd::ChangeDetector change_detector(data.model, cd_opts);

  // Add the images to the sequence. Each new image is compared with the others
  // and the inconsistencies are updated.
  for (auto &img : data.images) {
    change_detector.AddImage(img, 3);
  }

  // Compute and get the changes in the environment in the form of mean position
  // and covariance of the points of the regions
  std::vector<fastcd::PointCovariance3d> changes = change_detector.GetChanges();

  // Visualize the result
  for (auto &img : data.images) {
    visualizer.addCamera(img.GetCamera(),
                     fastcd::Visualizer::rgb2float(0.0f, 1.0f, 0.0f), 1.0f);
  }
  visualizer.addMesh(data.model);
  for (auto change : changes) {
    visualizer.addPoint(change.Point().cast<float>(), 0.2,
                    fastcd::Visualizer::rgb2float(1.0f, 1.0f, 1.0f));
    Mesh ellipsoid = change.ToMesh(100, 100, 0.0f, 0.0f, 1.0f, 0.5f);
    visualizer.addMesh(ellipsoid);
  }
  visualizer.show();
  visualizer.resize(1504, 1000);

  int32_t ret = app.exec();
  return ret;
}
