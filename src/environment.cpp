/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <vector>

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI> *pointProcesssor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool render_scene = false;
  bool render_clusters = true;
  bool render_filter = false;
  bool render_box = true;
  bool render_obs = false;
  bool render_plane = true;

  // TODO:: Load PCD data files

  // ProcessPointClouds<pcl::PointXYZI> *pointProcesssor =
  //     new ProcessPointClouds<pcl::PointXYZI>();
  // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud =
  //     pointProcesssor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  // TODO:: Create point processor
  // Filter Point Cloud in ROI and Voxel Grid
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud =
      pointProcesssor->FilterCloud(inputCloud, 0.3f,
                                   Eigen::Vector4f(-500, -20, -4, 1),
                                   Eigen::Vector4f(500, 20, 40, 1));

  // Segement Ground Plane and obstacles
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
      segmentedClouds = pointProcesssor->SegmentPlane(filterCloud, 100, 0.2);

  // Clustering obstacles
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      pointProcesssor->Clustering(segmentedClouds.first, 0.6, 20, 300);
  int clusterId = 0;

  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 0, 1), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    if (render_clusters) {
      std::cout << "cluster size";
      pointProcesssor->numPoints(cluster);
      renderPointCloud(viewer, cluster,
                       "obstacle Cloud" + std::to_string(clusterId),
                       colors[clusterId % 3]);
    }
    if (render_box) {
      BoxQ box = pointProcesssor->BoundingBoxQ(cluster);
      renderBox(viewer, box, clusterId);
    }
    clusterId++;
  }

  if (render_scene) {

    renderPointCloud(viewer, inputCloud, "inputCloud");
  }

  if (render_filter) {

    renderPointCloud(viewer, filterCloud, "filterCloud");
  }
  if (render_obs) {
    renderPointCloud(viewer, segmentedClouds.first, "obstacles Cloud",
                     Color(1, 0, 0));
  }
  if (render_plane) {
    renderPointCloud(viewer, segmentedClouds.second, "Segmented Plane Cloud",
                     Color(0, 1, 0));
  }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false;
  bool render_clusters = true;
  bool render_box = true;
  bool render_obs = false;
  bool render_plane = true;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // TODO:: Create lidar sensor
  Lidar *lidar = new Lidar(cars, 0);

  // TODO:: Create point processor
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
  // renderRays(viewer,lidar->position,inputCloud);
  // renderPointCloud(viewer, inputCloud, "CarPoint");
  ProcessPointClouds<pcl::PointXYZ> pointProcesssor;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
      segmentedClouds = pointProcesssor.SegmentPlane(inputCloud, 10, 0.2);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
      pointProcesssor.Clustering(segmentedClouds.first, 1.0, 5, 500);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 0, 1), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    if (render_clusters) {
      std::cout << "cluster size";
      pointProcesssor.numPoints(cluster);
      renderPointCloud(viewer, cluster,
                       "obstacle Cloud" + std::to_string(clusterId),
                       colors[clusterId % 3]);
    }
    if (render_box) {
      BoxQ box = pointProcesssor.BoundingBoxQ(cluster);
      renderBox(viewer, box, clusterId);
    }
    clusterId++;
  }

  if (render_obs) {
    renderPointCloud(viewer, segmentedClouds.first, "obstacles Cloud",
                     Color(1, 0, 0));
  }
  if (render_plane) {
    renderPointCloud(viewer, segmentedClouds.second, "Segmented Plane Cloud",
                     Color(0, 1, 0));
  }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
  case XY:
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
    break;
  case TopDown:
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
    break;
  case Side:
    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
    break;
  case FPS:
    viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  // PCD data stream

  ProcessPointClouds<pcl::PointXYZI> *pointProcessorI =
      new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream =
      pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  // simpleHighway(viewer);

  while (!viewer->wasStopped()) {

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;

    if (streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce();
  }
}
