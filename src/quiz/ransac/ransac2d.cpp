/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
#include <vector>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  int cloudSize = cloud->points.size();
  pcl::PointXYZ point1;
  pcl::PointXYZ point2;

  float minLoss = 99999.0;
  float a, b, d;
  // For max iterations
  //
  while (maxIterations--) {
    // iterating over all the points in the cloud
    // Randomly sample subset and fit line
    // minimal subset  size is 2
    int index1 = std::rand() % cloudSize;
    int index2 = std::rand() % cloudSize;
    while (index2 == index1) {
      index2 = std::rand() % cloudSize;
    }
    point1 = cloud->points[index1];
    point2 = cloud->points[index2];
    // Calculate the loss
    // ax + by = c
    // y = mx + c
    // m = (y2-y1)/(x2-x1)
    // c = y1 - (y2-y1)(x2-x1) (x1)
    a = point1.y - point2.y;
    b = point2.x - point1.x;
    d = point1.x * point2.y - point2.x - point1.y;

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    //
    std::unordered_set<int> inliers;
    float loss = 0.0;
    for (int index = 0; index < cloudSize; index++) {
      if (inliers.count(index) > 0)
        continue;
      pcl::PointXYZ point = cloud->points[index];
      float x3 = point.x;
      float y3 = point.y;
      float distance = fabs(a * x3 + b * y3 + d) / std::sqrt(a * a + b * b);
      loss += distance;
      if (distance < distanceTol) {
        inliers.insert(index);
      }
    }
    if (loss < minLoss) {
      minLoss = loss;
      inliersResult = inliers;
    }
  }

  // Return indicies of inliers from fitted line with most inliers

  return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  int cloudSize = cloud->points.size();
  pcl::PointXYZ point1;
  pcl::PointXYZ point2;
  pcl::PointXYZ point3;

  float minLoss = 99999.0;
  float a, b, c, d;
  // For max iterations
  //
  while (maxIterations--) {
    // iterating over all the points in the cloud
    // Randomly sample subset and fit line
    // minimal subset  size is 2
    int index1 = std::rand() % cloudSize;
    int index2 = std::rand() % cloudSize;
    while (index2 == index1) {
      index2 = std::rand() % cloudSize;
    }
    point1 = cloud->points[index1];
    point2 = cloud->points[index2];
    // Calculate the loss
    // ax + by = c
    // y = mx + c
    // m = (y2-y1)/(x2-x1)
    // c = y1 - (y2-y1)(x2-x1) (x1)
    a = point1.y - point2.y;
    b = point2.x - point1.x;
    d = point1.x * point2.y - point2.x - point1.y;

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    //
    std::unordered_set<int> inliers;
    float loss = 0.0;
    for (int index = 0; index < cloudSize; index++) {
      if (inliers.count(index) > 0)
        continue;
      pcl::PointXYZ point = cloud->points[index];
      float x3 = point.x;
      float y3 = point.y;
      float distance = fabs(a * x3 + b * y3 + d) / std::sqrt(a * a + b * b);
      loss += distance;
      if (distance < distanceTol) {
        inliers.insert(index);
      }
    }
    if (loss < minLoss) {
      minLoss = loss;
      inliersResult = inliers;
    }
  }

  // Return indicies of inliers from fitted line with most inliers

  return inliersResult;
}
int main() {

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

  // TODO: Change the max iteration and distance tolerance arguments for Ransac
  // function
  std::unordered_set<int> inliers = Ransac(cloud, 20, 0.8);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
