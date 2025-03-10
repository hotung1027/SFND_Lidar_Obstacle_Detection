// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <Eigen/Eigenvalues>
#include <chrono>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <math.h>
#include <pcl/PointIndices.h>
#include <pcl/features/statistical_multiscale_interest_region_extraction.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(
      new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> vg;
  pcl::CropBox<PointT> cbox;

  // pcl::StatisticalMultiscaleInterestRegionExtraction<PointT> smROI;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloud_filtered);

  typename pcl::PointCloud<PointT>::Ptr cloudRegion(
      new pcl::PointCloud<PointT>);
  cbox.setMin(minPoint);
  cbox.setMax(maxPoint);
  cbox.setInputCloud(cloud_filtered);
  cbox.filter(*cloudRegion);

  // Crop the roof of the car
  std::vector<int> indices; // roof indice being remove

  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, 4, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int point : indices) {
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstacles(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr segmentedPlane(
      new pcl::PointCloud<PointT>());
  for (int index : inliers->indices) {
    segmentedPlane->points.push_back(cloud->points[index]);
  }
  // extract outliers after segmentation
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacles);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obstacles, segmentedPlane);

  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::PclSegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // TODO:: Fill in this function to find inliers for the cloud.
  typename pcl::PointCloud<PointT>::Ptr obstacles; // obstacles point cloud

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  // segmentation
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    std::cout << "Could not estimate a planar model for the given dataset."
              << std::endl;
  }
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::MySegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  auto startTime = std::chrono::high_resolution_clock::now();
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  int cloudSize = cloud->points.size();
  PointT point1;
  PointT point2;
  PointT point3;

  float minLoss = std::numeric_limits<float>::infinity();
  float a, b, c, d;
  // For max iterations
  //
  while (maxIterations--) {
    // iterating over all the points in the cloud
    // Randomly sample subset and fit line
    // minimal subset  size is 2
    std::unordered_set<int> inliers;

    while (inliers.size() < 3) {

      inliers.insert(std::rand() % cloudSize);
    }
    auto iter = inliers.begin();

    point1 = cloud->points[*iter];
    iter++;
    point2 = cloud->points[*iter];
    iter++;
    point3 = cloud->points[*iter];
    // Calculate the loss
    // Ax + By + Cz + D = 0
    // calculate the plane normal vector
    // n = v1 x v2
    //
    // m = (y2-y1)/(x2-x1)
    // c = y1 - (y2-y1)(x2-x1) (x1)

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    //
    PointT v1, v2, plane;
    // Calculate the normal vector of the plane
    v1.getVector3fMap() = point2.getVector3fMap() - point1.getVector3fMap();
    v2.getVector3fMap() = point3.getVector3fMap() - point1.getVector3fMap();
    plane.getVector3fMap() = v1.getVector3fMap().cross(v2.getVector3fMap());
    // save the normal vector tov ariale
    a = plane.x;
    b = plane.y;
    c = plane.z;
    d = -(a * point1.x + b * point1.y + c * point1.z);

    float loss = 0.0;
    for (int index = 0; index < cloudSize; index++) {
      if (inliers.count(index) > 0)
        continue;
      PointT point = cloud->points[index];
      float distance = fabs(a * point.x + b * point.y + c * point.z + d) /
                       std::sqrt(a * a + b * b + c * c);
      loss += distance;
      if (distance < distanceThreshold) {
        inliers.insert(index);
      }
    }
    if (loss - inliers.size() < minLoss - inliersResult.size()) {
      minLoss = loss;
      inliersResult = inliers;
    }
  }

  typename pcl::PointCloud<PointT>::Ptr cloudInliers(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr cloudOutliers(
      new pcl::PointCloud<PointT>());

  for (int index = 0; index < cloud->points.size(); index++) {
    PointT point = cloud->points[index];
    if (inliersResult.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(cloudOutliers, cloudInliers);

  auto endTime = std::chrono::high_resolution_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "RansacPlane Time: " << elapsedTime.count() << " ms"
            << std::endl;

  return segResult;
  // Return indicies of inliers from fitted line with most inliers
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::PclClustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::high_resolution_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles

  // create Kd-Tree representation of the point cloud
  typename pcl::search::KdTree<PointT>::Ptr tree(
      new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  // start Segmentation
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // push back clusters to clusters vector
  int j = 0;
  for (const auto &cluster : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(
        new pcl::PointCloud<PointT>());
    for (const auto &idx : cluster.indices) {
      cloud_cluster->push_back((*cloud)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
    std::cout << "Point Cloud representing the Cluster: "
              << cloud_cluster->size() << " data points." << std::endl;
  }

  auto endTime = std::chrono::high_resolution_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

//  group cluster with nearest neighbours within distance tolerance
//  @param cluster reference to the cluster, modified inplace
template <typename PointT>
void ProcessPointClouds<PointT>::groupNearestNeighbour(
    const std::vector<PointT, Eigen::aligned_allocator<PointT>> &points, int id,
    std::vector<int> &cluster, KdTree<Eigen::Vector4f> *kdtree,
    std::unordered_map<int, bool> &processedPoints, float distanceTol,
    int maxAxis) {
  // mark point as processed
  //
  processedPoints[id] = true;
  cluster.push_back(id);

  // find nearest neighbours within distance tolerance
  std::vector<int> neighbours =
      kdtree->search(points[id].getVector4fMap(), distanceTol, maxAxis);

  if (neighbours.size() > 0) {
    for (int idx : neighbours) {
      if (!processedPoints[idx]) {
        groupNearestNeighbour(points, idx, cluster, kdtree, processedPoints,
                              distanceTol, maxAxis);
      }
    }
  }
}
template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(
    const std::vector<PointT, Eigen::aligned_allocator<PointT>> &points,
    KdTree<Eigen::Vector4f> *kdtree, float distanceTol, int minSize,
    int maxSize, int maxAxis) {

  // TODO: Fill out this function to return list of indices for each cluster
  std::vector<std::vector<int>> clusters;
  std::unordered_map<int, bool> processedPoints;
  // std::vector<bool> processedPoints(points.size(), false);

  for (int i = 0; i < points.size(); i++) {
    // Check if point has already been processed
    if (!processedPoints[i]) {
      // create new cluster and start group with new point
      std::vector<int> cluster;
      groupNearestNeighbour(points, i, cluster, kdtree, processedPoints,
                            distanceTol, maxAxis);

      if (minSize <= cluster.size() && cluster.size() <= maxSize) {

        clusters.push_back(cluster);
      }
    }
  }

  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::MyClustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::high_resolution_clock::now();

  KdTree<Eigen::Vector4f> *kdtree = new KdTree<Eigen::Vector4f>();

  // find median point
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  int point_size = cloud->points.size();
  int maxAxis = 3;

  // for Perforamnce the point shold be balanced first
  // Insert Points to Kd Tree
  for (auto id = 0; id < cloud->points.size(); id++) {
    kdtree->insert(cloud->points[id].getVector4fMap(), id, maxAxis);
  }

  std::vector<std::vector<int>> clusters_indices = euclideanCluster(
      cloud->points, kdtree, clusterTolerance, minSize, maxSize, maxAxis);

  for (const auto &cluster : clusters_indices) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(
        new pcl::PointCloud<PointT>());
    for (const auto &idx : cluster) {
      cloud_cluster->push_back((*cloud)[idx]);
    }
    clusters.push_back(cloud_cluster);
  }

  auto endTime = std::chrono::high_resolution_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;
  return clusters;
}
template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;

  Eigen::Vector4f pcaCentroid;

  pcl::compute3DCentroid(*cluster, pcaCentroid);

  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);

  covariance.block<2, 2>(0, 2) = 0 * covariance.block<2, 2>(0, 2);
  covariance.block<2, 2>(2, 0) = 0 * covariance.block<2, 2>(2, 0);

  // covariance(2, 2) = 1;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
      covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenvectorsPCA = eigen_solver.eigenvectors();
  eigenvectorsPCA.col(2) = eigenvectorsPCA.col(0).cross(eigenvectorsPCA.col(1));

  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenvectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) =
      -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  typename pcl::PointCloud<PointT>::Ptr projectedPointCloud(
      new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cluster, *projectedPointCloud, projectionTransform);

  pcl::getMinMax3D(*projectedPointCloud, minPoint, maxPoint);

  const Eigen::Vector3f meanDiagonal =
      0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
  // eigenvectorsPCA.row(2) = eigenvectorsPCA.row(2) * 0;
  // eigenvectorsPCA.col(2) = eigenvectorsPCA.col(2) * 0;
  Eigen::Quaternionf bboxQuaternion(eigenvectorsPCA);
  const Eigen::Vector3f bboxTransform =
      eigenvectorsPCA * meanDiagonal + pcaCentroid.head<3>();

  BoxQ box;
  // std::cout << bboxQuaternion.vec() << std::endl;
  // bboxQuaternion.x() = 0;
  // bboxQuaternion.y() = 0;
  // bboxQuaternion.z() = 0;
  // bboxQuaternion.w() = 1;

  // bboxQuaternion.z() = sin(acos(bboxQuaternion.w()));
  box.bboxQuaternion = bboxQuaternion;
  box.bboxTransform = bboxTransform;

  box.cube_width = maxPoint.x - minPoint.x;
  box.cube_length = maxPoint.y - minPoint.y;
  box.cube_height = maxPoint.z - minPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}
