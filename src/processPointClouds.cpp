// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <Eigen/Eigenvalues>
#include <chrono>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
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

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloud;
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
ProcessPointClouds<PointT>::SegmentPlane(
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
ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud,
                                    int maxIterations,
                                    float distanceThreshold) {
  auto startTime = std::chrono::high_resolution_clock::now();
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  int cloudSize = cloud->points.size();
  pcl::PointXYZ point1;
  pcl::PointXYZ point2;
  pcl::PointXYZ point3;

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
    pcl::PointXYZ v1, v2, plane;
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
      pcl::PointXYZ point = cloud->points[index];
      float distance = fabs(a * point.x + b * point.y + c * point.z + d) /
                       std::sqrt(a * a + b * b + c * c);
      loss += distance;
      if (distance < distanceThreshold) {
        inliers.insert(index);
      }
    }
    if (loss < minLoss) {
      minLoss = loss;
      inliersResult = inliers;
    }
  }

  auto endTime = std::chrono::high_resolution_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "RansacPlane Time: " << elapsedTime.count() << " ms"
            << std::endl;

  // Return indicies of inliers from fitted line with most inliers

  return inliersResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
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
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
      covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenvectorsPCA = eigen_solver.eigenvectors();
  // eigenvectorsPCA(0, 2) = 0;
  // eigenvectorsPCA(1, 2) = 0;
  eigenvectorsPCA.col(2) = eigenvectorsPCA.col(0).cross(eigenvectorsPCA.col(1));
  // eigenvectorsPCA.col(2) = 0 * eigenvectorsPCA.col(2);
  // eigenvectorsPCA(2, 2) = 1;

  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenvectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) =
      -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr projectedPointCloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cluster, *projectedPointCloud, projectionTransform);

  pcl::getMinMax3D(*projectedPointCloud, minPoint, maxPoint);

  const Eigen::Vector3f meanDiagonal =
      0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
  Eigen::Quaternionf bboxQuaternion(eigenvectorsPCA);
  const Eigen::Vector3f bboxTransform =
      eigenvectorsPCA * meanDiagonal + pcaCentroid.head<3>();

  BoxQ box;
  std::cout << bboxQuaternion.vec() << std::endl;
  // bboxQuaternion.x() = 0;
  // bboxQuaternion.y() = 0;
  bboxQuaternion.z() = 0;
  bboxQuaternion.w() = 1;
  box.bboxQuaternion = bboxQuaternion;
  box.bboxTransform = bboxTransform;
  box.cube_length = maxPoint.x - minPoint.x;
  box.cube_width = maxPoint.y - minPoint.y;
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
