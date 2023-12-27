// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include "boost/filesystem.hpp"
#include "kdtree.h"
#include "render/box.h"
#include <chrono>
#include <ctime>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/statistical_multiscale_interest_region_extraction.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <unordered_set>
#include <vector>
template <typename PointT> class ProcessPointClouds {
public:
  // constructor
  ProcessPointClouds();
  // deconstructor
  ~ProcessPointClouds();

  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr
  FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
              Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  SeparateClouds(pcl::PointIndices::Ptr inliers,
                 typename pcl::PointCloud<PointT>::Ptr cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  PclSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                  int maxIterations, float distanceThreshold);
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                 float distanceThreshold);
  std::vector<typename pcl::PointCloud<PointT>::Ptr>
  PclClustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                float clusterTolerance, int minSize, int maxSize);

  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  BoxQ BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

  void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  void groupNearestNeighbour(
      const std::vector<PointT, Eigen::aligned_allocator<PointT>> &points,
      int id, std::vector<int> &cluster, KdTree<Eigen::Vector4f> *kdtree,
      std::unordered_map<int, bool> &processedPoints, float distanceTol,
      int maxAxis);

  std::vector<std::vector<int>> euclideanCluster(
      const std::vector<PointT, Eigen::aligned_allocator<PointT>> &points,
      KdTree<Eigen::Vector4f> *kdtree, float distanceTol, int minSize,
      int maxSize, int maxAxis);

  std::vector<typename pcl::PointCloud<PointT>::Ptr>
  MyClustering(typename pcl::PointCloud<PointT>::Ptr cloud,
               float clusterTolerance, int minSize, int maxSize);

  typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

  std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
#endif /* PROCESSPOINTCLOUDS_H_ */
