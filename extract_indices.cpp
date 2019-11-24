#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <pcl/common/common.h>


using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
std::vector<pcl_ptr> planes;
int plane_ind = -1;
int ind = 0;

int main (int argc, char** argv)
{
  pcl::PointXYZ minZ;
  pcl::PointXYZ maxX = {0,0,0};
  pcl::PointXYZ maxY = {0,0,0};
  pcl::PointXYZ minx = {0,0,0};
  pcl::PointXYZ miny = {0,0,0};


  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there

  minZ.x = 0;
  minZ.y = 0;
  minZ.z = 1.0;
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    planes.push_back(cloud_p);
    //std::stringstream ss;
    //ss << "table_scene_lms400_plane_" << i << ".pcd";
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);

    
    for (size_t i = 1; i < cloud_p->points.size(); ++i) {
      if (cloud_p->points[i].z <= minZ.z)
      {
      //std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
        minZ.x = cloud_p->points[i].x;
        minZ.y = cloud_p->points[i].y;
        minZ.z = cloud_p->points[i].z;
        plane_ind = ind;
      }
    }
    //i++;
    ind += 1;

  }
  std::cout << minZ.z << minZ.x << minZ.y << std::endl;
  std::cout << "\n" << plane_ind << std::endl;



  for (size_t i = 1; i < planes[plane_ind]->points.size(); ++i) {
      if (planes[plane_ind]->points[i].x <= minx.x)
      {
      //std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
        minx.x = cloud_p->points[i].x;
        minx.y = cloud_p->points[i].y;
        minx.z = cloud_p->points[i].z;
      }
      if (planes[plane_ind]->points[i].y <= minx.y)
      {
      //std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
        miny.x = cloud_p->points[i].x;
        miny.y = cloud_p->points[i].y;
        miny.z = cloud_p->points[i].z;
      }
      if (planes[plane_ind]->points[i].x >= maxX.x)
      {
      //std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
        maxX.x = cloud_p->points[i].x;
        maxX.y = cloud_p->points[i].y;
        maxX.z = cloud_p->points[i].z;
      }
      if (planes[plane_ind]->points[i].y >= maxY.y)
      {
      //std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
        maxY.x = cloud_p->points[i].x;
        maxY.y = cloud_p->points[i].y;
        maxY.z = cloud_p->points[i].z;
      }

    }
  std::cout << maxY.y << maxX.x << std::endl;

  return (0);
}