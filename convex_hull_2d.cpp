#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <vector>
#include <math.h>
#include <bits/stdc++.h>

using namespace std;

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read ("test_pcd.pcd", *cloud);

  // Build a filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setIndices (inliers);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Create a Convex Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;

  pcl::PCDWriter writer;
  float dist;

  vector<float> vdist{0,0};
  vector<vector<int> > vind{ { 0, 0}, { 0, 0} };
  for (size_t i = 1; i < cloud_hull->points.size(); ++i) {
    for (size_t j = i + 1; j < cloud_hull->points.size(); ++j) {
      dist = sqrt(pow((cloud_hull->points[i].x - cloud_hull ->points[j].x),2) + pow((cloud_hull->points[i].y - cloud_hull ->points[j].y),2) + pow((cloud_hull->points[i].z - cloud_hull ->points[j].z),2));
      if (dist > vdist[0]) { //current distance is greater than first vector value
        if (dist > vdist[1]) { // current distance is greater than first and second vector value
          vdist[0] = vdist[1];
          vdist[1] = dist;
          vind[0][0] = vind[1][0];
          vind[0][1] = vind[1][1];
          vind[1][0] = i;
          vind[1][1] = j;
        }
        else {
        vdist[0] = dist;
        vind[0][0] = i;
        vind[0][1] = j;
        }
      }
    }
  }
  std::cout << "max dist 1: "<< vdist[0] << "max dist 2: " << vdist[1] << std::endl;
  std::cout << "index 1: "<< vind[0][0] << " " << vind[0][1] << "index 2: " << vind[1][0] << " " << vind[1][1] << std::endl;
  std::cout << "(" << cloud_hull->points[vind[0][0]].x << "," << cloud_hull->points[vind[0][0]].y << "," << cloud_hull->points[vind[0][0]].z << ")" << std::endl;
  std::cout << "(" << cloud_hull->points[vind[0][1]].x << "," << cloud_hull->points[vind[0][1]].y << "," << cloud_hull->points[vind[0][1]].z << ")" << std::endl;
  std::cout << "(" << cloud_hull->points[vind[1][0]].x << "," << cloud_hull->points[vind[1][0]].y << "," << cloud_hull->points[vind[1][0]].z << ")" << std::endl;
  std::cout << "(" << cloud_hull->points[vind[1][1]].x << "," << cloud_hull->points[vind[1][1]].y << "," << cloud_hull->points[vind[1][1]].z << ")" << std::endl;
  
  writer.write ("test_textured_hull.pcd", *cloud_hull, false);


  return (0);
}
