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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_corners (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;

  pcl::PCDWriter writer;
  float dist;

  vector<float> vdist;
  ///vector<vector<int> > vind{{0,0},{0,0}};
  vector<int> vi;
  vector<int> vj;
  int countav = 0;
  float distav = 0;
  float vmaxd = 0;

  float maxd1 = 0;
  int maxdi1;
  int maxdj1;

  float maxd2 = 0;
  int maxdi2;
  int maxdj2;

  for (size_t i = 1; i < cloud_hull->points.size(); ++i) {
    for (size_t j = i + 1; j < cloud_hull->points.size(); ++j) {
      distav += sqrt(pow((cloud_hull->points[i].x - cloud_hull ->points[j].x),2) + pow((cloud_hull->points[i].y - cloud_hull->points[j].y),2) + pow((cloud_hull->points[i].z - cloud_hull->points[j].z),2));
      
      countav+=1;
    }
  }
  float av = (distav/countav)/2.0;
  cout << av <<endl;

  for (size_t i = 1; i < cloud_hull->points.size(); ++i) {
    for (size_t j = i + 1; j < cloud_hull->points.size(); ++j) {
        dist = sqrt(pow((cloud_hull->points[i].x - cloud_hull ->points[j].x),2) + pow((cloud_hull->points[i].y - cloud_hull ->points[j].y),2) + pow((cloud_hull->points[i].z - cloud_hull ->points[j].z),2));
        if (dist > av) {
          vdist.push_back(dist);
          vi.push_back(i);
          vj.push_back(j);
          cout << "hi";
      }
    }
  }
  cout<<"hello";
  for (int i = 0; i < vdist.size(); i++) {
    if (vdist[i] > maxd1) {
      maxd1 = vdist[i];
      maxdi1 = vi[i];
      maxdj1 = vj[i];
    }
  }  
  for (int i = 0; i < vdist.size(); i++) {
    float xii1 = cloud_hull->points[maxdi1].x;
    float xii2 = cloud_hull->points[vi[i]].x;
    float yii1 = cloud_hull->points[maxdi1].y;
    float yii2 = cloud_hull->points[vi[i]].y;
    float zii1 = cloud_hull->points[maxdi1].z;
    float zii2 = cloud_hull->points[vi[i]].z;

    float xij1 = cloud_hull->points[maxdi1].x;
    float xij2 = cloud_hull->points[vj[i]].x;
    float yij1 = cloud_hull->points[maxdi1].y;
    float yij2 = cloud_hull->points[vj[i]].y;
    float zij1 = cloud_hull->points[maxdi1].z;
    float zij2 = cloud_hull->points[vj[i]].z;

    float xji1 = cloud_hull->points[maxdj1].x;
    float xji2 = cloud_hull->points[vi[i]].x;
    float yji1 = cloud_hull->points[maxdj1].y;
    float yji2 = cloud_hull->points[vi[i]].y;
    float zji1 = cloud_hull->points[maxdj1].z;
    float zji2 = cloud_hull->points[vi[i]].z;

    float xjj1 = cloud_hull->points[maxdj1].x;
    float xjj2 = cloud_hull->points[vj[i]].x;
    float yjj1 = cloud_hull->points[maxdj1].y;
    float yjj2 = cloud_hull->points[vj[i]].y;
    float zjj1 = cloud_hull->points[maxdj1].z;
    float zjj2 = cloud_hull->points[vj[i]].z;

    float distii = sqrt(pow(xii1-xii2,2) + pow(yii1 - yii2,2) + pow(zii1-zii2,2)); // distance of current i to maxdi1
    float distij = sqrt(pow(xij1-xij2,2) + pow(yij1 - yij2,2) + pow(zij1-zij2,2)); // distance of current j to maxdi1
    float distji = sqrt(pow(xji1-xji2,2) + pow(yji1 - yji2,2) + pow(zji1-zji2,2)); // distance of current i to maxdj1
    float distjj = sqrt(pow(xjj1-xjj2,2) + pow(yjj1 - yjj2,2) + pow(zjj1-zjj2,2)); // distance of current j to maxdj1


    if (vdist[i] > maxd2 && vi[i] != maxdi1 && vj[i] != maxdj1 && distii > av && distij > av && distji > av && distjj > av) {
      maxd2 = vdist[i];
      maxdi2 = vi[i];
      maxdj2 = vj[i];
    }
  }     
  cout << "hello";
  cloud_hull_corners->resize(4);
  cloud_hull_corners->points[0].x =  cloud_hull->points[maxdi1].x;
  cloud_hull_corners->points[0].y =  cloud_hull->points[maxdi1].y;
  cloud_hull_corners->points[0].z =  cloud_hull->points[maxdi1].z;

  cloud_hull_corners->points[1].x =  cloud_hull->points[maxdj1].x;
  cloud_hull_corners->points[1].y =  cloud_hull->points[maxdj1].y;
  cloud_hull_corners->points[1].z =  cloud_hull->points[maxdj1].z;

  cloud_hull_corners->points[2].x =  cloud_hull->points[maxdj2].x;
  cloud_hull_corners->points[2].y =  cloud_hull->points[maxdj2].y;
  cloud_hull_corners->points[2].z =  cloud_hull->points[maxdj2].z;

  cloud_hull_corners->points[3].x =  cloud_hull->points[maxdi2].x;
  cloud_hull_corners->points[3].y =  cloud_hull->points[maxdi2].y;
  cloud_hull_corners->points[3].z =  cloud_hull->points[maxdj2].z;
  cout << "hello";
  std::cout << "max dist 1: "<< maxd1 << "max dist 2: " << maxd2 << std::endl;
  std::cout << "index 1: "<< maxdi1 << " " << maxdj1 << "index 2: " << maxdi2 << " " << maxdj2 << std::endl;
  std::cout << "(" << cloud_hull->points[maxdi1].x << "," << cloud_hull->points[maxdi1].y << "," << cloud_hull->points[maxdi1].z << ")" << std::endl;
  std::cout << "(" << cloud_hull->points[maxdj1].x << "," << cloud_hull->points[maxdj1].y << "," << cloud_hull->points[maxdj1].z << ")" << std::endl;
  std::cout << "(" << cloud_hull->points[maxdi2].x << "," << cloud_hull->points[maxdi2].y << "," << cloud_hull->points[maxdi2].z << ")" << std::endl;
  std::cout << "(" << cloud_hull->points[maxdj2].x << "," << cloud_hull->points[maxdj2].y << "," << cloud_hull->points[maxdj2].z << ")" << std::endl;
  
  writer.write ("test_textured_hull.pcd", *cloud_hull, false);
  writer.write ("test_corners.pcd", *cloud_hull_corners, false);

  return (0);
}
