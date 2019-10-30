#include <pcl/stereo/disparity_map_converter.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::RGB>::Ptr left_image (new pcl::PointCloud<pcl::RGB>);
// Fill left image cloud.
pcl::DisparityMapConverter<pcl::PointXYZI> dmc;
dmc.setBaseline (0.8387445f);
dmc.setFocalLength (368.534700f);
dmc.setImageCenterX (318.112200f);
dmc.setImageCenterY (224.334900f);
dmc.setDisparityThresholdMin(15.0f);
// Left view of the scene.
dmc.setImage (left_image);
// Disparity map of the scene.
dmc.loadDisparityMap ("disparity.txt", 640, 480);
dmc.compute(*cloud);