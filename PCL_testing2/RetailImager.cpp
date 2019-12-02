// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

// ******************************************************************************************
// ******************************************************************************************
// Cleaned-up production code without visualizations
// ******************************************************************************************
// ******************************************************************************************

// Current functionality:
// 1. Capture depth frame @ 1280x720
// 2. Capture color frame @ 1920x1080
// 3. Align frames
// 4. Generate point cloud
// 5. Filter cloud to focus on box of products (z and x filters)
// 6a. Iterate filtered cloud generating array of plane segments
// 6b. Find min Z value in all segments (only want the segment with min Z)
// 6c. Find minimum oriented bounding box (OBB) of segment with min Z
// 7. Calculate box dimensions from OBB
// 8. Find corner points from OBB
// 9. Project (x,y,z) points of corners to (u,v) pixels of color image
// 10. Call Python & OpenCV: crop and rotate product of interest from color frame, save as PNG
// 11. Call Python & OpenCV: grab newly exported image from (10) above, SIFT-extract its features,
// search for it in the database specified, and send matching product Id to GUI (end of process)

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <single-file/stb_image_write.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pointxyz_vect = std::vector<pcl::PointXYZ>;
pcl::PointXYZ minZ, cornerp1, cornerp2, cornerp3, cornerp4;
int width, height, longedge;
pointxyz_vect minmaxXY;
std::ofstream outfile;// declaration of file pointer named outfile
pcl::StopWatch watch; // timer to track functions

pcl_ptr points_to_pcl(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = true;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}
	outfile << "Time taken to convert: " << watch.getTimeSeconds() << "\n";

	// 5. Filter cloud to focus on box of products (z and x filters)
	pcl_ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass1;
	pass1.setInputCloud(cloud);
	pass1.setFilterFieldName("z");
	pass1.setFilterLimits(0.5, 0.68);  // Set z filter values here
	pass1.filter(*cloud_filtered1);
	outfile << "Time taken to filter z: " << watch.getTimeSeconds() << "\n";

	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_filtered1);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.3, 0.2);  // Set x filter values here to remove stand's legs
	pass.filter(*cloud_filtered);
	outfile << "Time taken to filter x: " << watch.getTimeSeconds() << "\n";

	// Stats about size of filtered cloud vs original
	size_t num_points1 = cloud->size();
	size_t num_points2 = cloud_filtered->size();
	outfile << "size of cloud: " << num_points1 << "\n";
	outfile << "size of cloud_filtered: " << num_points2 << "\n";

	return cloud_filtered;
}

int segment_minmax_xy(pcl_ptr& cloud_filtered)
{
	// 6a. Iterate filtered cloud generating array of plane segments
	// 6b. Find min Z value in all segments (only want this segment)
	// 6c. Find minimum oriented bounding box (OBB) of segment with min Z
	// 7. Calculate box dimensions from OBB
	// 8. Find corner points from OBB
	// Init variables for use in this section
	std::vector<pcl_ptr> planes;
	int plane_ind = -1;
	int ind = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

	// Create the filtering object: downsample the dataset using a leaf size of 1mm
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_filtered);
	sor.setLeafSize(0.001f, 0.001f, 0.001f);
	sor.filter(*cloudFiltered);

	// Setup for segmentation
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// Init variable used in thesegmentation and later processing
	int i = 0, nr_points = (int)cloudFiltered->points.size();
	minZ.x = 0;
	minZ.y = 0;
	minZ.z = 1.0;

	// 6a.Iterate filtered cloud generating array of plane segments
	// While 30% of the original cloud is still there, keep segmenting
	while (cloudFiltered->points.size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloudFiltered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			outfile << "Could not estimate a planar model for the given dataset.\n";
			break;
		}

		// Extract the inliers
		extract.setInputCloud(cloudFiltered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		outfile << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points.\n";

		// Put the extracted segment (plane) into the planes vector
		planes.push_back(cloud_p);

		// Create the filtering object to remove the last segment from the main cloud
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloudFiltered.swap(cloud_f);

		// 6b. Find min Z value in all segments (only want the segment with min Z)
		// Iterate through the points in this segment to see if it has the minimum z 
		// (and if so, save its vector index)
		for (size_t i = 1; i < cloud_p->points.size(); ++i) {
			if (cloud_p->points[i].z <= minZ.z)
			{
				minZ.x = cloud_p->points[i].x;
				minZ.y = cloud_p->points[i].y;
				minZ.z = cloud_p->points[i].z;
				plane_ind = ind;
			}
		}
		ind += 1;  // Increment the vector index variable and loop again
	}

	outfile << "plane-ind: " << plane_ind << "\n";

	// *******************************************************************************
	// ORIENTED BOUNDING BOX BELOW
	// FROM: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
	// *******************************************************************************
	// 6c. Find minimum oriented bounding box (OBB) of segment with min Z
	// Compute principal directions
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*planes[plane_ind], pcaCentroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*planes[plane_ind], pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	// The next line is necessary for proper orientation in some cases. The numbers come out the same without it,
	// but the signs are different and the box doesn't get correctly oriented in some cases.
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
																					

	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*planes[plane_ind], *cloudPointsProjected, projectionTransform);
	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	// Final transform
	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
	const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	// 7. Calculate box dimensions from OBB
	width = (maxPoint.z - minPoint.z) * 100;  // The z being width is a feature of the OBB calculations
	height = (maxPoint.y - minPoint.y) * 100;
	longedge = ((height/5) * 5) + 5;  // Edge used to determine which database to search
	if (width > height) longedge = ((width/5) * 5) + 5;
	outfile << "max point: " << maxPoint << "\n";
	outfile << "min point: " << minPoint << "\n";
	outfile << "depth: " << maxPoint.x - minPoint.x << "\n";
	outfile << "height: " << height << "\n";
	outfile << "width: " << width << "\n";
	outfile << "long egde: " << longedge<< "\n";

	// 8. Find corner points from OBB
	// First, build corners from min/max points in axis-aligned space
	Eigen::Vector3f p1(minPoint.x, minPoint.y, minPoint.z);  // MIN X, MIN Y - bottom left
	Eigen::Vector3f p2(minPoint.x, minPoint.y, maxPoint.z);  // MAX X, MIN Y - bottom right
	Eigen::Vector3f p3(minPoint.x, maxPoint.y, minPoint.z);  // MIN X, MAX Y - top left
	Eigen::Vector3f p4(minPoint.x, maxPoint.y, maxPoint.z);  // MAX X, MAX Y - top right

	// Then translate those points back to original-space
	p1 = eigenVectorsPCA * p1 + bboxTransform;  // Rotate point (*) then transform (+)
	p2 = eigenVectorsPCA * p2 + bboxTransform;
	p3 = eigenVectorsPCA * p3 + bboxTransform;
	p4 = eigenVectorsPCA * p4 + bboxTransform;
	
	// Finally, update corner "PointXYZ" objects with values from translated points above
	cornerp1.x = p1[0];
	cornerp1.y = p1[1];
	cornerp1.z = p1[2];
	cornerp2.x = p2[0];
	cornerp2.y = p2[1];
	cornerp2.z = p2[2];
	cornerp4.x = p4[0];
	cornerp4.y = p4[1];
	cornerp4.z = p4[2];
	cornerp3.x = p3[0];
	cornerp3.y = p3[1];
	cornerp3.z = p3[2];
	
	outfile << "p1 point (min x, min y, D): (" << p1[0] << "," << p1[1] << "," << p1[2] << ")" << "\n";
	outfile << "p2 point (max x, min y, C): (" << p2[0] << "," << p2[1] << "," << p2[2] << ")" << "\n";
	outfile << "p5 point (min x, max y, A): (" << p3[0] << "," << p3[1] << "," << p3[2] << ")" << "\n";
	outfile << "p6 point (max x, may y, B): (" << p4[0] << "," << p4[1] << "," << p4[2] << ")" << "\n";
	// *******************************************************************************
	// ORIENTED BOUNDING BOX ABOVE
	// *******************************************************************************

	return 0;
}

int main(int argc, char* argv[]) try
{
	// 1. Capture depth frame @ 1280x720
	// 2. Capture color frame @ 1920x1080
	// 3. Align frames
	// 4. Generate point cloud

	watch.reset();
	outfile.open("log.txt", std::ios::out); // opens file named "filename" for output

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Configure the stream to capture both images
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080);
	cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720);

	rs2::context ctx; // Create librealsense context for managing devices
	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
	rs2::device dev = list.front();
	auto depthSensor = dev.first<rs2::depth_sensor>();
	// set depth unit to be 0.1mm (default is 1mm)
	depthSensor.set_option(RS2_OPTION_DEPTH_UNITS, 0.0001); // 0.0001 from 0.001

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	// The start function returns the pipeline profile which the pipeline used to start the device
	rs2::pipeline_profile profile = pipe.start(cfg);

	// Create align object
	rs2::align align(RS2_STREAM_COLOR);

	// Get camera intrinsics for later projection from (X,Y,Z) points to (U,V) pixels
	const rs2_intrinsics intr = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();

	// Declare post-processing filters
	rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
	rs2::threshold_filter thr_filter;  // Threshold - removes values outside recommended range
	rs2::spatial_filter spat_filter;  // Spatial - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;  // Temporal - reduces temporal noise

	// Declare disparity transform from depth to disparity and vice versa
	const std::string disparity_filter_name = "Disparity";
	rs2::disparity_transform depth_to_disparity(true);
	rs2::disparity_transform disparity_to_depth(false);

	outfile << "Time taken to setup camera: " << watch.getTimeSeconds() << "\n";

	// Wait for the next set of frames from the camera
	// Camera warmup - dropping several first frames to let auto-exposure stabilize
	rs2::frameset frames;
	for (int i = 0; i < 30; i++)
	{
		//Wait for all configured streams to produce a frame
		frames = pipe.wait_for_frames();
	}

	outfile << "Time taken to capture useful frames: " << watch.getTimeSeconds() << "\n";

	// 3. Align frames
	// Align frames for projection
	frames = align.process(frames);

	outfile << "Time taken to align frames: " << watch.getTimeSeconds() << "\n";

	// 1. Capture depth frame @ 1280x720
	auto depth = frames.get_depth_frame();
	outfile << "Depth width: " << depth.get_width() << "\n";
	outfile << "Depth height: " << depth.get_height() << "\n";

	// For post-processing:
	rs2::frame filtered = depth; // Does not copy the frame, only adds a reference
	/* Apply filters.
	The implemented flow of the filters pipeline is in the following order:
	1. apply decimation filter
	2. apply threshold filter
	3. transform the scene into disparity domain
	4. apply spatial filter
	5. apply temporal filter
	6. revert the results back (if step Disparity filter was applied
	to depth domain (each post processing block is optional and can be applied independantly).
	*/
	filtered = dec_filter.process(filtered);
	filtered = thr_filter.process(filtered);
	filtered = depth_to_disparity.process(filtered);
	filtered = spat_filter.process(filtered);
	filtered = temp_filter.process(filtered);
	filtered = disparity_to_depth.process(filtered);

	outfile << "Time taken for depth post-processing: " << watch.getTimeSeconds() << "\n";

	// Generate the pointcloud and texture mappings
	points = pc.calculate(depth);

	outfile << "Time taken to calculate depth: " << watch.getTimeSeconds() << "\n";

	// 2. Capture color frame @ 1920x1080
	auto color = frames.get_color_frame();
	outfile << "Color width: " << color.get_width() << "\n";
	outfile << "Color height: " << color.get_height() << "\n";

	// Tell pointcloud object to map to this color frame
	pc.map_to(color);

	outfile << "Time taken to map color to point cloud: " << watch.getTimeSeconds() << "\n";

	// 4. Generate point cloud
	auto pcl_points = points_to_pcl(points);

	// 6a. Iterate filtered cloud generating array of plane segments
	// 6b. Find min Z value in all segments (only want the segment with min Z)
	// 6c. Find minimum oriented bounding box (OBB) of segment with min Z
	// 7. Calculate box dimensions from OBB
	// 8. Find corner points from OBB
	int segment = segment_minmax_xy(pcl_points);

	outfile << "Time taken to segment: " << watch.getTimeSeconds() << "\n";

	// 8. Project (x,y,z) points of corners (min and max X, Y) to (u,v) pixels of color image
	minmaxXY.push_back(cornerp1);
	minmaxXY.push_back(cornerp2);
	minmaxXY.push_back(cornerp3);
	minmaxXY.push_back(cornerp4);
	std::vector<std::array<float, 2>> minMaxPixels;  // Holds all 4 pixels representing corners of the product
	// Actually project the pixels
	for (auto&& coord : minmaxXY) {
		float xyz[3] = { coord.x, coord.y, coord.z };
		float pixel[2];
		rs2_project_point_to_pixel(pixel, &intr, xyz);
		std::array<float, 2> tempPixel;
		tempPixel[0] = pixel[0];
		tempPixel[1] = pixel[1];
		minMaxPixels.push_back(tempPixel);
		outfile << "Cloud x: " << xyz[0] << "\n";
		outfile << "Cloud y: " << xyz[1] << "\n";
		outfile << "Cloud z: " << xyz[2] << "\n";
		outfile << "Image x: " << pixel[0] << "\n";
		outfile << "Image y: " << pixel[1] << "\n";
	}

	outfile << "Time taken to project pixels: " << watch.getTimeSeconds() << "\n";

	// Write images to disk
	std::stringstream png_file;
	stbi_write_png("Img/scene.png", color.get_width(), color.get_height(),
		color.get_bytes_per_pixel(), color.get_data(), color.get_stride_in_bytes());
	outfile << "Saved image of scene: Img/scene.png\n";

	outfile << "Time taken to save PNG: " << watch.getTimeSeconds() << "\n";

	// 10. Call Python & OpenCV: crop and rotate product of interest from color frame, save as PNG
	std::string s2 = "python3 imgextract_prod.py --image Img/scene.png --coords \"[(" + std::to_string((int)minMaxPixels[0][0]) + "," + std::to_string((int)minMaxPixels[0][1]) + "),(" + std::to_string((int)minMaxPixels[1][0]) + "," + std::to_string((int)minMaxPixels[1][1]) + "),(" + std::to_string((int)minMaxPixels[2][0]) + "," + std::to_string((int)minMaxPixels[2][1]) + "),(" + std::to_string((int)minMaxPixels[3][0]) + "," + std::to_string((int)minMaxPixels[3][1]) + ")]";
	system(s2.c_str());

	outfile << "Time taken to call 1st Python script: " << watch.getTimeSeconds() << "\n";

	// 11. Call Python & OpenCV: grab newly exported image from (10) above, SIFT-extract its features,
	// search for it in the database specified, and send matching product Id to GUI (end of process)
	s2 = "c:/Bin/venv/Scripts/activate && python FLANN_Index_Search.py " + std::to_string(longedge) + " Img/product.png";
	system(s2.c_str());

	outfile << "Time taken to call 2nd Python script: " << watch.getTimeSeconds() << "\n";
	
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
