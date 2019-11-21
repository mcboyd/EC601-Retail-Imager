// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
// Current functionality:
// 1. Capture depth frame @ 1280x720
// 2. Capture color frame @ 1280x720
// 3. Align frames
// 4. Generate point cloud
// 5. Filter cloud to focus on box of products
// 6a. Iterate filtered cloud generating array of plane segments
// 6b. Find min Z value in all segments (only want this segment)
// 6c. Find min and max X and Y values in segment with min Z
// 7. Calculate box dimensions from min and max X, Y values
// 8. Project (x,y,z) points of corners (min and max X, Y) to (u,x) pixels of color image
// 9. Call Python & OpenCV: crop and rotate product of interest from color frame, save as PNG

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include "../../../examples/example.hpp" // Include short list of convenience functions for rendering
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <single-file/stb_image_write.h>

// Struct for managing rotation of pointcloud view
struct state {
	state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
		ml(false), offset_x(0.0f), offset_y(0.0f) {}
	double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pointxyz_vect = std::vector<pcl::PointXYZ>;
pcl::PointXYZ minZ;
pcl::PointXYZ maxX = { -100,0,0 };
pcl::PointXYZ maxY = { 0,-100,0 };
pcl::PointXYZ minx = { 100,0,0 };
pcl::PointXYZ miny = { 0,100,0 };
pointxyz_vect minmaxXY;
std::ofstream outfile;// declaration of file pointer named outfile

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);

/**
Class to encapsulate a filter alongside its options
*/
//class filter_options
//{
//public:
//	filter_options(const std::string name, rs2::filter& filter);
//	filter_options(filter_options&& other);
//	std::string filter_name;                                   //Friendly name of the filter
//	rs2::filter& filter;                                       //The filter in use
//	std::map<rs2_option, filter_slider_ui> supported_options;  //maps from an option supported by the filter, to the corresponding slider
//	std::atomic_bool is_enabled;                                        //The filter in use
//};

pcl_ptr points_to_pcl(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StopWatch watch;
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
	pcl::console::print_highlight("Time taken to convert: %f\n", watch.getTimeSeconds());
	outfile << "Time taken to convert: " << watch.getTimeSeconds()  << "\n";

	// 5. Filter cloud to focus on box of products
	pcl_ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass1;
	pass1.setInputCloud(cloud);
	pass1.setFilterFieldName("z");
	pass1.setFilterLimits(0.5, 0.67);  // Set z filter values here; can also x,y filter...
	pass1.filter(*cloud_filtered1);
	pcl::console::print_highlight("Time taken to filter z: %f\n", watch.getTimeSeconds());
	outfile << "Time taken to filter: " << watch.getTimeSeconds() << "\n";

	// 5. Filter cloud to focus on box of products
	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_filtered1);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.3, 0.2);  // Set z filter values here; can also x,y filter...
	pass.filter(*cloud_filtered);
	pcl::console::print_highlight("Time taken to filter x: %f\n", watch.getTimeSeconds());
	outfile << "Time taken to filter x: " << watch.getTimeSeconds() << "\n";

	/*std::string filename = "test_pcd.pcd";
	pcl::io::savePCDFileASCII(filename, *cloud_filtered);*/

	//printf("123");
	size_t num_points1 = cloud->size();
	size_t num_points2 = cloud_filtered->size();
	std::cout << "size of cloud: " << num_points1 << std::endl;
	std::cout << "size of cloud_filtered: " << num_points2 << std::endl;
	outfile << "size of cloud: " << num_points1 << "\n";
	outfile << "size of cloud_filtered: " << num_points2 << "\n";


	minZ = { 0,0,0 };
	minZ.x = 0;
	minZ.y = 0;
	minZ.z = 1.0;
	for (size_t i = 1; i < cloud_filtered->points.size(); ++i) {
		if (cloud_filtered->points[i].z <= minZ.z)
		{
			//std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
			minZ.x = cloud_filtered->points[i].x;
			minZ.y = cloud_filtered->points[i].y;
			minZ.z = cloud_filtered->points[i].z;
		}
	}
	pcl::console::print_highlight("Time taken to alt min: %f\n", watch.getTimeSeconds());
	std::cout << "Alt Min x: " << minZ.x << std::endl;
	std::cout << "Alt Min y: " << minZ.y << std::endl;
	std::cout << "Alt Min z: " << minZ.z << std::endl;
	outfile << "Alt Min x: " << minZ.x << "\n";
	outfile << "Alt Min y: " << minZ.y << "\n";
	outfile << "Alt Min z: " << minZ.z << "\n";

	//return cloud;
	return cloud_filtered;
}

int segment_minmax_xy(pcl_ptr& cloud_filtered)
{
	// 6a. Iterate filtered cloud generating array of plane segments
	// 6b. Find min Z value in all segments (only want this segment)
	// 6c. Find min and max X and Y values in segment with min Z
	//pointxyz_vect returnValues;
	std::vector<pcl_ptr> planes;
	int plane_ind = -1;
	int ind = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
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

	// Initi variable used in thesegmentation and later processing
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
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud(cloudFiltered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		// Put the extracted segment (plane) in the planes vector
		planes.push_back(cloud_p);

		// Create the filtering object to remove the last segment from the main cloud
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloudFiltered.swap(cloud_f);

		// 6b. Find min Z value in all segments (only want this segment)
		// Iterate through the points in this segment to see if it has the minimum z (and if so, save its vector index)
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
		ind += 1;  // Increment the vector index variable and loop again
	}

	std::cout << "plane-ind: " << plane_ind << std::endl;
	outfile << "plane-ind: " << plane_ind << "\n";

	std::string filename = "C:/bin/test_pcd.pcd";
	pcl::io::savePCDFileASCII(filename, *planes[plane_ind]);

	/*for (size_t i = 1; i < planes[plane_ind]->points.size(); ++i) {
		std::cout << planes[plane_ind]->points[i] << std::endl;
	}*/

	// Create a Convex Hull representation of the min z plane segment
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(planes[plane_ind]);
	chull.reconstruct(*cloud_hull);
	std::cout << "hull created (points): " << cloud_hull->points.size() << std::endl;
	outfile << "hull created (points): " << cloud_hull->points.size() << "\n";

	// 6c. Find min and max X and Y values in segment with min Z
	// Now, get the min/max X/Y values from the plane with the minimum Z
	for (size_t i = 1; i < planes[plane_ind]->points.size(); ++i) {
		if (planes[plane_ind]->points[i].x < minx.x)
		{
			//std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
			minx.x = planes[plane_ind]->points[i].x;
			minx.y = planes[plane_ind]->points[i].y;
			minx.z = planes[plane_ind]->points[i].z;
		}
		if (planes[plane_ind]->points[i].y < miny.y)
		{
			//std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
			miny.x = planes[plane_ind]->points[i].x;
			miny.y = planes[plane_ind]->points[i].y;
			miny.z = planes[plane_ind]->points[i].z;
		}
		if (planes[plane_ind]->points[i].x > maxX.x)
		{
			//std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
			maxX.x = planes[plane_ind]->points[i].x;
			maxX.y = planes[plane_ind]->points[i].y;
			maxX.z = planes[plane_ind]->points[i].z;
		}
		if (planes[plane_ind]->points[i].y > maxY.y)
		{
			//std::cout << "i: " << i << " , points.z: " << cloud_filtered->points[i].z << " , minz.z: " << minZ.z << std::endl;
			maxY.x = planes[plane_ind]->points[i].x;
			maxY.y = planes[plane_ind]->points[i].y;
			maxY.z = planes[plane_ind]->points[i].z;
		}

	}
	//std::cout << maxY.y << maxX.x << std::endl;

	/*returnValues.push_back(minx);
	returnValues.push_back(miny);
	returnValues.push_back(maxX);
	returnValues.push_back(maxY);
	return returnValues;*/
	return 0;
}

std::vector<float> calcDims()
{
	// 7. Calculate box dimensions from min and max X, Y values
	std::vector<float> returnValue;
	float d = sqrt(pow(miny.x - minx.x, 2) +
		pow(miny.y - minx.y, 2) +
		pow(miny.z - minx.z, 2) * 1.0);
	std::cout << std::fixed;
	std::cout << std::setprecision(2);
	std::cout << "Distance (minx->miny) is (in meters): " << d << std::endl;
	outfile << "Distance (minx->miny) is (in meters): " << d << "\n";
	returnValue.push_back(d);

	d = sqrt(pow(maxY.x - minx.x, 2) +
		pow(maxY.y - minx.y, 2) +
		pow(maxY.z - minx.z, 2) * 1.0);
	std::cout << std::fixed;
	std::cout << std::setprecision(2);
	std::cout << "Distance (minx->maxY) is (in meters): " << d << std::endl;
	outfile << "Distance (minx->maxY) is (in meters): " << d << "\n";
	returnValue.push_back(d);
	return returnValue;
}

float3 colors[]{ { 0.8f, 0.1f, 0.3f },
{ 0.1f, 0.9f, 0.5f },
};

int main(int argc, char* argv[]) try
{
	outfile.open("c:/Bin/test_log.txt", std::ios::out); // opens file named "filename" for output

	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense PCL Pointcloud Example");
	// Construct an object to manage view state
	state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, app_state);

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Configure the stream to capture both images @ 1280x720
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720);
	cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720);

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	//pipe.start(cfg);
	//The start function returns the pipeline profile which the pipeline used to start the device
	rs2::pipeline_profile profile = pipe.start(cfg);

	// Create align object
	rs2::align align(RS2_STREAM_COLOR);

	// Get camera intrinsics for later projection from (X,Y,Z) points to (U,V) pixels
	//const rs2_intrinsics i = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
	const rs2_intrinsics intr = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

	// Declare post-processing filters
	rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
	rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
	rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

	// Declare disparity transform from depth to disparity and vice versa
	const std::string disparity_filter_name = "Disparity";
	rs2::disparity_transform depth_to_disparity(true);
	rs2::disparity_transform disparity_to_depth(false);

	// Initialize a vector that holds filters and their options
	//std::vector<rs2::filter> filters;

	// The following order of emplacement will dictate the orders in which filters are applied
	/*filters.emplace_back("Decimate", dec_filter);
	filters.emplace_back("Threshold", thr_filter);
	filters.emplace_back(disparity_filter_name, depth_to_disparity);
	filters.emplace_back("Spatial", spat_filter);
	filters.emplace_back("Temporal", temp_filter);*/

	// Declaring two concurrent queues that will be used to enqueue and dequeue frames from different threads
	//rs2::frame_queue original_data;
	//rs2::frame_queue filtered_data;

	// Wait for the next set of frames from the camera
	//auto frames = pipe.wait_for_frames();
	// Camera warmup - dropping several first frames to let auto-exposure stabilize
	rs2::frameset frames;
	for (int i = 0; i < 30; i++)
	{
		//Wait for all configured streams to produce a frame
		frames = pipe.wait_for_frames();
	}

	// Align frames for projection
	frames = align.process(frames);
	//rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
	//rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();

	// 1. Capture depth frame @ 1280x720
	auto depth = frames.get_depth_frame();
	std::cout << "Depth width: " << depth.get_width() << std::endl;
	std::cout << "Depth height: " << depth.get_height() << std::endl;
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
	
	/*bool revert_disparity = false;
	for (auto&& filter : filters)
	{
		filtered = filter.filter.process(filtered);
		if (filter.filter_name == disparity_filter_name)
		{
			revert_disparity = true;
		}
	}
	if (revert_disparity)
	{
		filtered = disparity_to_depth.process(filtered);
	}*/

	// Generate the pointcloud and texture mappings
	points = pc.calculate(depth);

	// 2. Capture color frame @ 1280x720
	auto color = frames.get_color_frame();
	std::cout << "Color width: " << color.get_width() << std::endl;
	std::cout << "Color height: " << color.get_height() << std::endl;
	outfile << "Color width: " << color.get_width() << "\n";
	outfile << "Color height: " << color.get_height() << "\n";

	// Tell pointcloud object to map to this color frame
	// 3. Align frames
	//pc.map_to(color);

	// Get camera intrinsics for later projection from (X,Y,Z) points to (U,V) pixels
	//const rs2_intrinsics i = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

	// 4. Generate point cloud
	auto pcl_points = points_to_pcl(points);

	// 6a. Iterate filtered cloud generating array of plane segments
	// 6b. Find min Z value in all segments (only want this segment)
	// 6c. Find min and max X and Y values in segment with min Z (return value)
	int segment = segment_minmax_xy(pcl_points);

	// 7. Calculate box dimensions from min and max X, Y values
	std::vector<float> dimensions = calcDims();

	// 8. Project (x,y,z) points of corners (min and max X, Y) to (u,x) pixels of color image
	//float minZpixel[2];
	//float minZpoint[3] = { minZ.x, minZ.y, minZ.z };
	minmaxXY.push_back(minx);
	minmaxXY.push_back(miny);
	minmaxXY.push_back(maxX);
	minmaxXY.push_back(maxY);
	std::vector<std::array<float, 2>> minMaxPixels;  // Holds all 4 pixels representing corners of the product
	for (auto&& coord : minmaxXY) {
		float xyz[3] = { coord.x, coord.y, coord.z };
		float pixel[2];
		//rs2_project_point_to_pixel(minZpixel, &i, minZpoint);
		rs2_project_point_to_pixel(pixel, &intr, xyz);
		std::array<float, 2> tempPixel;
		tempPixel[0] = pixel[0];
		tempPixel[1] = pixel[1];
		minMaxPixels.push_back(tempPixel);
		std::cout << "Cloud x: " << xyz[0] << std::endl;
		std::cout << "Cloud y: " << xyz[1] << std::endl;
		std::cout << "Cloud z: " << xyz[2] << std::endl;
		std::cout << "Image x: " << pixel[0] << std::endl;
		std::cout << "Image y: " << pixel[1] << std::endl;
		outfile << "Cloud x: " << xyz[0] << "\n";
		outfile << "Cloud y: " << xyz[1] << "\n";
		outfile << "Cloud z: " << xyz[2] << "\n";
		outfile << "Image x: " << pixel[0] << "\n";
		outfile << "Image y: " << pixel[1] << "\n";
	}


	// Write images to disk
	std::stringstream png_file;
	//png_file << "c:/Bin/rs-save-to-disk-output-" << "test.png";
	stbi_write_png("c:/Bin/test.png", color.get_width(), color.get_height(),
		color.get_bytes_per_pixel(), color.get_data(), color.get_stride_in_bytes());
	std::cout << "Saved " << png_file.str() << std::endl;

	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pcl_points);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*cloud_filtered);

	std::vector<pcl_ptr> layers;
	layers.push_back(pcl_points);
	layers.push_back(cloud_filtered);

	while (app) // Application still alive?
	{
		draw_pointcloud(app, app_state, layers);
	}

	outfile.close();// closes file; always do this when you are done using the file
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

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, state& app_state)
{
	app.on_left_mouse = [&](bool pressed)
	{
		app_state.ml = pressed;
	};

	app.on_mouse_scroll = [&](double xoffset, double yoffset)
	{
		app_state.offset_x += static_cast<float>(xoffset);
		app_state.offset_y += static_cast<float>(yoffset);
	};

	app.on_mouse_move = [&](double x, double y)
	{
		if (app_state.ml)
		{
			app_state.yaw -= (x - app_state.last_x);
			app_state.yaw = std::max(app_state.yaw, -120.0);
			app_state.yaw = std::min(app_state.yaw, +120.0);
			app_state.pitch += (y - app_state.last_y);
			app_state.pitch = std::max(app_state.pitch, -80.0);
			app_state.pitch = std::min(app_state.pitch, +80.0);
		}
		app_state.last_x = x;
		app_state.last_y = y;
	};

	app.on_key_release = [&](int key)
	{
		if (key == 32) // Escape
		{
			app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
		}
	};
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
{
	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	float width = app.width(), height = app.height();

	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	glTranslatef(0, 0, +0.5f + app_state.offset_y * 0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;

	for (auto&& pc : points)
	{
		auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];
		glPointSize(1);
		glBegin(GL_POINTS);
		glColor3f(c.x, c.y, c.z);

		/* this segment actually prints the pointcloud */
		for (int i = 0; i < pc->points.size(); i++)
		{
			auto&& p = pc->points[i];
			if (p.z)
			{
				// upload the point and texture coordinates only for points we have depth data for
				glVertex3f(p.x, p.y, p.z);
			}
		}

		glEnd();

		// Added to display minZ, min/max x/y
		// *********BEGIN************
		glPointSize(6);  // Make it bigger so we can see it
		glBegin(GL_POINTS);
		glColor3f(0.8, 0.1, 0.3);  // Red color for minZ
		// Display the point minZ
		glVertex3f(minZ.x, minZ.y, minZ.z);

		glColor3f(0.0, 0.0, 1.0);  // Blue color (for X/Y values)
		// Display the points 
		glVertex3f(minx.x, minx.y, minx.z);
		glVertex3f(miny.x, miny.y, miny.z);
		glVertex3f(maxX.x, maxX.y, maxX.z);
		glVertex3f(maxY.x, maxY.y, maxY.z);

		glEnd();
		// **********END*************
	}

	// OpenGL cleanup
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}
