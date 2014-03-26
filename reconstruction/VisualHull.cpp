/*
 * VisualHull.cpp
 *
 *  Created on: Mar 25, 2014
 *      Author: viharipiratla
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// This function displays the help
void
showHelp(char * program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h:	Show this help." << std::endl;
}

// This is the main function
int
main (int argc, char** argv)
{

	// Show help
	if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
		showHelp (argv[0]);
		return 0;
	}

	const char silhouettes[] = "/Users/viharipiratla/repos/btp/data/bunny_data/silhouettes/%04d.pgm";
	const char projection[] = "/Users/viharipiratla/repos/btp/data/bunny_data/calib/%04d.txt";

	int i = 0;
	while(i<36){
		char silhouette_file[100], projection_file[100];
		printf("Hull: %d\n",i);
		sprintf(silhouette_file,silhouettes,i);
		sprintf(projection_file,projection,i);
		printf("Reading image from: %s\n",silhouette_file);
		printf("Reading projection from: %s\n",projection_file);
		cv::Mat silhoeutte = cv::imread(silhouette_file,1);
		FILE* PROJ = fopen(projection_file,"r");
		Eigen::MatrixXf proj(3,4);
		fseek(PROJ,9,SEEK_SET);
		std::cerr<<"spme\n";
		for(int x=0;x<3;x++){
			float x1,x2,x3,x4;
			fscanf(PROJ,"%f %f %f %f", &x1,&x2,&x3,&x4);
			printf("%f %f %f %f\n",x1,x2,x3,x4);
			proj(x,0) = x1;
			proj(x,1) = x2;
			proj(x,2) = x3;
			proj(x,3) = x4;
		}
		std::cerr<<"Able to read\n";
		Eigen::MatrixXf invProj(3,4);
		invProj = proj.inverse();
		boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > cloud (new pcl::PointCloud<pcl::PointXYZ>());
		for(int y=0;y<silhoeutte.cols;i++)
			for(int x=0;x<silhoeutte.rows;x++)
				if(silhoeutte.at<unsigned char>(y,x)==0){
					Eigen::MatrixXf pointh(3,1);
					pointh(0,0) = x;
					pointh(1,0) = y;
					pointh(2,0) = 1;
					Eigen::Matrix3f point_3d = invProj*pointh;
					point_3d(0,0) = floor(point_3d(0,0)/2);
					point_3d(0,1) = floor(point_3d(0,1)/2);
					point_3d(0,2) = floor(point_3d(0,2)/2);

					pcl::PointXYZ some(point_3d(0,0),point_3d(1,0),point_3d(2,0));
					cloud->push_back(some);
				}
		i++;
		fclose(PROJ);
		std::string pcd_file = "final.pcd";
		pcl::io::savePCDFileASCII("final.pcd", *cloud);
	}
	return 0;
}
