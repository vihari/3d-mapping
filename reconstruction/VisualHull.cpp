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
#include <Eigen/Eigen>
#include <Eigen/Dense>

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

	const char silhouettes[] = "/Users/viharipiratla/repos/btp/data/bunny_data/silhouettes/%04d.png";
	const char projection[] = "/Users/viharipiratla/repos/btp/data/bunny_data/calib/%04d.txt";

	int i = 0;
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > cloud (new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//36
	while(i<1){
		char silhouette_file[100], projection_file[100];
		printf("Hull: %d\n",i);
		sprintf(silhouette_file,silhouettes,i);
		sprintf(projection_file,projection,i);
		printf("Reading image from: %s\n",silhouette_file);
		printf("Reading projection from: %s\n",projection_file);
		cv::Mat silhoeutte = cv::imread(silhouette_file,0);
		FILE* PROJ = fopen(projection_file,"r");
		Eigen::MatrixXf proj(3,4);
		fseek(PROJ,9,SEEK_SET);

		for(int x=0;x<3;x++){
			float x1,x2,x3,x4;
			fscanf(PROJ,"%f %f %f %f", &x1,&x2,&x3,&x4);
			printf("%f %f %f %f\n",x1,x2,x3,x4);
			proj(x,0) = x1;
			proj(x,1) = x2;
			proj(x,2) = x3;
			proj(x,3) = x4;
		}

		//Eigen::JacobiSVD<Eigen::MatrixXf> svd(proj);

		int prev = 1000;
		int darkPoints = 0;
		for(int y=0;y<silhoeutte.cols;y++){
			for(int x=0;x<silhoeutte.rows;x++){
				int rows = silhoeutte.rows;
				int cols = silhoeutte.cols;
				int now = silhoeutte.data[y*rows+x];
				prev=now;
				if(silhoeutte.data[y*rows+cols]<10){
					Eigen::VectorXf pointh(3,1);
					pointh(0) = x;
					pointh(1) = y;
					pointh(2) = 1;
					std::cerr<<x<<" "<<y<<" "<<std::endl;
					Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(proj);
					Eigen::VectorXf point_homogeneous = dec.solve(pointh);

					double norm = point_homogeneous(3);
					//to avoid situations where norm is very close to zero
					norm=1;
					if(norm>0.1){
						pcl::PointXYZ some(point_homogeneous(0)/norm,point_homogeneous(1)/norm,point_homogeneous(2)/norm);
						pcl::PointXYZ some2(pointh(0),pointh(1),pointh(2));
						cloud->push_back(some2);
					}else{
						pcl::PointXYZ some(point_homogeneous(0),point_homogeneous(1),point_homogeneous(2));
						pcl::PointXYZ some2(pointh(0),pointh(1),pointh(2));
						cloud->push_back(some2);
					}
					darkPoints++;
				}
			}
		}
		printf("The number of dark points are: %d\n",darkPoints);
		i++;
		fclose(PROJ);
	}
	if(cloud->size()>0)
		pcl::io::savePCDFileASCII("final.pcd", *cloud.get());
	return 0;
}
