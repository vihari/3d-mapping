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
//#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/eigen.hpp>

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

	const char silhouettes[] = "/home/psyche/RiseNFall/BTP/Ground_0/3d-mapping/data/pig_data/silhouettes/%04d.pgm";
	const char projection[] = "/home/psyche/RiseNFall/BTP/Ground_0/3d-mapping/data/pig_data/calib/%04d.txt";

	int i = 1;
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > cloud (new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//36
	while(i<3){
		char silhouette_file[100], projection_file[100];
		printf("Hull: %d\n",i);
		sprintf(silhouette_file,silhouettes,i);
		sprintf(projection_file,projection,i);
		printf("Reading image from: %s\n",silhouette_file);
		printf("Reading projection from: %s\n",projection_file);
		cv::Mat silhoeutte = cv::imread(silhouette_file,0);
		FILE* PROJ = fopen(projection_file,"r");
		Eigen::MatrixXf proj(3,4);
		fseek(PROJ,8,SEEK_SET);

		for(int x=0;x<3;x++){
			float x1,x2,x3,x4;
			fscanf(PROJ,"%f %f %f %f", &x1,&x2,&x3,&x4);
			printf("%f %f %f %f\n",x1,x2,x3,x4);
			proj(x,0) = x1;
			proj(x,1) = x2;
			proj(x,2) = x3;
			proj(x,3) = x4;
		}
		Eigen::VectorXf residue(3);
		residue(0) = proj(0,3);
		residue(1) = proj(1,3);
		residue(2) = proj(2,3);
		Eigen::MatrixXf proj_mat(3,3);
		for(int x=0;x<3;x++){
			proj_mat(x,0) = proj(x,0);
			proj_mat(x,1) = proj(x,1);
			proj_mat(x,2) = proj(x,2);
		}
		std::cout<<proj_mat.inverse()<<std::endl;
		//Eigen::JacobiSVD<Eigen::MatrixXf> svd(proj);
		int prev = 1000;
		int darkPoints = 0;

		cv::Mat invProjM;
		cv::Mat projM;
		cv::eigen2cv(proj, projM);
		invProjM = projM.inv(cv::DECOMP_SVD);

		Eigen::MatrixXf invProj;
		cv::cv2eigen(invProjM, invProj);

		for(int y=0;y<silhoeutte.cols;y++){
			for(int x=0;x<silhoeutte.rows;x++){
				int rows = silhoeutte.rows;
				int cols = silhoeutte.cols;
				int now = silhoeutte.data[x*cols+y];
				prev=now;
				if(silhoeutte.data[x*cols+y]<10){
					Eigen::VectorXf pointh(3,1);
					pointh(0) = (double)x;
					pointh(1) = (double)y;
					pointh(2) = (double)1;
					//std::cerr<<x<<" "<<y<<" "<<std::end;
					Eigen::VectorXf point_homogeneous = invProj*pointh;
					if(point_homogeneous.rows()==3){
						std::cout<<"The difference is: "<<std::endl;
						std::cout<<(proj_mat*point_homogeneous+(residue-pointh))<<std::endl;
					}else{
						Eigen::VectorXf test(3);
						test(0) = point_homogeneous(0);
						test(1) = point_homogeneous(1);
						test(2) = point_homogeneous(2);

						std::cout<<"The difference is: "<<std::endl;
						std::cout<<(proj_mat*test+(residue-pointh))<<std::endl;
					}
					double norm = point_homogeneous(3);
					//to avoid situations where norm is very close to zero
					if(norm>0){
						pcl::PointXYZ some(point_homogeneous(0)/norm,point_homogeneous(1)/norm,point_homogeneous(2)/norm);
						std::cout<<pointh;
						std::cout<<some<<std::endl;
						cloud->push_back(some);
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
