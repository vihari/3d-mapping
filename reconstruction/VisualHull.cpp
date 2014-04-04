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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>

//#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/eigen.hpp>

#define BGD_PIX 255
#define SIG(x) (x > 0) ? 1 : ((x < 0) ? -1 : 0)


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

	const char silhouettes[] = "data/bunny_data/silhouettes/%04d.pgm";
	const char projection[] = "data/bunny_data/calib/%04d.txt";

	int i = 0;
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > cloud (new pcl::PointCloud<pcl::PointXYZI>());
	//for pig_data
	//float xLims[] =  {-6, 6};
	//float yLims[] = {-6, 6};
	//float zLims[] =  {-6, 0};
	//for bunny
	float xLims[] =  {-7.5, 7.5};
	float yLims[] = {-10, 10};
	float zLims[] =  {-7.5, 15};
	
	float step = 0.5;

	for(float x=xLims[0];x<xLims[1];x+=step){
		for(float y=yLims[0];y<yLims[1];y+=step){
			for(float z=zLims[0];z<zLims[1];z+=step){
				pcl::PointXYZI some;
				some.x = x;
				some.y = y;
				some.z = z;
				some.intensity = 1.0;
				cloud->push_back(some);
			}
		}
	}
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(cloud);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//36
	while(i<36){
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

		int prev = 1000;
		int darkPoints = 0;

		cv::Mat invProjM;
		cv::Mat projM;
		cv::eigen2cv(proj, projM);
		invProjM = projM.inv(cv::DECOMP_SVD);

		Eigen::MatrixXf invProj;
		cv::cv2eigen(invProjM, invProj);

		int nVotes = 4;

		for(int ptIdx=0;ptIdx<cloud->size();ptIdx++){
			Eigen::VectorXf point(4);
			point(0) = cloud->points[ptIdx].x;
			point(1) = cloud->points[ptIdx].y;
			point(2) = cloud->points[ptIdx].z;
			point(3) = 1;
			Eigen::Vector3f pixel = proj*point;
			float x = pixel(0)/pixel(2);
			float y = pixel(1)/pixel(2);
			if((x>=0)&&(x<silhoeutte.cols)&&(y>=0)&&(y<silhoeutte.rows)){
			  if(silhoeutte.data[silhoeutte.cols*((int)y)+(int)x] == BGD_PIX ) {
								//cloud->points[ptIdx].intensity-=0.3;
								float prevIntensity = cloud->points[ptIdx].intensity;
								cloud->points[ptIdx].intensity = SIG(prevIntensity)*(prevIntensity - 1/nVotes);
			  }
			}
		}

		boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PassThrough<pcl::PointXYZI> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("intensity");
		pass.setFilterLimits(0,1.0);
		pass.filter(*cloud_filtered);
		pcl::copyPointCloud(*cloud_filtered,*cloud);

		printf("The number of dark points are: %d\n",darkPoints);
		i++;
		fclose(PROJ);
	}
	if(cloud->size()>0)
		pcl::io::savePCDFileASCII("final.pcd", *cloud.get());
	return 0;
}
