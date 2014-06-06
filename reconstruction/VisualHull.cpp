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
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>

//#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/eigen.hpp>

#define BGD_PIX 0
#define FGD_PIX 1
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

	const char silhouettes[] = "data/pepsi_data/silhouettes/image-%03d.png";
	const char projection[] = "data/pepsi_data/calib/%04d.txt";

	int i = 1;
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > cloud (new pcl::PointCloud<pcl::PointXYZI>());
	float xLims[] =  {-20, 20};
	float yLims[] = {-20, 20};
	float zLims[] =  {-20, 20};

	//for pig_data
	//float xLims[] =  {-0.04, 0.04};
	//float yLims[] = {-0.001, 0.08};
	//float zLims[] =  {-0.03, 0.03};
	//for bunny
	//float xLims[] =  {-7.5, 7.5};
	//float yLims[] = {-10, 10};
	//float zLims[] =  {-7.5, 15};
	
	float step = 1;

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
	//pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	//kdtree.setInputCloud(cloud);


	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//36
	//FILE* PROJ = fopen("data/dino/dino_par.txt","r+");
	int num=100;
	//fscanf(PROJ,"%d",&num);
	char sName[40];
	int STEP=3;
	//while(i<27){
	for(i=1;i<num;i+=STEP){
		/*fscanf(PROJ,"%lf",&matrix[0][0]);
		fscanf(PROJ,"%lf",&matrix[0][1]);
		fscanf(PROJ,"%lf",&matrix[0][2]);
		fscanf(PROJ,"%lf",&matrix[1][0]);
		fscanf(PROJ,"%lf",&matrix[1][1]);
		fscanf(PROJ,"%lf",&matrix[1][2]);
		fscanf(PROJ,"%lf",&matrix[2][0]);
		fscanf(PROJ,"%lf",&matrix[2][1]);
		fscanf(PROJ,"%lf",&matrix[2][2]);
		fscanf(PROJ,"%lf",&matrix[0][3]);
		fscanf(PROJ,"%lf",&matrix[1][3]);
		fscanf(PROJ,"%lf",&matrix[2][3]);*/

		if(i>100)
			break;

		char silhouette_file[100], projection_file[100];
		printf("Hull: %d\n",i);
		sprintf(silhouette_file,silhouettes,i);
		sprintf(projection_file,projection,i);
		printf("Reading image from: %s\n",silhouette_file);
		printf("Reading projection from: %s\n",projection_file);
		cv::Mat dst = cv::imread(silhouette_file,1);
		cv::Mat thresholded;
		threshold( dst, thresholded, 0.19*255, 255, 0);
		cv::Mat silhoeutte (dst.rows,dst.cols,CV_32FC1,cv::Scalar(0));

		FILE* PROJ = fopen(projection_file,"r+");
		double matrix[3][4];
		for(int t=0;t<12;t++)
			fscanf(PROJ,"%lf",&matrix[t/4][t%4]);

		for(int x=0;x<dst.rows;x++)
			for(int y=0;y<dst.cols;y++){
				//uchar* d =  &thresholded.data[3*x*thresholded.rows+y];
				if(thresholded.at<cv::Vec3b>(x,y)[0]==255 || thresholded.at<cv::Vec3b>(x,y)[1]==255 || thresholded.at<cv::Vec3b>(x,y)[2]==255)
					silhoeutte.data[x*thresholded.rows+y] = 1;
			}
		//cv::namedWindow("Threshold",0);
		//imshow("Threshold",silhoeutte);
		//cv::waitKey(-1);

		Eigen::MatrixXf proj(3,4);
		Eigen::MatrixXf calibM(3,3);
		/*FILE* PROJ = fopen(projection_file,"r");
		fseek(PROJ,8,SEEK_SET);

		}*/
		for(int x=0;x<3;x++){
			proj(x,0) = matrix[x][0];
			proj(x,1) = matrix[x][1];
			proj(x,2) = matrix[x][2];
			proj(x,3) = matrix[x][3];
		}
		/*for(int x=0;x<3;x++){
			calibM(x,0) = calib[x][0];
			calibM(x,1) = calib[x][1];
			calibM(x,2) = calib[x][2];
		}*/

		//proj = calibM*proj;
		int prev = 1000;

		//cv::Mat invProjM;
		//cv::Mat projM;
		//cv::eigen2cv(proj, projM);
		//invProjM = projM.inv(cv::DECOMP_SVD);

		//Eigen::MatrixXf invProj;
		//cv::cv2eigen(invProjM, invProj);

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
			//printf("%f %f\n",x,y);
			if((x>=0)&&(x<silhoeutte.cols)&&(y>=0)&&(y<silhoeutte.rows)){
			  if(silhoeutte.data[silhoeutte.cols*((int)y)+(int)x] == BGD_PIX ) {
				  cloud->points[ptIdx].intensity-=0.3;
				  //TODO: This method is not working for pig data.
				  //float prevIntensity = cloud->points[ptIdx].intensity;
				  //cloud->points[ptIdx].intensity = SIG(prevIntensity)*(prevIntensity - 1/nVotes);
			  }
			}
			else{
				cloud->points[ptIdx].intensity=0;
			}
		}

		boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PassThrough<pcl::PointXYZI> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("intensity");
		pass.setFilterLimits(0,1.0);
		pass.filter(*cloud_filtered);
		pcl::copyPointCloud(*cloud_filtered,*cloud);
		printf("The size of the cloud is: %d\n",cloud->size());
		//i++;
		//fclose(PROJ);
	}
	if(cloud->size()>0)
		pcl::io::savePCDFileASCII("final.pcd", *cloud.get());
	return 0;
}
