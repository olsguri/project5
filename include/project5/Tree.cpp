#include "Tree.h"
#include <unistd.h>
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#define PI 3.14159265358979323846
#define DEG2RAD 0.017453293f

Tree::Tree() {
}

Tree::~Tree() {
}

void Tree::newScan(std::vector<std::vector<float> > points, float thDist) {
    //points_ = points;
	thDist_ = thDist;
	start_ = 0;
	//end_ = points.size()-1;
}

void Tree::generateTree()
{
    //cleanAll();
	//split(start_, end_);
	//fitting();
	//merge();
	//extractFeatures();
}

void Tree::split(int start, int end)
{
	float maxDist = -1;
	int idx = end + 1;
	for(int i = start; i < end+1; i++) {
		float Dist;
		float x0 = points_[i][0];
		float y0 = points_[i][1];
		float x1 = points_[start][0];
		float y1 = points_[start][1];
		float x2 = points_[end][0];
		float y2 = points_[end][1];
		if(sqrt(pow(y2-y1,2)+pow(x2-x1,2)) == 0) Dist = 0;
		else {
			Dist = fabs((y2-y1)*x0-(x2-x1)*y0+x2*y1-y2*x1)/sqrt(pow(y2-y1,2)+pow(x2-x1,2));
		}
		if(Dist > maxDist) {
			maxDist = Dist;
			idx = i;
		}
	}
	if(maxDist > thDist_) {
		split(start,idx);
		split(idx,end);
	}
	else {
		float deltaX = points_[end][0]-points_[start][0];
		float deltaY = points_[end][1]-points_[start][1];
		float dist = sqrt(pow(deltaX,2)+pow(deltaY,2));
		if(dist > 0.09 && end-start+1 > 10) {
		  float maxDist = 0;
			for(int i = start; i < end; i++) {
				float deltaX = points_[i+1][0]-points_[i][0];
				float deltaY = points_[i+1][1]-points_[i][1];
				float dist = sqrt(pow(deltaX,2)+pow(deltaY,2));
				if(dist > maxDist) maxDist = dist;
			}
			if(maxDist < 0.15 || end-start+1 > 50) {
				std::vector<int> segment;
				segment.push_back(start);
				segment.push_back(end);
				segments_.push_back(segment);
			}
		}
	}
}

void Tree::fitting()
{
	int size = segments_.size();
	for(int i = 0; i < size; i++) {
		int beginIdx = segments_[i][0];
		int endIdx = segments_[i][1];
		int numPoints = endIdx-beginIdx+1;
		float sumX = 0, sumY = 0;
		float meanX, meanY;
		float weightX, weightY;
		float alpha;
		float r;
		for(int j = beginIdx; j < endIdx+1; j++) {
			sumX = sumX+points_[j][0];
			sumY = sumY+points_[j][1];
		}
		weightX = sumX/numPoints;
		weightY = sumY/numPoints;
		
		float denominator = 0, numerator = 0;
		for(int j = beginIdx; j < endIdx+1; j++) {
			numerator = numerator-2*(weightX-points_[j][0])*(weightY-points_[j][1]);
			denominator = denominator+pow(weightY-points_[j][1],2)-pow(weightX-points_[j][0],2);
		}
		alpha = 0.5*atan2(numerator,denominator);
		r = weightX*cos(alpha)+weightY*sin(alpha);
		if(alpha < 0) {
			alpha = PI+alpha;
			r = -r;
		}
		r_.push_back(r);
		theta_.push_back(alpha);
	}
}

void Tree::visualizeWall() {
    cv::Mat img;
    img = cv::Mat::zeros(500,500,CV_8UC3);
    int img_w = 500;
    int img_h = 500;
	int size = merge_r_.size();
		
	// DRAW LASER END POINTS	
	for(int i = 0; i < points_.size(); i++) { 
		int x_point = points_[i][0]*50+250;
		int y_point = -points_[i][1]*50+250;
		cv::circle(img, cv::Point(x_point, y_point), 1, cv::Scalar(0,0,255), CV_FILLED);
	}
	
	cv::namedWindow("Mapping");
	cv::imshow("Mapping", img);
	cv::waitKey(1);
}

void Tree::merge()
{
	int size = segments_.size();
	std::vector<bool> table;
	for(int i = 0; i < size; i++) {
		table.push_back(true);
	}
	for(int i = 0; i < size; i++) {
		float r1 = r_[i];
		float th1 = theta_[i];
		for(int j = i+1; j < size; j++) {
			float r2 = r_[j];
			float th2 = theta_[j];
			if(fabs(r1-r2) < 0.5 && fabs(th1-th2) < 0.5) {
				int numPoints = 0;
				float sumX = 0, sumY = 0;
				float meanX, meanY;
				float weightX, weightY;
				float alpha;
				float r;
				for(int k = segments_[i][0]; k < segments_[i][1]+1; k++) {
					sumX = sumX+points_[k][0];
					sumY = sumY+points_[k][1];
					numPoints++;
				}
				for(int k = segments_[j][0]; k < segments_[j][1]+1; k++) {
					sumX = sumX+points_[k][0];
					sumY = sumY+points_[k][1];
					numPoints++;
				}
				weightX = sumX/numPoints;
				weightY = sumY/numPoints;
				float denominator = 0, numerator = 0;
				for(int k = segments_[i][0]; k < segments_[i][1]+1; k++) {
					numerator = numerator-2*(weightX-points_[k][0])*(weightY-points_[k][1]);
					denominator = denominator+pow(weightY-points_[k][1],2)-pow(weightX-points_[k][0],2);
				}
				for(int k = segments_[j][0]; k < segments_[j][1]+1; k++) {
					numerator = numerator-2*(weightX-points_[k][0])*(weightY-points_[k][1]);
					denominator = denominator+pow(weightY-points_[k][1],2)-pow(weightX-points_[k][0],2);
				}
				alpha = 0.5*atan2(numerator,denominator);
				r = weightX*cos(alpha)+weightY*sin(alpha);
				if(alpha < 0) {
					alpha = PI+alpha;
					r = -r;
				}
				merge_r_.push_back(r);	  
				merge_theta_.push_back(alpha);
				table[i] = false;
				table[j] = false;
				break;
			}
		}
	}
	for(int i = 0; i < size; i++) {
		if(table[i]) {
			merge_r_.push_back(r_[i]);
			merge_theta_.push_back(theta_[i]);
		}
	}
}
