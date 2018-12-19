#include <ros/ros.h>
#include <project5/pid.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include "project5/Tree.h"

float error = 0;
float thDist_ = 0.5;
void split(int start, int end);
void fit();
void merge();
void visualize();
int **init_vector2(int row, int column, int value);
void scan_callback(sensor_msgs::LaserScanConstPtr msgs);
void scan_callback2(sensor_msgs::LaserScanConstPtr msgs);
void Laser2grid(int **laser_grid, sensor_msgs::LaserScanConstPtr msgs);
//std::vector<std::vector<int> > split(std::vector<std::vector<int> > segments_, int start, int end);
std::vector<std::vector<float> > points_;
std::vector<std::vector<int> > segments_;
std::vector<float> r_; 
std::vector<float> theta_; 
std::vector<float> merge_r_; 
std::vector<float> merge_theta_; 

int main(int argc, char** argv) {
    ros::init(argc, argv,"wall_following");
    ros::NodeHandle n;
    
    ros::Publisher car_ctrl_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0", 1);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, scan_callback);
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
    PID pid_ctrl;

    geometry_msgs::Pose model_pose;
    model_pose.position.x = 1.5;
    model_pose.position.y = 1.0;
    model_pose.position.z = 0.3;
    model_pose.orientation.x = 0.0;
    model_pose.orientation.y = 0.0;
    model_pose.orientation.z = 0.0;
    model_pose.orientation.w = 0.0;

    geometry_msgs::Twist model_twist;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = "racecar";
    modelstate.reference_frame = "world";
    modelstate.pose = model_pose;
    modelstate.twist = model_twist;

    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;

    gazebo_set.call(setmodelstate);
    ros::spinOnce();
    ros::Rate(0.5).sleep();
    
    // control rate, 10 Hz
    ros::Rate control_rate(10);
    
    ackermann_msgs::AckermannDriveStamped drive_msg_stamped;
    drive_msg_stamped.drive.speed = 7.0;
    drive_msg_stamped.drive.steering_angle = 0;
    
    while(ros::ok()) {
        ros::spinOnce();
        drive_msg_stamped.drive.steering_angle = pid_ctrl.get_control(error);
        //if (drive_msg_stamped.drive.steering_angle > 0.3) drive_msg_stamped.drive.speed = 7.0;
        //else drive_msg_stamped.drive.speed = 9.0;
        car_ctrl_pub.publish(drive_msg_stamped);
        control_rate.sleep();
    } 
}

void scan_callback2(sensor_msgs::LaserScanConstPtr msgs) {
    int **laser_grid = init_vector2(50, 50, 0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    Laser2grid(laser_grid, msgs);
    cv::Mat img;
    img = cv::Mat::zeros(50, 50, CV_8UC3);
    //img.at<cv::Vec3b>(0, 0)[0] = 255;
	for (int i = 0; i < 50; i++) {
		for (int j = 0; j < 50; j++) {
			if (laser_grid[i][j] == 1) {
			    img.at<cv::Vec3b>(i, 49 - j)[0] = 255;
			    img.at<cv::Vec3b>(i, 49 - j)[1] = 255;
			    img.at<cv::Vec3b>(i, 49 - j)[2] = 255;
            }
		}
	}
    /*
	//static int seed = 0;
	for(int i = 0; i < 50; i++) {	
		//seed++;
		//srand(seed);
		//random_deg[i] = (double) 0.5 - (double) 1 * (double) rand() / (double) RAND_MAX;
		random_deg[i] = 0.5 - 0.02 * i;
		//random_vel[i] = (double) 500 * (double) rand() / (double) RAND_MAX;
		random_vel[i] = (double) 300;
	}
	
	double th = d * tan(alpha) / L;
    double p1_x = L / tan(alpha) * sin(th);
	double p1_y = L / tan(alpha) * (1- cos(th));
	        
    cv::namedWindow("Mapping");
	cv::imshow("Mapping", img);
	cv::waitKey(1);*/
}

int **init_vector2(int row, int column, int value) {
	int **matrix = (int **)malloc(row * sizeof(int *));
	for (int i = 0; i < row; i++)
		matrix[i] = (int *)malloc(column * sizeof(int));
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			matrix[i][j] = value;
		}
	}
	return matrix;
}

void Laser2grid(int **laser_grid, sensor_msgs::LaserScanConstPtr msgs) {
	for(int i = 0; i < 50; i++) {
		for(int j = 0; j < 50; j++) {
			laser_grid[i][j] = 0;
		}
	}
	
	float angle_min = msgs->angle_min;
    float angle_max = msgs->angle_max;
    float angle_increment = msgs->angle_increment;
    float xmin = -4;
    float ymin = -4;
    float xunit = 8.0 / 50;
    float yunit = 8.0 / 50;
    
	for(int i = 0; i < msgs->ranges.size(); i++) {
		if(msgs->ranges[i] < 4) {
			double j = 0;
			while(j < 0.5) {
				double temp = msgs->ranges[i] + j;
				if(temp > 4) break;
				double x = temp * cos(angle_min + angle_increment * i);
				double y = temp * sin(angle_min + angle_increment * i);
				int row = round((x - xmin) / xunit);
				int column = round((y - ymin) / yunit); 
				laser_grid[row][column] = 1;
				j = j + 0.05;
			}
		}
	}	
}

void scan_callback(sensor_msgs::LaserScanConstPtr msgs) {
    /*float angle_min = msgs->angle_min;
    float angle_max = msgs->angle_max;
    float angle_increment = msgs->angle_increment;
    int index_0 = (-M_PI / 2 - angle_min) / angle_increment + 1;
    float angle_0 = angle_min + angle_increment * (index_0 - 1);
    float dist_0 = msgs->ranges[index_0 - 1];
    int index_70 = (-40 * M_PI / 180 - angle_min) / angle_increment + 1;
    float angle_70 = angle_min + angle_increment * (index_70 - 1);
    float dist_70 = msgs->ranges[index_70 - 1];
    if(dist_0 > 10) dist_0 = 10;
    if(dist_70 > 10) dist_70 = 10;
    float alpha = atan((dist_70 * cos(angle_70) - dist_0) / (dist_70 * sin(angle_70)));
    error = 1.35 - 0.1 * sin(alpha) - dist_0 * cos(alpha);*/
    float angle_min = msgs->angle_min;
    float angle_max = msgs->angle_max;
    float angle_increment = msgs->angle_increment;
    int index_0 = (M_PI / 2 - angle_min) / angle_increment + 1;
    float angle_0 = angle_min + angle_increment * (index_0 - 1);
    float dist_0 = msgs->ranges[index_0 - 1];
    int index_70 = (45 * M_PI / 180 - angle_min) / angle_increment + 1;
    float angle_70 = angle_min + angle_increment * (index_70 - 1);
    float dist_70 = msgs->ranges[index_70 - 1];
    if(dist_0 > 10) dist_0 = 10;
    if(dist_70 > 10) dist_70 = 10;
    angle_70 = M_PI * 45 / 180;
    float alpha = atan((dist_70 * cos(angle_70) - dist_0) / (dist_70 * sin(angle_70)));
    //error = -1.35 - 0.5 + 0.3 * sin(alpha) + dist_0 * cos(alpha); //7m/s
    error = -1.35 -0.55 + 0.45 * sin(alpha) + dist_0 * cos(alpha);
    /*index_0 = (-M_PI / 2 - angle_min) / angle_increment + 1;
    angle_0 = angle_min + angle_increment * (index_0 - 1);
    dist_0 = msgs->ranges[index_0 - 1];
    index_70 = (-45 * M_PI / 180 - angle_min) / angle_increment + 1;
    angle_70 = angle_min + angle_increment * (index_70 - 1);
    dist_70 = msgs->ranges[index_70 - 1];
    if(dist_0 > 10) dist_0 = 10;
    if(dist_70 > 10) dist_70 = 10;
    angle_70 = M_PI * 45 / 180;
    alpha = atan((dist_70 * cos(angle_70) - dist_0) / (dist_70 * sin(angle_70)));
    float error_temp = 1.35 - dist_0 * cos(alpha);
    error = (fabs(error) > fabs(error_temp)) ? error : error_temp;*/
    /*
    int range_count = msgs->ranges.size();
	float range_max = msgs->range_max;
	float range_min = msgs->range_min;
	//std::vector<std::vector<float> > lsr_points;
	
	int beam_skip = 1;
	points_.clear();
	segments_.clear();
	r_.clear();
	theta_.clear();
	merge_r_.clear();
	merge_theta_.clear();
		
    for(int i = 0; i < range_count; i = i + beam_skip) {
		float dist;
		if(msgs->ranges[i] <= range_min) dist = range_max;
		else dist = msgs->ranges[i];
		float angle = angle_min + i * angle_increment;
		if(angle > -M_PI / 2 && angle < M_PI / 2) {
			float x = dist * cos(angle);
			float y = dist * sin(angle);
			std::vector<float> point;
			point.push_back(x);
			point.push_back(y);
			points_.push_back(point);
		}
	}
	split(0, points_.size()-1);
	fit();
	merge();
	visualize();*/
	//printf("segment size: %f\n", merge_r_[0]);
	//error = 1.35 - fabs(merge_r_[0]);
    //printf("%f, %f, %f, %f\n", error, dist_0, dist_70, alpha * M_PI / 180);
}

void split(int start, int end)
{
	float maxDist = -1;
	int idx = end + 1;
	for(int i = start; i < end + 1; i++) {
		float Dist;
		float x0 = points_[i][0];
		float y0 = points_[i][1];
		float x1 = points_[start][0];
		float y1 = points_[start][1];
		float x2 = points_[end][0];
		float y2 = points_[end][1];
		if(sqrt(pow(y2-y1, 2)+pow(x2-x1, 2)) == 0) Dist = 0;
		else {
			Dist = fabs((y2-y1)*x0-(x2-x1)*y0+x2*y1-y2*x1) / sqrt(pow(y2-y1, 2)+pow(x2-x1, 2));
		}
		if(Dist > maxDist) {
			maxDist = Dist;
			idx = i;
		}
	}
	if(maxDist > thDist_) {
		split(start, idx);
		split(idx, end);
	}
	else {
		float deltaX = points_[end][0] - points_[start][0];
		float deltaY = points_[end][1] - points_[start][1];
		float dist = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
		if(dist > 0.09 && end-start + 1 > 10) {
		  float maxDist = 0;
			for(int i = start; i < end; i++) {
				float deltaX = points_[i+1][0] - points_[i][0];
				float deltaY = points_[i+1][1] - points_[i][1];
				float dist = sqrt(pow(deltaX, 2)+pow(deltaY, 2));
				if(dist > maxDist) maxDist = dist;
			}
			if(maxDist < 0.15 || end - start + 1 > 50) {
				std::vector<int> segment;
				segment.push_back(start);
				segment.push_back(end);
				segments_.push_back(segment);
			}
		}
	}
}

void fit()
{
	int size = segments_.size();
	for(int i = 0; i < size; i++) {
		int beginIdx = segments_[i][0];
		int endIdx = segments_[i][1];
		int numPoints = endIdx - beginIdx + 1;
		float sumX = 0, sumY = 0;
		float meanX, meanY;
		float weightX, weightY;
		float alpha;
		float r;
		for(int j = beginIdx; j < endIdx + 1; j++) {
			sumX = sumX + points_[j][0];
			sumY = sumY + points_[j][1];
		}
		weightX = sumX/numPoints;
		weightY = sumY/numPoints;
		
		float denominator = 0, numerator = 0;
		for(int j = beginIdx; j < endIdx+1; j++) {
			numerator = numerator - 2 * (weightX - points_[j][0]) * (weightY - points_[j][1]);
			denominator = denominator + pow(weightY - points_[j][1], 2) - pow(weightX-points_[j][0], 2);
		}
		alpha = 0.5 * atan2(numerator, denominator);
		r = weightX * cos(alpha) + weightY * sin(alpha);
		if(alpha < 0) {
			alpha = M_PI + alpha;
			r = -r;
		}
		r_.push_back(r);
		theta_.push_back(alpha);
	}
}

void visualize() 
{
    cv::Mat img;
    /*img = cv::Mat::zeros(500, 500, CV_8UC3);
    int img_w = 500;
    int img_h = 500;
	int size = r_.size();
		
	// DRAW LASER END POINTS	
	for(int i = 0; i < points_.size(); i++) { 
		int x_point = points_[i][0] * 50 + 250;
		int y_point = -points_[i][1] * 50 + 250;
		cv::circle(img, cv::Point(x_point, y_point), 1, cv::Scalar(0, 0, 255), CV_FILLED);
	}
	
	// DRAW LANES
	for(int i = 0; i < merge_r_.size(); i++) {
		std::string Text;
		float r = merge_r_[i];
		float t = merge_theta_[i];
		if(t >= M_PI/4 && t <= 3*M_PI/4) {
            int x1 = 0;
            int y1 = -(x1 - r*cos(t)*50 - img_w/2)/tan(t) + r*sin(t)*50 + img_h/2;
            int x2 = img_w;
            int y2 = -(x2 - r*cos(t)*50 - img_w/2)/tan(t) + r*sin(t)*50 + img_h/2;
            cv::line(img, cv::Point(x1, 500 - y1), cv::Point(x2, 500 - y2), cv::Scalar(255, 255, 255), 1);
				//cv::putText(img, Text, cv::Point(pointX, pointY), 2, 1.2, cv::Scalar(255,255,255));
		}
		else {
            int y1 = 0;
	        int x1 = -(y1 - r*sin(t)*50 - img_h/2)*tan(t) + r*cos(t)*50 + img_w/2;
            int y2 = img_h;
            int x2 = -(y2 - r*sin(t)*50 - img_h/2)*tan(t) + r*cos(t)*50 + img_w/2;
            cv::line(img, cv::Point(x1, 500 - y1), cv::Point(x2, 500 - y2), cv::Scalar(255, 255, 255), 1);
		}
	}*/
    img = cv::Mat::zeros(500, 500, CV_8UC3);
    int img_w = 500;
    int img_h = 500;
	int size = r_.size();
	
	// DRAW LASER END POINTS	
	for(int i = 0; i < points_.size(); i++) { 
		int x_point = points_[i][0] * 25 + 250;
		int y_point = -points_[i][1] * 25 + 250;
		cv::circle(img, cv::Point(x_point, y_point), 1, cv::Scalar(0, 0, 255), CV_FILLED);
	}
	
	// DRAW LANES
	for(int i = 0; i < 2; i++) {
		std::string Text;
		float r = merge_r_[i];
		float t = merge_theta_[i];
		if(t >= M_PI/4 && t <= 3*M_PI/4) {
            int x1 = 0;
            int y1 = -(x1 - r*cos(t)*25 - img_w/2)/tan(t) + r*sin(t)*25 + img_h/2;
            int x2 = img_w;
            int y2 = -(x2 - r*cos(t)*25 - img_w/2)/tan(t) + r*sin(t)*25 + img_h/2;
            cv::line(img, cv::Point(x1, 500 - y1), cv::Point(x2, 500 - y2), cv::Scalar(255, 255, 255), 1);
//				cv::putText(img, Text, cv::Point(pointX, pointY), 2, 1.2, cv::Scalar(255,255,255));
		}
		else {
            int y1 = 0;
	        int x1 = -(y1 - r*sin(t)*25 - img_h/2)*tan(t) + r*cos(t)*25 + img_w/2;
            int y2 = img_h;
            int x2 = -(y2 - r*sin(t)*25 - img_h/2)*tan(t) + r*cos(t)*25 + img_w/2;
            cv::line(img, cv::Point(x1, 500 - y1), cv::Point(x2, 500 - y2), cv::Scalar(255, 255, 255), 1);
	    }
	}
	cv::namedWindow("Mapping");
	cv::imshow("Mapping", img);
	cv::waitKey(1);
}

void merge()
{
	int size = segments_.size();
	std::vector<bool> table;
	for(int i = 0; i < size; i++) {
		table.push_back(true);
	}
	for(int i = 0; i < size; i++) {
		float r1 = r_[i];
		float th1 = theta_[i];
		for(int j = i + 1; j < size; j++) {
			float r2 = r_[j];
			float th2 = theta_[j];
			if(fabs(r1 - r2) < 0.5 && fabs(th1 - th2) < 0.5) {
				int numPoints = 0;
				float sumX = 0, sumY = 0;
				float meanX, meanY;
				float weightX, weightY;
				float alpha;
				float r;
				for(int k = segments_[i][0]; k < segments_[i][1] + 1; k++) {
					sumX = sumX+points_[k][0];
					sumY = sumY+points_[k][1];
					numPoints++;
				}
				for(int k = segments_[j][0]; k < segments_[j][1] + 1; k++) {
					sumX = sumX+points_[k][0];
					sumY = sumY+points_[k][1];
					numPoints++;
				}
				weightX = sumX / numPoints;
				weightY = sumY / numPoints;
				float denominator = 0, numerator = 0;
				for(int k = segments_[i][0]; k < segments_[i][1] + 1; k++) {
					numerator = numerator- 2 * (weightX - points_[k][0]) * (weightY - points_[k][1]);
					denominator = denominator + pow(weightY - points_[k][1], 2) - pow(weightX - points_[k][0], 2);
				}
				for(int k = segments_[j][0]; k < segments_[j][1] + 1; k++) {
					numerator = numerator - 2 * (weightX - points_[k][0]) * (weightY - points_[k][1]);
					denominator = denominator + pow(weightY - points_[k][1], 2)-pow(weightX - points_[k][0], 2);
				}
				alpha = 0.5 * atan2(numerator, denominator);
				r = weightX * cos(alpha) + weightY * sin(alpha);
				if(alpha < 0) {
					alpha = M_PI + alpha;
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
