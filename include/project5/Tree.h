#include <cmath>
#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>

class Tree
{
private:
    std::vector<std::vector<float> > points_;
    std::vector<std::vector<int> > segments_;
    std::vector<std::vector<int> > merge_segments_;
    std::vector<std::vector<float> > wall_;
    std::vector<float> merge_r_;
    std::vector<float> merge_theta_;
    std::vector<float> r_; 
    std::vector<float> theta_; 

    float thDist_;
    int start_, end_;
	void fitting();
	void split(int start, int end);
	void merge();
	
public:
	Tree();
	~Tree();
	void newScan(std::vector<std::vector<float> > points, float thDist);
	//std::tuple<float, float, float, float, float, float> extractFeatures();
	void generateTree();
	void visualizeWall();
	void cleanAll();
};
