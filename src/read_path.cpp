#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv) {
  std::string line;
  std::stringstream ss;
  std::stringstream stod;
  double temp;
  std::vector<double> point;
  std::vector<double> point_x;
  std::vector<double> point_y;
  std::ifstream inFile("./catkin_ws/src/following_the_path/src/path.csv", std::ios::in);
  if (!inFile) {
    std::cout << "Fail！" << std::endl;
    exit(1);
  }
  while (getline(inFile, line)) {
    double count = 0;
    ss << line;
    while (getline(ss, line, ',')) {
      stod << line;
      stod >> temp;
      point.push_back(temp);
      stod.str("");
      stod.clear();
    }
    ss.str("");
    ss.clear();
  }
  for(int i=0; i<point.size(); i++) {
    i%2 == 0 ? point_x.push_back(point[i]) : point_y.push_back(point[i]);
  }

   ros::init(argc, argv, "S_path");
   ros::NodeHandle n;
   ros::Publisher path_pub = n.advertise<nav_msgs::Path>("move_base/GlobalPlanner/plan", 1000);
   ros::Rate loop_rate(10);
   int count = 0;
   while (ros::ok()) {
     nav_msgs::Path s_path;
     geometry_msgs::PoseStamped temp;
     for(int i=0; i<point_x.size(); i++) {
        temp.pose.position.x = point_y[i];
        temp.pose.position.y = point_x[i];
        s_path.poses.push_back(temp);
     }
     path_pub.publish(s_path);
     ros::spinOnce();
     loop_rate.sleep();
     ++count;
   }

}
