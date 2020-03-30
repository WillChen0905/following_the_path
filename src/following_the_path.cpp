#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"

class FollowingThePath {
public:
  FollowingThePath (ros::NodeHandle &nodehandle) : nh_(nodehandle) {
    path_type_sub_ = nh_.subscribe("path_type", 10, &FollowingThePath::PathTypeCB, this);
    global_path_sub_ = nh_.subscribe("global_path", 10, &FollowingThePath::Random_PathCB, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

private:
  void Circle_Path (bool boo) {
    std::cout << "Circle_Path_Start" << std::endl;

    for (int i=0; i<50; i++) {
      vel_.linear.x = 0.15;
      vel_.linear.y = 0.0;
      cmd_vel_pub_.publish(vel_);
      ros::Duration(0.2).sleep();
    }

    double radius = 1, w = 0.4, Vx = 0, Vy = 0;
    for(int t=0; t<times_; t++) {
      for (int d=0; d<165; d++) {
        Vx = radius*w*cos(w*d/10);
        Vy = radius*w*sin(w*d/10);
        vel_.linear.x = Vx;
        vel_.linear.y = Vy;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.1).sleep();
      }
      vel_.linear.x = 0;
      vel_.linear.y = 0;
      cmd_vel_pub_.publish(vel_);
    }
    std::cout << "Circle_Path_End" << std::endl;
  }

  void Rectangle_Path (bool boo) {
    std::cout << "Rectangle_Path_Start" << std::endl;
    for (int t=0; t<times_; t++) {
      for (int i=0; i<50; i++) {
        vel_.linear.x = 0.3;
        vel_.linear.y = 0.0;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<4; i++) {
        vel_.linear.x = 0.3 - 0.5 * i;
        vel_.linear.y = 0.0;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<50; i++) {
        vel_.linear.x = 0.0;
        vel_.linear.y = 0.3;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<4; i++) {
        vel_.linear.x = 0.0;
        vel_.linear.y = 0.3 - 0.5 * i;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<50; i++) {
        vel_.linear.x = -0.3;
        vel_.linear.y = 0.0;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<4; i++) {
        vel_.linear.x = -0.3 + 0.5 * i;
        vel_.linear.y = 0.0;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<50; i++) {
        vel_.linear.x = 0.0;
        vel_.linear.y = -0.3;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

    }
    vel_.linear.x = 0;
    vel_.linear.y = 0;
    cmd_vel_pub_.publish(vel_);
    std::cout << "Rectangle_Path_End" << std::endl;
  }

  void Random_PathCB(const nav_msgs::Path &path) {
    std::cout << "Random_Path_Start" << std::endl;
    std::cout << "Random_Path_End" << std::endl;
  }

  void PathTypeCB (const std_msgs::String &str) {
    switch ( str.data ) {
      case "circle":
        std::cout << "Circle" << std::endl;
        Circle_Path(1);
        break;
      case "rectangle":
        std::cout << "Rectangle" << std::endl;
        Rectangle_Path(1);
        break;
      case "random":
        std::cout << "Random" << std::endl;
        Random_PathCB(const nav_msgs::Path &path);
        break;
      default :
        std::cout << "Not a Selected Path" << std::endl;
        break;
    }

    /*
    if (str.data == "rectangle") {
      std::cout << "Rectangle" << std::endl;
      Rectangle_Path(1);
    }
    else {
      std::cout << "Circle" << std::endl;
      Circle_Path(1);
    }
    */
  }

  ros::NodeHandle nh_;
  ros::Subscriber path_type_sub_;
  ros::Subscriber global_path_sub_;
  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Twist vel_;
  int times_ = 1;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "following_the_path");
  ros::NodeHandle nh;
  FollowingThePath  FTP(nh);
  ros::spin();
  return 0;
}
