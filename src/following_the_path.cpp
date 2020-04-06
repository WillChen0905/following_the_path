#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class FollowingThePath {
public:
  FollowingThePath (ros::NodeHandle &nodehandle) : nh_(nodehandle) {
    path_type_sub_ = nh_.subscribe("path_type", 10, &FollowingThePath::PathTypeCB, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

private:
  void Circle_Path (bool boo) {
    std::cout << "Circle_Path_Start" << std::endl;
    double radius = 1.2, w = 0.314, Vx = 0, Vy = 0;
    for(int t=0; t<times_; t++) {
      for (int d=0; d<200; d++) {
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

      for (int i=0; i<6; i++) {
        vel_.linear.x = 0.3 - 0.05 * i;
        vel_.linear.y = 0.0;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<6; i++) {
        vel_.linear.x = 0.0;
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

      for (int i=0; i<6; i++) {
        vel_.linear.x = 0.0;
        vel_.linear.y = 0.3 - 0.05 * i;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<6; i++) {
        vel_.linear.x = 0.0;
        vel_.linear.y = 0.0;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<50; i++) {
        vel_.linear.x = -0.3;
        vel_.linear.y = 0.0;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<6; i++) {
        vel_.linear.x = -0.3 + 0.05 * i;
        vel_.linear.y = 0.0;
        cmd_vel_pub_.publish(vel_);
        ros::Duration(0.2).sleep();
      }

      for (int i=0; i<6; i++) {
        vel_.linear.x = 0.0;
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

  void Global_PathCB(const nav_msgs::Path path) {
    std::cout << "Global_Path_Start" << std::endl;
    global_path_sub_.shutdown();
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    for(int i=0; i<5; i++) {
      try {
        transformStamped = tfBuffer.lookupTransform("base_link", "map", ros::Time(0));
        std::cout << "Get Transform" << std::endl;
        break;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(0.2).sleep();
        continue;
      }
    }

    geometry_msgs::Pose path2aiv;
    geometry_msgs::Point temp;
    std::vector<geometry_msgs::Point> Positions;


    for(int i=0; i<path.poses.size(); i++) {
      tf2::doTransform(path.poses[i].pose, path2aiv, transformStamped);
      temp.x = path2aiv.position.x;
      temp.y = path2aiv.position.y;
      temp.z = 0.0;
      Positions.push_back(temp);
    }



    std::vector<geometry_msgs::Point> Velocity;
    for(int i=0; i<Positions.size()-4; i++) {
      temp.x = (Positions[i+1].x-Positions[i].x)/0.1;
      temp.y = (Positions[i+1].y-Positions[i].y)/0.1;
      temp.z = 0.0;
      Velocity.push_back(temp);
    }

    for(int i=0; i<Velocity.size(); i++) {
      vel_.linear.x = Velocity[i].x;
      vel_.linear.y = Velocity[i].y;
      cmd_vel_pub_.publish(vel_);
      ros::Duration(0.1).sleep();
    }

    vel_.linear.x = 0;
    vel_.linear.y = 0;
    cmd_vel_pub_.publish(vel_);
    std::cout << "Global_Path_End" << std::endl;
  }

  void PathTypeCB (const std_msgs::String &str) {
    if (str.data == "rectangle") {
      std::cout << "Rectangle" << std::endl;
      Rectangle_Path(1);
    }
    else if (str.data == "circle") {
      std::cout << "Circle" << std::endl;
      Circle_Path(1);
    }
    else if (str.data == "plan") {
      std::cout << "Global_Plan" << std::endl;
      global_path_sub_ = nh_.subscribe("move_base_node/GlobalPlanner/plan", 10, &FollowingThePath::Global_PathCB, this);
    }
    else {
      std::cout << "Not a Selected Path" << std::endl;
    }
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
