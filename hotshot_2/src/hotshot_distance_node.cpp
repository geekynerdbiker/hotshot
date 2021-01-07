#include <ros/ros.h>
#include <gb_visual_detection_3d_msgs/BoundingBox3d.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>

#define HZ 10

class Hotshot {
  public: 
    Hotshot()  {
      hotshot_pub = nh.advertise<std_msgs::Float64MultiArray>("coord", 3);
      hotshot_sub = nh.subscribe("/darknet_ros_3d/bounding_boxes", 1, &Hotshot::hotshotCb, this);

    }

    void getCoord() {
      coord.data.clear();

      for(auto bbx : bboxes) {
        float cX = (bbx.xmin + bbx.xmax) / 2.0;
        float cY = (bbx.ymin + bbx.ymax) / 2.0;
        float cZ = (bbx.zmin + bbx.zmax) / 2.0;
        ROS_INFO("Fire: (x=%lf, y=%lf, z=%lf)\n", cX, cY, cZ);
  
        coord.data.push_back(cX);
        coord.data.push_back(cY);
        coord.data.push_back(cZ);
  
        hotshot_pub.publish(coord);
      }
    }
  
  private:
    void hotshotCb(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr & msg) {
      bboxes = msg->bounding_boxes;
    }

    ros::NodeHandle nh;
    ros::Publisher hotshot_pub;
    ros::Subscriber hotshot_sub;

    std_msgs::Float64MultiArray coord;
    std::vector<gb_visual_detection_3d_msgs::BoundingBox3d> bboxes;
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "hotshot_distance_node");
  Hotshot hotshot;
  ros::Rate loop_rate(HZ);

  while (ros::ok())
  {
    ros::spinOnce();
    hotshot.getCoord();
    loop_rate.sleep();
  }

  return 0;
}
