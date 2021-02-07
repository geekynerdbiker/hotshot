#include <ros/ros.h>
#include <gb_visual_detection_3d_msgs/BoundingBox3d.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <std_msgs/Float64.h>
#include <vector>

#define HZ 10

class Hotshot {
  public: 
    Hotshot()  {
      hotshot_pub_x = nh.advertise<std_msgs::Float64>("dis_x", 100);
      hotshot_pub_y = nh.advertise<std_msgs::Float64>("dis_y", 100);
      hotshot_sub = nh.subscribe("/darknet_ros_3d/bounding_boxes", 1, &Hotshot::hotshotCb, this);
    }

    void getCoord() {
      for(auto bbx : bboxes) {
        float cX = (bbx.xmin + bbx.xmax) / 2.0;
        float cY = (bbx.ymin + bbx.ymax) / 2.0;

        ROS_INFO("Flame: (x=%lf, y=%lf)\n", cX, cY);

        if(prevX == cX && prevY == cY) count++;
        if(count == 10) {
          dis_x.data = -1;
          dis_y.data = -1;

          hotshot_pub_x.publish(dis_x);
          hotshot_pub_y.publish(dis_y);
        }

        else {
          dis_x.data = cX;
          dis_y.data = cY;

          hotshot_pub_x.publish(dis_x);
          hotshot_pub_y.publish(dis_y);
        }

        prevX = cX;
        prevX = cY;
      }
    }
  
  private:
    void hotshotCb(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr & msg) {
      bboxes = msg->bounding_boxes;
    }

    ros::NodeHandle nh;
    ros::Publisher hotshot_pub_x;
    ros::Publisher hotshot_pub_y;
    ros::Subscriber hotshot_sub;

    std_msgs::Float64 dis_x;
    std_msgs::Float64 dis_y;
    std::vector<gb_visual_detection_3d_msgs::BoundingBox3d> bboxes;

    int count = 0;
    float prevX = 0;
    float prevY = 0;

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
