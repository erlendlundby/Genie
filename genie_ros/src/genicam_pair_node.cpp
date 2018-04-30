#include <sstream>
#include <AD_130GE.h>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) {

    using namespace std;

    string cam_pair_no;
    if (argc > 1) {
        cam_pair_no = argv[1];
    }
    else{
        cam_pair_no = "0";
    }

    string genicam_params_id = "cam_pair_" + cam_pair_no;
    ROS_INFO(genicam_params_id);

    int RGB_CAM_PORT, MONO_CAM_PORT;
    string cam_rgb_topic, cam_mono_topic;
    string camera_pair_namespace;

    ros::param::get("genicam/"+ genicam_params_id +"/pair_namespace", camera_pair_namespace);
    ros::param::param<int>("genicam/"+ genicam_params_id +"/camera_mono_port", RGB_CAM_PORT, 0);
    ros::param::param<int>("genicam/"+ genicam_params_id +"/camera_rgb_port", MONO_CAM_PORT, 1);
    ros::param::param<std::string>("genicam/"+ genicam_params_id +"/camera_rgb_topic", cam_rgb_topic, "camera_rgb");
    ros::param::param<std::string>("genicam/"+ genicam_params_id +"/camera_mono_topic", cam_mono_topic, "camera_mono");

    ROS_INFO(camera_pair_namespace.c_str());

    ros::init(argc, argv, genicam_params_id);
    ros::NodeHandle nh(camera_pair_namespace);

    ros::Publisher rgb_pub = nh.advertise<sensor_msgs::Image>(cam_rgb_topic, 100);
    ros::Publisher mono_pub = nh.advertise<sensor_msgs::Image>(cam_mono_topic, 100);

    int rgb_count = 0;
    int mono_count = 0;

    sensor_msgs::Image image_msg;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header; // empty header

    GenICam::Camera camera_rgb;
    GenICam::Camera camera_mono;

	try {
                camera_rgb.Init(RGB_CAM_PORT);
                camera_rgb.Start();

                camera_mono.Init(MONO_CAM_PORT);
                camera_mono.Start();
	}
	catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
		return 0;
	}

        ROS_INFO("Opened cameras on ports %d and %d", RGB_CAM_PORT, MONO_CAM_PORT);

    while (ros::ok()) {

        if (rgb_pub.getNumSubscribers() ||
        mono_pub.getNumSubscribers())
        {
                camera_mono.Capture();
                camera_rgb.Capture();

                header.stamp = ros::Time::now(); // time

                //rgb images
                if (rgb_pub.getNumSubscribers()) {
                        header.seq = rgb_count++;
                        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                camera_rgb.Image());
                        img_bridge.toImageMsg(image_msg);
                        rgb_pub.publish(image_msg);
                }

                //mono images
                if (mono_pub.getNumSubscribers()) {
                        header.seq = mono_count++;
                        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8,
                                camera_mono.Image());
                        img_bridge.toImageMsg(image_msg);
                        mono_pub.publish(image_msg);
                }
        }

        ros::spinOnce();
  }

  return 0;
}
