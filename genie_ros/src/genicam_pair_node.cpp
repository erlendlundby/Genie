#include <sstream>
#include <AD_130GE.h>
#include <string>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

int main(int argc, char **argv) {

    using namespace std;

    int RGB_CAM_1_PORT, MONO_CAM_1_PORT;
    int RGB_CAM_2_PORT, MONO_CAM_2_PORT;

    if (argc == 5) {
        RGB_CAM_1_PORT = atoi(argv[1]);
        MONO_CAM_1_PORT = atoi(argv[2]);
        RGB_CAM_2_PORT = atoi(argv[3]);
        MONO_CAM_2_PORT = atoi(argv[4]);
 
    }
    else{
        cerr << "Using default camera port settings" << endl;
        RGB_CAM_1_PORT = 0;
        MONO_CAM_1_PORT = 1;
        RGB_CAM_2_PORT = 2;
        MONO_CAM_2_PORT = 3;
    }
 
    ros::init(argc, argv, "genicam_pair");
    ros::NodeHandle nh;
    
    image_transport::ImageTransport it(nh);
    
    image_transport::Publisher rgb_1_pub = it.advertise("rgb_1/image_raw", 1);
    image_transport::Publisher mono_1_pub = it.advertise("mono_1/image_raw", 1);

    image_transport::Publisher rgb_2_pub = it.advertise("rgb_2/image_raw", 1);
    image_transport::Publisher mono_2_pub = it.advertise("mono_2/image_raw", 1);

    int count = 0;

    sensor_msgs::Image image_msg;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header; // empty header

    ros::Rate rate(5);

    GenICam::Camera camera_rgb_1;
    GenICam::Camera camera_mono_1;

    GenICam::Camera camera_rgb_2;
    GenICam::Camera camera_mono_2;

	try {
		camera_rgb_1.Init(RGB_CAM_1_PORT);
		camera_rgb_1.Start();

		camera_mono_1.Init(MONO_CAM_1_PORT);
		camera_mono_1.Start();
	
		camera_rgb_2.Init(RGB_CAM_2_PORT);
		camera_rgb_2.Start();

		camera_mono_2.Init(MONO_CAM_2_PORT);
		camera_mono_2.Start();
        }
	catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
		return 0;
	}

//        cerr << "Camera Initialised." << "RGB:" << RGB_CAM_PORT << "mono" << MONO_CAM_PORT << endl;

    while (ros::ok()) {

        if (rgb_1_pub.getNumSubscribers() ||
            mono_1_pub.getNumSubscribers()||
            rgb_2_pub.getNumSubscribers() ||
            mono_2_pub.getNumSubscribers())
        {
                header.stamp = ros::Time::now(); // time
                header.seq = count++;

                camera_rgb_1.Capture();
                camera_mono_1.Capture();
                camera_rgb_2.Capture();
                camera_mono_2.Capture();
                
                //rgb images
                if (rgb_1_pub.getNumSubscribers()) {
	                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                              camera_rgb_1.Image());
                        img_bridge.toImageMsg(image_msg);
                        rgb_1_pub.publish(image_msg);
                }

                //mono images
                if (mono_1_pub.getNumSubscribers()) {
                        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8,
                                camera_mono_1.Image());
                        img_bridge.toImageMsg(image_msg);
                        mono_1_pub.publish(image_msg);
                }
                //rgb images
                if (rgb_2_pub.getNumSubscribers()) {
                        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                camera_rgb_2.Image());
                        img_bridge.toImageMsg(image_msg);
                        rgb_2_pub.publish(image_msg);
                }

                //mono images
                if (mono_2_pub.getNumSubscribers()) {
                        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8,
                                camera_mono_2.Image());
                        img_bridge.toImageMsg(image_msg);
                        mono_2_pub.publish(image_msg);
                } 
       }

 //       rate.sleep();
        ros::spinOnce();
  }

  return 0;
}
