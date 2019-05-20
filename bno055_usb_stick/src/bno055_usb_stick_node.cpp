#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <tf/transform_broadcaster.h>

#include <bno055_usb_stick/bno055_usb_stick.hpp>
#include <bno055_usb_stick/decoder.hpp>
#include <bno055_usb_stick_msgs/Output.h>

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace bus = bno055_usb_stick;

ros::Publisher out_pub;
ros::Publisher imu_pub;
ros::Publisher pose_pub;
ros::Publisher mag_pub;
ros::Publisher temp_pub;
ros::Publisher heading_abs_pub;
ros::Publisher heading_rel_pub;
boost::shared_ptr< tf::TransformBroadcaster > tf_pub;

void publish(const bno055_usb_stick_msgs::Output &output, const std::string &fixed_frame_id) {
  if (out_pub.getNumSubscribers() > 0) {
    out_pub.publish(output);
  }
  if (imu_pub.getNumSubscribers() > 0) {
    imu_pub.publish(bus::Decoder::toImuMsg(output));
  }
  if (pose_pub.getNumSubscribers() > 0) {
    pose_pub.publish(bus::Decoder::toPoseMsg(output, fixed_frame_id));
  }
  if (mag_pub.getNumSubscribers() > 0) {
    mag_pub.publish(bus::Decoder::toMagMsg(output));
  }

  if (heading_abs_pub.getNumSubscribers() > 0) {
    heading_abs_pub.publish(bus::Decoder::toHeadingAbsMsg(output));
  }

  if (heading_rel_pub.getNumSubscribers() > 0) {
    heading_rel_pub.publish(bus::Decoder::toHeadingRelMsg(output));
  }

  if (temp_pub.getNumSubscribers() > 0) {
    temp_pub.publish(bus::Decoder::toTempMsg(output));
  }
  if (tf_pub) {
    tf_pub->sendTransform(bus::Decoder::toTFTransform(output, fixed_frame_id));
  }
}

int main(int argc, char *argv[]) {
  // init ROS
  ros::init(argc, argv, "bno055_usb_stick_node");
  ros::NodeHandle nh;

  // load parameters
  const std::string fixed_frame_id(ros::param::param< std::string >("~fixed_frame_id", "fixed"));
  const bool publish_tf(ros::param::param("~publish_tf", false));

  // setup publishers
  out_pub = nh.advertise< bno055_usb_stick_msgs::Output >("imu_output", 1);
  imu_pub = nh.advertise< sensor_msgs::Imu >("imu", 1);
  pose_pub = nh.advertise< geometry_msgs::PoseStamped >("imu_pose", 1);
  mag_pub = nh.advertise< sensor_msgs::MagneticField >("imu_magnetic_field", 1);
  temp_pub = nh.advertise< sensor_msgs::Temperature >("imu_temperature", 1);
  heading_abs_pub = nh.advertise< std_msgs::Float64 >("imu_heading_abs", 1);
  heading_rel_pub = nh.advertise< std_msgs::Float64 >("imu_heading_rel", 1);
  //rotspeed_pub = nh.advertise< sensor_msgs::RotSpeed >("imu_rotspeed", 1);

  if (publish_tf) {
    tf_pub.reset(new tf::TransformBroadcaster);
  }

  // construct the worker
  boost::asio::io_service asio_service;
  bus::BNO055USBStick device(asio_service, boost::bind(publish, _1, fixed_frame_id));

  // run the worker
  std::cout<<"Working!"<<std::endl;
  while (nh.ok()) {
    asio_service.run_one();
  }

  return 0;
}
