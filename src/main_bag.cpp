/*
 * Offline runner: read a rosbag in-process (no rosbag play), run FAST-LIVO2, log pose/vel via log_value.
 */
#include "LIVMapper.h"
#include "common_lib.h"

#include <cv_bridge/cv_bridge.h>
#include <livox_ros_driver/CustomMsg.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <iostream>
#include <string>

namespace
{

bool dispatchMessage(LIVMapper &mapper, const rosbag::MessageInstance &m, int lidar_type, const std::string &lid_topic,
                     const std::string &imu_topic, const std::string &img_topic)
{
  const std::string &topic = m.getTopic();

  if (topic == imu_topic)
  {
    auto msg = m.instantiate<sensor_msgs::Imu>();
    if (!msg)
      return false;
    mapper.imu_cbk(msg);
    return true;
  }

  if (topic == img_topic)
  {
  if (m.getDataType() == "sensor_msgs/Image")
    {
      auto msg = m.instantiate<sensor_msgs::Image>();
      if (!msg)
        return false;
      mapper.img_cbk(msg);
      return true;
    }
    if (m.getDataType() == "sensor_msgs/CompressedImage")
    {
      auto cmsg = m.instantiate<sensor_msgs::CompressedImage>();
      if (!cmsg)
        return false;
      try
      {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(cmsg, sensor_msgs::image_encodings::BGR8);
        sensor_msgs::ImagePtr img_msg = cv_ptr->toImageMsg();
        img_msg->header = cmsg->header;
        mapper.img_cbk(img_msg);
        return true;
      }
      catch (const cv_bridge::Exception &e)
      {
        ROS_WARN("cv_bridge CompressedImage failed: %s", e.what());
        return false;
      }
    }
    return false;
  }

  if (topic == lid_topic)
  {
    if (lidar_type == AVIA && m.getDataType() == "livox_ros_driver/CustomMsg")
    {
      auto msg = m.instantiate<livox_ros_driver::CustomMsg>();
      if (!msg)
        return false;
      mapper.livox_pcl_cbk(msg);
      return true;
    }
    if (m.getDataType() == "sensor_msgs/PointCloud2")
    {
      auto msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (!msg)
        return false;
      mapper.standard_pcl_cbk(msg);
      return true;
    }
    return false;
  }

  return false;
}

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fastlivo_bag", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  bool quiet = true;
  pnh.param("quiet", quiet, true);
  if (quiet)
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

  std::string bag_path;
  pnh.param<std::string>("bag", bag_path, "");
  if (bag_path.empty() && argc > 1)
    bag_path = argv[1];
  if (bag_path.empty())
  {
    ROS_ERROR("No bag file. Set ~bag or pass as argv[1]. Example: rosrun fast_livo fastlivo_bag _bag:=/data/CBD_Building_01.bag");
    return 1;
  }

  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh);
  mapper.initValueLog(argc, argv);
  mapper.initializeSubscribersAndPublishers(nh, it);

  const int lidar_type = mapper.p_pre->lidar_type;
  const std::string &lid_topic = mapper.lid_topic;
  const std::string &imu_topic = mapper.imu_topic;
  const std::string &img_topic = mapper.img_topic;

  ROS_WARN("FAST-LIVO2 bag mode: %s", bag_path.c_str());
  ROS_WARN("Topics: lidar=%s imu=%s image=%s", lid_topic.c_str(), imu_topic.c_str(), img_topic.c_str());
  if (mapper.value_log_en)
    ROS_WARN("value_log dir: %s", mapper.value_log_dir.c_str());

  rosbag::Bag bag;
  try
  {
    bag.open(bag_path, rosbag::bagmode::Read);
  }
  catch (const rosbag::BagException &e)
  {
    ROS_ERROR("Failed to open bag: %s", e.what());
    return 1;
  }

  std::vector<std::string> topics = {lid_topic, imu_topic, img_topic};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  size_t fed = 0;
  size_t processed = 0;
  for (const rosbag::MessageInstance &m : view)
  {
    if (!dispatchMessage(mapper, m, lidar_type, lid_topic, imu_topic, img_topic))
      continue;
    ++fed;

    while (mapper.runOnce())
      ++processed;
  }
  bag.close();

  while (mapper.runOnce())
    ++processed;

  mapper.savePCD();
  mapper.shutdownValueLog();

  ROS_WARN("Bag playback finished. fed_messages=%zu mapping_steps=%zu", fed, processed);
  if (mapper.value_log_en)
    ROS_WARN("[value_log] saved under: %s", mapper.value_log_dir.c_str());
  return 0;
}
