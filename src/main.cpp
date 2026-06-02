#include "LIVMapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh);
  mapper.initValueLog(argc, argv);
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}