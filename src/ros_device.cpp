#include <calibration_common/pcl/utils.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ct_calibration/ros_device.h>


namespace ct_calibration
{

void PinholeRGBDevice::imageCallback(const sensor_msgs::Image::ConstPtr & image_msg)
{
  last_messages_.image_msg = image_msg;
  setHasNewMessages(last_messages_.camera_info_msg.get() != 0);
}

void PinholeRGBDevice::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg)
{
  last_messages_.camera_info_msg = camera_info_msg;
  if (not isSensorSet())
  {
    sensor_ = boost::make_shared<calibration::PinholeSensor>();
    calibration::PinholeCameraModel::ConstPtr cm = boost::make_shared<calibration::PinholeCameraModel>(*camera_info_msg);
    sensor_->setFrameId(frameId());
    sensor_->setCameraModel(cm);
  }
  setHasNewMessages(last_messages_.image_msg.get() != 0);
}

void PinholeRGBDevice::createSubscribers(ros::NodeHandle & nh,
                                         image_transport::ImageTransport & image_transport_nh,
                                         const std::string & main_topic)
{
  std::stringstream ss;
  ss << main_topic << "/image";
  image_sub_ = image_transport_nh.subscribe(ss.str(), 1, &PinholeRGBDevice::imageCallback, this);

  ss.str("");
  ss << main_topic << "/camera_info";
  camera_info_sub_ = nh.subscribe(ss.str(), 1, &PinholeRGBDevice::cameraInfoCallback, this);
}

PinholeRGBDevice::Data::Ptr PinholeRGBDevice::convertMessages(const Messages & messages)
{
  assert(sensor_);
  cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(messages.image_msg, sensor_msgs::image_encodings::BGR8);

  Data::Ptr data = boost::make_shared<Data>();

  try
  {
    cv::Mat rectified;
    sensor_->cameraModel()->rectifyImage(image_ptr->image, rectified);
    data->image = rectified;
  }
  catch (const image_geometry::Exception & ex)
  {
    ROS_WARN_STREAM_ONCE(ex.what() << std::endl << "Message (from topic \"" << image_sub_.getTopic() << "\"): " << std::endl << sensor_->cameraModel()->cameraInfo());
    data->image = image_ptr->image;
  }

  return data;
}


void KinectDevice::imageCallback(const sensor_msgs::Image::ConstPtr & image_msg,
                                 const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg)
{
  last_messages_.image_msg = image_msg;
  last_messages_.camera_info_msg = camera_info_msg;
  if (not isColorSensorSet())
  {
    color_sensor_ = boost::make_shared<calibration::PinholeSensor>();
    calibration::PinholeCameraModel::ConstPtr cm = boost::make_shared<calibration::PinholeCameraModel>(*camera_info_msg);
    color_sensor_->setFrameId(colorFrameId());
    color_sensor_->setCameraModel(cm);
  }
  setHasNewMessages(last_messages_.cloud_msg.get() != 0);
}


//void KinectDevice::depthCallback(const sensor_msgs::Image::ConstPtr & image_msg,
//                                 const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg)
//{
//  last_messages_.depth_image_msg = image_msg;
//  last_messages_.depth_camera_info_msg = camera_info_msg;
//  if (not isDepthSensorSet())
//  {
//    depth_sensor_ = boost::make_shared<calibration::KinectDepthSensor<UndistortionModel> >();
//    calibration::KinectDepthCameraModel::ConstPtr cm = boost::make_shared<calibration::KinectDepthCameraModel>(*camera_info_msg);
//    depth_sensor_->setFrameId(depthFrameId());
//    depth_sensor_->setCameraModel(cm);
//  }
//}

void KinectDevice::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & cloud_msg)
{
  last_messages_.cloud_msg = cloud_msg;
  setHasNewMessages(last_messages_.camera_info_msg and last_messages_.image_msg);
}

void KinectDevice::createSubscribers(ros::NodeHandle & nh,
                                     image_transport::ImageTransport & image_transport_nh,
                                     const std::string & main_topic)
{
  std::stringstream ss;
  ss << main_topic << "/image";
  camera_sub_ = image_transport_nh.subscribeCamera(ss.str(), 1, &KinectDevice::imageCallback, this);

//  ss.str("");
//  ss << main_topic << "/depth_image";
//  depth_sub_ = image_transport_nh.subscribeCamera(ss.str(), 1, &KinectDevice::depthCallback, this);

  ss.str("");
  ss << main_topic << "/cloud";
  cloud_sub_ = nh.subscribe(ss.str(), 1, &KinectDevice::cloudCallback, this);
}

KinectDevice::Data::Ptr KinectDevice::convertMessages(const Messages & messages)
{
  assert(color_sensor_);
  cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(messages.image_msg, sensor_msgs::image_encodings::BGR8);

  Data::Ptr data = boost::make_shared<Data>();

  try
  {
    cv::Mat rectified;
    color_sensor_->cameraModel()->rectifyImage(image_ptr->image, rectified);
    data->image = rectified;
  }
  catch (const image_geometry::Exception & ex)
  {
    ROS_WARN_STREAM_ONCE(ex.what() << std::endl << "Message (from topic \"" << camera_sub_.getTopic() << "\"): " << std::endl << color_sensor_->cameraModel()->cameraInfo());
    data->image = image_ptr->image;
  }

  data->cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  pcl::fromROSMsg(*messages.cloud_msg, *data->cloud);
  return data;
}




} /* namespace ct_calibration */
