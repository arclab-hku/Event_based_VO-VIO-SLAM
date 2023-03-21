#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <utility.hpp>

// ------------------------------------------------------------------------- //

std::mutex         mtx1, mtx2;
std::vector<Event> event_buffer1, event_buffer2;
int                cam1_width, cam1_height, cam2_width, cam2_height;

// ------------------------------------------------------------------------- //

void Event1Callback(const EventArray::ConstPtr & message, const std::string & topic_name) {
  ROS_INFO_ONCE("%s", colorful_char::info("Receive event messages under topic name: " + topic_name).c_str());
  mtx1.lock();
  for (const Event & event : message->events) event_buffer1.emplace_back(event);
  mtx1.unlock();
  cam1_width  = message->width;
  cam1_height = message->height;
}

void Event2Callback(const EventArray::ConstPtr & message, const std::string & topic_name) {
  ROS_INFO_ONCE("%s", colorful_char::info("Receive event messages under topic name: " + topic_name).c_str());
  mtx2.lock();
  for (const Event & event : message->events) event_buffer2.emplace_back(event);
  mtx2.unlock();
  cam2_width  = message->width;
  cam2_height = message->height;
}

// ------------------------------------------------------------------------- //

int main(int argc, char ** argv) {
  ros::init(argc, argv, "event_visualizer");
  ros::NodeHandle nh;

  // Read and validate the input parameters from config.
  std::vector<std::string> in_topics, out_topics;
  int                      out_freq;
  ros::param::get("in_event_stream_topic", in_topics);
  ros::param::get("out_event_frame_topic", out_topics);
  ros::param::get("out_frequency", out_freq);
  if (in_topics.size() != out_topics.size()) {
    ROS_ERROR("%s", colorful_char::error("The in and out topic number do not match with each other.").c_str());
    ros::shutdown();
    return -1;
  } else if (in_topics.size() > 2) {
    ROS_ERROR("%s", colorful_char::error("The current version does not support more than two event cameras.").c_str());
    ros::shutdown();
    return -1;
  }
  int cam_num = in_topics.size();

  // Set up the subscriber and publisher according to the sensor setup (monocular or stereo).
  ros::Subscriber sub1, sub2;
  ros::Publisher  pub1, pub2;
  for (size_t idx = 0; idx < cam_num; ++idx) {
    if (idx == 0) {
      sub1 = nh.subscribe<EventArray>(in_topics[idx], 100000, boost::bind(Event1Callback, _1, in_topics[idx]));
      pub1 = nh.advertise<Image>(out_topics[idx], out_freq);
    } else if (idx == 1) {
      sub2 = nh.subscribe<EventArray>(in_topics[idx], 100000, boost::bind(Event2Callback, _1, in_topics[idx]));
      pub2 = nh.advertise<Image>(out_topics[idx], out_freq);
    }
  }

  ros::Rate rate(out_freq);
  while (ros::ok()) {
    // Swap the events from the buffer to a temporary holder.
    std::vector<Event> event_holder1, event_holder2;
    mtx1.lock();
    event_holder1.swap(event_buffer1);
    mtx1.unlock();
    if (cam_num == 2) {
      mtx2.lock();
      event_holder2.swap(event_buffer2);
      mtx2.unlock();
    }
    if ((cam_num == 1 && event_holder1.size() == 0) || (cam_num == 2 && (event_holder1.size() == 0 || event_holder2.size() == 0))) {
      rate.sleep();
      ros::spinOnce();
      continue;
    }

    // Accumulate events onto the image canvas. Note that, only the last event will be displayed on the canvas with its polarity.
    cv_bridge::CvImage event_frame1, event_frame2;
    // event_frame1.image = cv::Mat(cam1_height, cam1_width, CV_8UC3, cv::Vec3b(255, 255, 255));
    event_frame1.image = cv::Mat::zeros(cam1_height, cam1_width, CV_8UC3);
    for (auto & event : event_holder1) {
      if (event.polarity) event_frame1.image.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0, 0);  // positive - blue
      else
        event_frame1.image.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 0, 255);  // negative - red
    }
    if (cam_num == 2) {
      event_frame2.image = cv::Mat(cam2_height, cam2_width, CV_8UC3, cv::Vec3b(255, 255, 255));
      for (auto & event : event_holder2) {
        if (event.polarity) event_frame2.image.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0, 0);
        else
          event_frame2.image.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 0, 255);
      }
    }

    // Publish the accumulated event frame as image.
    event_frame1.encoding     = "bgr8";
    event_frame1.header.stamp = ros::Time::now();
    pub1.publish(event_frame1.toImageMsg());
    if (cam_num == 2) {
      event_frame2.encoding     = "bgr8";
      event_frame2.header.stamp = event_frame1.header.stamp;
      pub2.publish(event_frame2.toImageMsg());
    }

    rate.sleep();
    ros::spinOnce();
  }
  ros::shutdown();
  return 0;
}
