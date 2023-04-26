#include <dv_ros_messaging/messaging.hpp>

#include <ros/ros.h>

#include <gtest/gtest.h>

TEST(DvRosMessagingTests, TimestampConversionTests) {
	int64_t someTime  = 1644507922'623652;
	ros::Time rosTime = dv_ros_msgs::toRosTime(someTime);
	EXPECT_EQ(rosTime.sec, 1644507922);
	EXPECT_EQ(rosTime.nsec, 623652000);
	int64_t dvTime = dv_ros_msgs::toDvTime(rosTime);
	EXPECT_EQ(dvTime, someTime);
}

TEST(DvRosMessagingTests, GrayImageConversionTests) {
	cv::Mat gray     = cv::Mat::ones(cv::Size(100, 100), CV_8UC1);
	int64_t someTime = 1644507922'623652;
	dv::Frame frame  = dv::Frame(someTime, gray);
	auto rosMsg      = dv_ros_msgs::frameToRosImageMessage(frame);
	EXPECT_EQ(rosMsg.header.stamp.sec, 1644507922);
	EXPECT_EQ(rosMsg.header.stamp.nsec, 623652000);
	EXPECT_EQ(rosMsg.width, 100);
	EXPECT_EQ(rosMsg.height, 100);
	EXPECT_EQ(std::string(rosMsg.encoding), sensor_msgs::image_encodings::MONO8);
	EXPECT_EQ(rosMsg.data.front(), 1);
}

TEST(DvRosMessagingTests, ColorImageConversionTests) {
	cv::Mat color    = cv::Mat::ones(cv::Size(100, 100), CV_8UC3);
	int64_t someTime = 1644507922'623652;
	dv::Frame frame  = dv::Frame(someTime, color);
	auto rosMsg      = dv_ros_msgs::frameToRosImageMessage(frame);
	EXPECT_EQ(rosMsg.header.stamp.sec, 1644507922);
	EXPECT_EQ(rosMsg.header.stamp.nsec, 623652000);
	EXPECT_EQ(rosMsg.width, 100);
	EXPECT_EQ(rosMsg.height, 100);
	EXPECT_EQ(std::string(rosMsg.encoding), sensor_msgs::image_encodings::BGR8);
	EXPECT_EQ(rosMsg.data.front(), 1);
}

TEST(DvRosMessagingTests, InvalidImageConversionTests) {
	cv::Mat floatingImage = cv::Mat::ones(cv::Size(100, 100), CV_32FC3);
	int64_t someTime      = 1644507922'623652;
	dv::Frame frame       = dv::Frame(someTime, floatingImage);
	EXPECT_ANY_THROW(dv_ros_msgs::frameToRosImageMessage(frame));
}

TEST(DvRosMessagingTests, EventStoreConversions) {
	dv::EventStore events;
	int64_t someTime = 1644507922'623652;
	events.emplace_back(someTime, 0, 0, true);
	events.emplace_back(someTime + 1000, 0, 0, true);
	events.emplace_back(someTime + 2000, 0, 0, true);
	events.emplace_back(someTime + 3000, 0, 0, true);

	const auto rosEvents = dv_ros_msgs::toRosEventsMessage(events, cv::Size(100, 100));

	dv::EventStore eventsBack = dv_ros_msgs::toEventStore(rosEvents);

	EXPECT_EQ(rosEvents.events.size(), events.size());
	EXPECT_EQ(eventsBack.size(), events.size());
	EXPECT_EQ(eventsBack.getLowestTime(), events.getLowestTime());
	EXPECT_EQ(eventsBack.getHighestTime(), events.getHighestTime());
	for (size_t i = 0; i < eventsBack.size(); i++) {
		EXPECT_EQ(eventsBack.at(i).timestamp(), events.at(i).timestamp());
		EXPECT_EQ(eventsBack.at(i).x(), events.at(i).x());
		EXPECT_EQ(eventsBack.at(i).y(), events.at(i).y());
		EXPECT_EQ(eventsBack.at(i).polarity(), events.at(i).polarity());
	}
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
