#pragma once

#include <dv_ros_capture/SetImuInfoRequest.h>
#include <dv_ros_capture/SetImuInfoResponse.h>
#define SetImuInfoRequest  SetImuInfoRequest_<boost::container::allocator<void>>
#define SetImuInfoResponse SetImuInfoResponse_<boost::container::allocator<void>>
#include <dv_ros_capture/SetImuInfo.h>
#undef SetImuInfoRequest
#undef SetImuInfoResponse
