#pragma once

#include <dv_ros_capture/SynchronizeCameraRequest.h>
#include <dv_ros_capture/SynchronizeCameraResponse.h>
#define SynchronizeCameraRequest  SynchronizeCameraRequest_<boost::container::allocator<void>>
#define SynchronizeCameraResponse SynchronizeCameraResponse_<boost::container::allocator<void>>
#include <dv_ros_capture/SynchronizeCamera.h>
#undef SynchronizeCameraRequest
#undef SynchronizeCameraResponse
