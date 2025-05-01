#pragma once

#if __has_include(<rclcpp/version.h>)
#include <rclcpp/version.h>

#if (RCLCPP_VERSION_MAJOR >= 5)
#define SNP_CALLBACK_GROUP_SUPPORTED
#endif

#if (RCLCPP_VERSION_MAJOR >= 28)
#define SNP_QOS_REQUIRED_IN_SERVICE
#endif

#endif
