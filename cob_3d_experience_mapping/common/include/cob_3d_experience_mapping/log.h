#pragma once

#ifdef NDEBUG

#define DBG_PRINTF(str, ...) ;
#define ERROR_PRINTF(str, ...) ROS_ERROR(str, ##__VA_ARGS__)

#else

#define DBG_PRINTF(str, ...) printf(str, ##__VA_ARGS__)
#define ERROR_PRINTF(str, ...) ROS_ERROR(str, ##__VA_ARGS__)

#endif
