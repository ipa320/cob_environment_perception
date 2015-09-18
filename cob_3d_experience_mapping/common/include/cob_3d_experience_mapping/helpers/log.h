#pragma once

//#define NDEBUG

#ifdef NDEBUG

#define DBG_PRINTF(str, ...) ;
#define ERROR_PRINTF(str, ...) ROS_ERROR(str, ##__VA_ARGS__)

#define DBG_PRINTF_URGENT(str, ...) ;
//#define DBG_PRINTF_URGENT(str, ...) {printf(str, ##__VA_ARGS__);fflush(stdout);}

#else

#define DBG_PRINTF(str, ...) {printf(str, ##__VA_ARGS__);fflush(stdout);}
#define ERROR_PRINTF(str, ...) ROS_ERROR(str, ##__VA_ARGS__)
#define DBG_PRINTF_URGENT(str, ...) {printf(str, ##__VA_ARGS__);fflush(stdout);}

#endif
