/**************************************************************
* Author: Snehasish Kar
* date: 15th Mar 2015
* Version: 1.0.00
* Description:
* Below code is used to handle printf(), fprintf()
* Input for the same is the message and the error_lvl
**************************************************************/

#ifndef INCLUDE_VIPL_PRINTF_HPP_
#define INCLUDE_VIPL_PRINTF_HPP_

#include <stdint.h>
extern int32_t error_lvl;
void vipl_printf(char message[],int error_lvl,char file[],int line);

#endif /* INCLUDE_VIPL_PRINTF_HPP_ */
