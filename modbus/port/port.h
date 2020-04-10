/*! 
 * \file        port.h
 * \brief        
 *
 * \version     1.0
 * \date        2019-10-08
 * \author      Sage Xiong
 * \email       sage.xiong@foxmail.com
 *
 */
#ifndef _PORT_H
#define _PORT_H

/* ----------------------- Platform includes --------------------------------*/
#include "platform.h"

/* ----------------------- Defines ------------------------------------------*/
#define ENTER_CRITICAL_SECTION( )
#define EXIT_CRITICAL_SECTION( )

#define assert( x )

typedef bool    BOOL;

typedef unsigned char UCHAR;
typedef char    CHAR;

typedef unsigned short USHORT;
typedef short   SHORT;

typedef unsigned long ULONG;
typedef long    LONG;

#ifndef TRUE
#define TRUE            true
#endif

#ifndef FALSE
#define FALSE           false
#endif

#endif /* END OF _PORT_H */
