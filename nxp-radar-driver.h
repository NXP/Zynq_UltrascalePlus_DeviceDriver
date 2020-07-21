/*
 *  nxp-radar-driver.h - Linux kernel modules for NXP driver to control the 
 *  custom PL block for radar devices 
 *   
 *
 *  Copyright (C) 2019 NXP. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * 
 */

#ifndef RADARDEV_H
#define RADARDEV_H

static int pid; // Stores application PID in user space

//#define SIG_DOLPHIN_MCUINT 44

#ifdef USE_IOCTL

/* * Based on checked ioctl-number.txt */
#define MAGIC_NUM 0x8A
/* * Set the message of the device driver */
#define IOCTL_SET_MSG _IOR(MAGIC_NUM, 0, char *)
/*
 * _IOR means that we're creating an ioctl command
 * number for passing information from a user process
 * to the kernel module.
 *
 * The first arguments, MAGIC_NUM, is the major device
 * number we're using.
 *
 * The second argument is the number of the command
 * (there could be several with different meanings).
 *
 * The third argument is the type we want to get from
 * the process to the kernel.
 */
/* * Get the message of the device driver */
#define IOCTL_GET_MSG _IOR(MAGIC_NUM, 1, char *)
/*
 * This IOCTL is used for output, to get the message
 * of the device driver. However, we still need the
 * buffer to place the message in to be input,
 * as it is allocated by the process.
 */
/* * Get the n'th byte of the message */
#define IOCTL_GET_NTH_BYTE _IOWR(MAGIC_NUM, 2, int)


#endif

#endif
