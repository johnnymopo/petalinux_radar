
#ifndef RADAR_IOCTL_H
#define RADAR_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
typedef unsigned int u32;
#endif

#define MOWER_MAGIC 'm'

struct radar_params
{
    char use_external_pps;
    char enable_trigger;
    char reset;
    char leds;
    u32 time;
    u32 trigger_dwell;
    u32 sys_clock_frequency;
};

#define RADAR_SET_REGS     _IOW(MOWER_MAGIC, 0, struct radar_params)
#define RADAR_GET_REGS     _IOR(MOWER_MAGIC, 1, struct radar_params)
#define RADAR_SET_DEFAULTS _IO(MOWER_MAGIC, 2)

#define RADAR_DEFAULT_FREQUENCY           100000000
#define RADAR_DEFAULT_TRIGGER_DWELL       1<<31
#define RADAR_DEFAULT_EXTERNAL_PPS        0
#define RADAR_DEFAULT_TRIGGER_ENABLE      0
#define RADAR_DEFAULT_LEDS                1<<2

#define RADAR_RST_BIT                     0
#define RADAR_WRITE_REG_BIT               1
#define RADAR_PPS_EXTERN_BIT              2
#define RADAR_TRIGGER_ENABLE_BIT          3
#define RADAR_LED_BITS                    29

#define RADAR_CONTROL_OFFSET              0x00000000
#define RADAR_EPOCH_OFFSET                0x00000004
#define RADAR_TRIGGER_OFFSET              0x00000008
#define RADAR_CLOCK_OFFSET                0x0000000C

#endif

