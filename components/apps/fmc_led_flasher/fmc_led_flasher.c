
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include "radar_ioctl.h"

int main(void)
{
    printf("reading from ioctl\n");

    int fid;
    unsigned int i;

    struct radar_params *params;

    params = (struct radar_params *) malloc(sizeof(struct radar_params));
    if (!params) {
	printf("malloc failed\n");
	return 1;
    }

    fid = open("/dev/radar_controller",O_RDWR);
    if (fid < 0) {
	printf("could not open /dev/radar_controller\n");
	free(params);
	return 1;
    }

    if (ioctl(fid, RADAR_GET_REGS, params) != 0) {
	printf("ioctl read failed\n");
	free(params);
	return 1;
    }

    printf("use_external_pps:    %d\n",(u32)params->use_external_pps);
    printf("enable_trigger:      %d\n",(u32)params->enable_trigger);
    printf("reset:               %d\n",(u32)params->reset);
    printf("leds:                %d\n",(u32)params->leds);
    printf("time:                %d\n",(u32)params->time);
    printf("trigger_dwell:       %d\n",(u32)params->trigger_dwell);
    printf("sys_clock_frequency: %d\n",(u32)params->sys_clock_frequency);

    printf("I'm going to flash fmc carrier leds 1,2,3 to 7, on the second\n");
    
    for (i=0;i<8;i++) {
	printf("writing %d\n",i);
	params->leds = (char)i;
	if (ioctl(fid, RADAR_SET_REGS, params) != 0) 
	    printf("ioctl write failed\n");
	sleep(1);
    }

    if (ioctl(fid, RADAR_SET_DEFAULTS) != 0) {
	printf("ioctl default failed\n");
	free(params);
	return 1;
    }

    close(fid);
    free(params);

    return 0;
}
