
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <sys/ioctl.h>

#define MOWER_MAGIC 'm'
#define RADAR_START_DMA _IO(MOWER_MAGIC, 1)
#define RADAR_STOP_DMA _IO(MOWER_MAGIC, 2)

int main(int argc, char *argv[])
{
    int i,ii;
    int fid_dma;
    unsigned int *map;
    int size;
    int iters;
    int read_return;
    unsigned int filesize;
    unsigned int *data;

    struct pollfd pfds[1];

    if (argc != 3) {
	printf("usage: $radar_dma_tester <Nvals> <Niter>\n");
	return 1;
    }

    // byte size is 4x
    size = atoi(argv[1]);
    iters = atoi(argv[2]);

    if (!size || !iters) {
	printf("improper argument(s)\n");
	return 1;
    }

    printf("will read %d ints\n", size);
    
    filesize = (unsigned int)(size * sizeof(int));
    data = (unsigned int*)malloc(filesize);

    printf("filesize is %d bytes\n", filesize);

    fid_dma = open("/dev/radar_dma", O_RDWR | O_CREAT | O_TRUNC, (mode_t) 0600);

    printf("mapping kernel space to user\n");
    map = (unsigned int *)mmap(0, filesize, PROT_READ | PROT_WRITE, MAP_SHARED, fid_dma, 0);

    printf("spooling up dma\n");
    ioctl(fid_dma, RADAR_START_DMA, NULL);

    pfds[0].fd = fid_dma;
    pfds[0].events = POLLIN;

    for (i=0; i<iters; i++) {
	poll(pfds, 1, 1000);
	memcpy(data, map, filesize);
	for (ii=0; ii<size;ii++) 
	    printf("%d, ", data[ii]);
	printf("\n");
    }

    printf("stopping dma\n");
    ioctl(fid_dma, RADAR_STOP_DMA, NULL);

    munmap(map,filesize);
    close(fid_dma);
    free(data);
    return 0;
}


