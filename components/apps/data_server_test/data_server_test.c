
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <string.h>

#define PORT 10001
#define BUFLEN 64

void err(char *str) 
{
    perror(str);
    exit(1);
}

int main(int argc, char *argv[]) 
{
    struct sockaddr_in serv_addr, cli_addr;
    fd_set readfds;
    int sockfd;
    int i;
    socklen_t slen = sizeof(cli_addr);
    char buf[BUFLEN];
    int running;
    int cmpval;
    int nvals;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	err("could not open socket");
    
    bzero(&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
	err("could not bind");

    FD_ZERO(&readfds);
    FD_SET(sockfd, &readfds);
    
    while (select(sockfd+1, &readfds, NULL, NULL, NULL)) {
	recvfrom(sockfd, buf, BUFLEN, 0, (struct sockaddr *)&cli_addr, &slen);
	cmpval = strncmp(buf, "run", 3);
	if (cmpval == 0) {
	    nvals = atoi(buf+3);
	    printf("nvals %d\n", nvals);
	}

	printf("recieved packet from %s:%d\nData: %s\n\n",
	       inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buf);

	memset(&buf[0], 0, sizeof(buf));

    }

    close(sockfd);

    return 0;
}


