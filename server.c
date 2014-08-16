/* UDP server code taken from:
 * http://www.cs.ucsb.edu/~almeroth/classes/W01.176B/hw2/examples/udp-server.c
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <strings.h>

#define PORT 12547
#define TRUE 1
#define FALSE 0
#define MESG_SIZE 15

int main(int argc, char**argv)
{
    int sockfd, n;
    struct sockaddr_in servaddr, cliaddr;
    socklen_t len;
    char mesg[MESG_SIZE];

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
    bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

    while(TRUE)
    {
        len = sizeof(cliaddr);
        n = recvfrom(sockfd, mesg, MESG_SIZE, 0, (struct sockaddr *) &cliaddr, &len);
        //sendto(sockfd, mesg, n, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
        mesg[n] = 0;
        printf("%s\n", mesg);
    }
}
