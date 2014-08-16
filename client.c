/* UDP client code taken from:
 * http://www.cs.ucsb.edu/~almeroth/classes/W01.176B/hw2/examples/udp-client.c
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>

#define PORT 12547
#define MESG_SIZE 1000

#define IP_ADDR "192.168.1.17"

int main(int argc, char**argv)
{
    int sockfd, n;
    struct sockaddr_in servaddr, cliaddr;
    char sendline[MESG_SIZE];
    char recvline[MESG_SIZE];

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(IP_ADDR);
    servaddr.sin_port = htons(PORT);

    while (fgets(sendline, 10000, stdin) != NULL)
    {
        sendto(sockfd, sendline, strlen(sendline), 0,
                (struct sockaddr *)&servaddr, sizeof(servaddr));
        n = recvfrom(sockfd, recvline, 10000, 0, NULL, NULL);
        recvline[n] = 0;
        fputs(recvline, stdout);
    }
}


