#include <stdio.h>
#define TRUE 1
#define FALSE 0

/* joystick */
#include <fcntl.h>
#include <linux/joystick.h>

/* UDP */
#include <sys/socket.h>
#include <netinet/in.h>
#include <strings.h>
#include <string.h>

#define PORT 12547
#define MESG_SIZE 15
#define IP_ADDR "192.168.1.17"


/*
 * joystick code came from:
 * https://github.com/abedra/geekfest-linux-kernel-joystick-api/\
 * blob/master/fun-with-the-linux-kernel.org
 *
 * UDP client code taken from:
 * http://www.cs.ucsb.edu/~almeroth/classes/W01.176B/hw2/examples/udp-client.c
 */

int open_joystick (char *device_name)
{
    int joystick_fd = -1;

    if (device_name == NULL)
    { return joystick_fd; }

    joystick_fd = open (device_name, O_RDONLY | O_NONBLOCK);

    if (joystick_fd < 0)
    {
        printf ("Could not locate joystick device %s\n", device_name);
        return joystick_fd;
    }

    return joystick_fd;
}

int main()
{
    int fd, result;
    struct js_event jse;

    int sockfd, n;
    struct sockaddr_in servaddr, cliaddr;
    char sendline[MESG_SIZE];
    char recvline[MESG_SIZE];

    fd = open_joystick ("/dev/input/js0");
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(IP_ADDR);
    servaddr.sin_port = htons(PORT);

    while (TRUE)
    {
        while (read (fd, &jse, sizeof (jse)) > 0)
        {
            printf ("Event: time %8u, value %8hd, type: %3u, axis/button: %u\n",
                    jse.time, jse.value, jse.type, jse.number);
            /*
             * sent format is "axis: value"
             */
            sprintf(sendline, "%u: %hd", jse.number, jse.value);
            sendto(sockfd, sendline, strlen(sendline), 0,
                    (struct sockaddr*)&servaddr, sizeof(servaddr));
            //n = recvfrom(sockfd, recvline, 10000, 0, NULL, NULL);
            //recvline[n] = 0;
            /* response: */
            //fputs(recvline, stdout);
        }
    }

    return 0;
}
