/* UDP server code taken from:
 * http://www.cs.ucsb.edu/~almeroth/classes/W01.176B/hw2/examples/udp-server.c
 */

/* UDP */
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <strings.h>

/* shared memory */
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define PORT 12547
#define TRUE 1
#define FALSE 0
#define MESG_SIZE 15


typedef struct JoystickData
{
    int axis;
    int value;
} JoystickData;



JoystickData * setupMem()
{
    int id;
    key_t key;
    JoystickData * jPtr;

    key = ftok("/home/jackson/.ipc/ipc_file", 'R');
    id = shmget (key, sizeof(JoystickData), IPC_CREAT | SHM_W | SHM_R);

    jPtr = shmat(id, NULL, 0);
}

void cleanupMem(JoystickData * jPtr)
{
    shmdt(jPtr);
}

int main(int argc, char**argv)
{
    int sockfd, n;
    struct sockaddr_in servaddr, cliaddr;
    socklen_t len;
    char mesg[MESG_SIZE];
    JoystickData jData;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
    bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

    jData = * setupMem();

    while(TRUE)
    {
        len = sizeof(cliaddr);
        n = recvfrom(sockfd, mesg, MESG_SIZE, 0, (struct sockaddr *) &cliaddr, &len);
        //sendto(sockfd, mesg, n, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
        mesg[n] = 0;
        sscanf(mesg, "%d: %d", jData.axis, jData.value);
        printf("%s\n", mesg);
        printf("%d: %d\n", jData.axis, jData.value);
    }
}
