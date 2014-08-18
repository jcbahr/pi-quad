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

#include <errno.h>

#define PORT 12547
#define TRUE 1
#define FALSE 0
#define MESG_SIZE 15


typedef struct JoystickData
{
    int axis;
    int value;
} JoystickData;



int allocateMem()
{
    int shmid;
    key_t key;

    key = ftok("/home/jackson/quadcopter/.ipc/ipc_file", 'R');
    if (key == -1)
    {
        perror("ftok");
    }
    shmid = shmget(key, sizeof(JoystickData), IPC_CREAT|IPC_EXCL|0600);
    if (shmid == -1)
    {
        printf("Shared memory seg exists - opening as client\n");
        shmid = shmget(key, sizeof(JoystickData), 0);
        if (shmid == -1)
        {
            perror("shmget");
            return;
        }
    }

    return shmid;
}

JoystickData * attachMem(int id)
{
    JoystickData * jPtr;

    if ((jPtr = (JoystickData *)shmat(id, NULL, 0)) == (JoystickData *)-1)
    {
        perror("shmat");
    }
    return jPtr;
}

void detachMem(JoystickData * jPtr)
{
    if (shmdt(jPtr) == -1)
    {
        perror("shmdt");
    }
}

void deallocateMem(int id)
{
    if (shmctl(id, IPC_RMID, NULL) == -1)
    {
        perror("shmctl");
    }
}



int main(int argc, char**argv)
{
    int sockfd, n;
    struct sockaddr_in servaddr, cliaddr;
    socklen_t len;
    char mesg[MESG_SIZE];

    JoystickData * joyPtr;
    int shmid;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
    bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

    shmid = allocateMem();
    joyPtr = attachMem(shmid);

    /*
    while(TRUE)
    {
        len = sizeof(cliaddr);
        n = recvfrom(sockfd, mesg, MESG_SIZE, 0, (struct sockaddr *) &cliaddr, &len);
        mesg[n] = '\0';
        printf("before: %d: %d\n", joyPtr->axis, joyPtr->value);
        sscanf(mesg, "%d: %d", &(joyPtr->axis), &(joyPtr->value));
        printf("%s\n", mesg);
        printf("after: %d: %d\n", joyPtr->axis, joyPtr->value);
    }
    */ 

    char sendline[1000];
    while (fgets(sendline, 10000, stdin) != NULL)
    {
        printf("%d: %d\n", joyPtr->axis, joyPtr->value);
        sscanf(sendline, "%d: %d", &(joyPtr->axis), &(joyPtr->value));
    }

    detachMem(joyPtr);
    deallocateMem(shmid);
}
