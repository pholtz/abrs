#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define MSGSZ     128

//Create the message structure to be passed
typedef struct msgbuf
{
         long    mtype;
         char    mtext[MSGSZ];
} message_buf;

int mq_init(key_t key);
int mq_attach(key_t key);
void mq_send_message(int msqid, message_buf *sbuf);
int mq_receive_message(int msqid, int message_type, message_buf *rbuf);
