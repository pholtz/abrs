#include "message_queue.h"

int mq_init(key_t key)
{
	int msqid;
	//Try to initialize the Message Queue
    if ((msqid = msgget(key, IPC_CREAT | 0666 )) < 0)
    {
        perror("msgget");
        exit(1);
    }
    printf("Created and Attached to Message Queue with MQID: %d\n", msqid);
    return msqid;
}

int mq_attach(key_t key)
{
	int msqid;
	//Atempt to get the Message Queue ID using the common key
    if ((msqid = msgget(key, 0666)) < 0)
    {
        perror("msgget");
        exit(1);
    }
    printf("Attached to Message Queue with MQID: %d\n", msqid);
    return msqid;
}

void mq_send_message(int msqid, message_buf *sbuf)
{
    size_t buf_length = strlen(sbuf->mtext) + 1 ;
	
	/*
     * Send a message.
     */
    if (msgsnd(msqid, sbuf, buf_length, IPC_NOWAIT) < 0) {
		printf ("%d, %ld, %s, %ld\n", msqid, sbuf->mtype, sbuf->mtext, buf_length);
        perror("msgsnd");
        exit(1);
    }
    else 
		printf("Message: \"%s\" Sent\n", sbuf->mtext);
}

int mq_receive_message(int msqid, int message_type, message_buf *rbuf)
{
    /*
     * Receive an answer of desired message type.
     */
    if (msgrcv(msqid, rbuf, MSGSZ, message_type, IPC_NOWAIT) < 0) {
		if(errno != ENOMSG)
		{
			//The message was not received properly, exit with errors
			perror("msgrcv");
			exit(1);
		}
		printf("No messages of type %d received\n", message_type);
		return 0;
    }
    else
		printf("%s received\n", rbuf->mtext);
		return 1;
}
