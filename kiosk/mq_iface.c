#include "message_queue.h"

int mq_id;
key_t mq_key;
message_buf mq_sbuf;
message_buf mq_rbuf;


int main(int argc, char *argv[])
{

    /*
     * Get the message queue id for the
     * "name" 1234, which was created by
     * the server.
     */
    mq_key = 1234;
	mq_id = mq_init(mq_key);
    //It is now safe to call send_message() and receive_message()
    
    //Send a Message of Type 1 into the Message Queue
    mq_sbuf.mtype = 1;
    strcpy(mq_sbuf.mtext, "Did you get this?");
	mq_send_message(mq_id, &mq_sbuf);
}
