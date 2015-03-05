#include "message_queue.h"

int mq_id;
key_t mq_key;
message_buf mq_rbuf;
message_buf mq_sbuf;


int main(int argc, char *argv[])
{

    /*
     * Get the message queue id for the
     * "name" 1234, which was created by
     * the server.
     */
    mq_key = 1234;
	mq_id = mq_attach(mq_key);
    //It is now safe to call send_message() and receive_message()
	
	//Check for a message of type 1 in the Message Queue
	mq_receive_message(mq_id, 1, &mq_rbuf);
}
