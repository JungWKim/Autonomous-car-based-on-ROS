/*
 * =====================================================================================
 *
 *       Filename:  queue.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2020년 10월 13일 22시 47분 10초
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/msg.h>
#include <signal.h>
#include <iostream>
#include <unistd.h>

using namespace std;

struct message
{
    long msg_type;
    int data;
};

