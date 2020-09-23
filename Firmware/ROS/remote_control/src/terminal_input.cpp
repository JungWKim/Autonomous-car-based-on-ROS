/*
 * =====================================================================================
 *
 *       Filename:  terminal_input.cpp
 *
 *    Description:  terminal attribute reset and initialize keyboard input
 *
 *        Version:  1.0
 *        Created:  2020년 09월 24일 01시 05분 18초
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#include "terminal_input.h"

void init_keyboard()
{

             tcgetattr(0, &initial_settings);

             new_settings = initial_settings;

             new_settings.c_lflag &= ~ICANON;

             new_settings.c_lflag &= ~ECHO;

             new_settings.c_lflag &= ~ISIG;

             new_settings.c_cc[VMIN] = 1;

             new_settings.c_cc[VTIME] = 0;

             tcsetattr(0, TCSANOW, &new_settings);

}

void close_keyboard()
{
             tcsetattr(0, TCSANOW, &initial_settings);
}

int kbhit()
{
             char ch;

             int nread;

             if (peek_character != -1) return 1;

             new_settings.c_cc[VMIN] = 0;

             tcsetattr(0, TCSANOW, &new_settings);

             nread = read(0, &ch, 1);

             new_settings.c_cc[VMIN] = 1;

             tcsetattr(0, TCSANOW, &new_settings);

             if (nread == 1)
             {
                          peek_character = ch;

                          return 1;

             }

             return 0;
}

int readch()
{
             char ch;

             if (peek_character != -1)
             {
                          ch = peek_character;

                          peek_character = -1;

                          return ch;
             }

             read(0, &ch, 1);

             return ch;
}

//reference site : https://m.blog.naver.com/PostView.nhn?blogId=tipsware&logNo=221009514492&proxyReferer=https:%2F%2Fwww.google.com%2F
//https://corsa.tistory.com/16
