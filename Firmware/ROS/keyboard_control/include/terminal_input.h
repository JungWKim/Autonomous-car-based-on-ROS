#include <unistd.h>
#include <termios.h>

static struct termios initial_settings, new_settings;                                          
static int peek_character = -1;                                                                

void init_keyboard();                                                                          
void close_keyboard();                                                                         
int kbhit();                                                                                   
int readch();
