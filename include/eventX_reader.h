#ifndef _EVENTX_READER_H
#define _EVENTX_READER_H

#include <sys/time.h>
#include <mouse_move.h>

enum Event_types
{
    EV_SYN = 0,
    EV_REL = 2
};

enum Event_codes
{
    REL_X = 0,
    REL_Y = 1,
    REL_WHEEL = 8
};

struct input_event
{
    struct timeval time;
    unsigned short type;
    unsigned short code;
    unsigned int value;
};

MouseMove getMouseMoveEvent(int mouseToCapture);


#endif // _EVENTX_READER_H
