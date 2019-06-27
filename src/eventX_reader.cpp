#include <eventX_reader.h>

#include <ros/ros.h>
#include <unistd.h>
#include <fcntl.h>

MouseMove getMouseMoveEvent(int mouseToCapture)
{
    input_event ie;
    int n_read = read(mouseToCapture, &ie, sizeof(ie));

    ROS_DEBUG("Event t=%d c=%d v=%d", ie.type, ie.code, ie.value);

    MouseMove deltaMove;

    switch (ie.type) {
        case EV_SYN:
            break;

        case EV_REL:
            switch (ie.code) {
                case REL_X:
                    deltaMove.y = ie.value;
                    break;

                case REL_Y:
                    deltaMove.x = ie.value;
                    break;

                case REL_WHEEL:
                    ROS_INFO("Wheel %d", ie.value);
                    break;
            }
            break;
    }

    return deltaMove;
}