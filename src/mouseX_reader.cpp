#include <mouseX_reader.h>

#include <ros/ros.h>
#include <unistd.h>
#include <fcntl.h>

MouseMove getMouseMove(int mouseToCapture)
{
    unsigned char buffer[3];
    int n_read = -1;
    MouseMove result;

    n_read = read(mouseToCapture, buffer, 3);

    if (n_read == -1) {
        ROS_WARN("Error occured when tring to capture mouse movement\n");
    }

    bool x_negative = buffer[0] >> 4 & 1;
    bool y_negative = buffer[0] >> 5 & 1;

    result.x = buffer[1] - (x_negative ? 256 : 0);
    result.y = buffer[2] - (y_negative ? 256 : 0);

    return result;
}
