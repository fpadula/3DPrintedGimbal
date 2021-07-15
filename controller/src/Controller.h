#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define INFO_REPORT 'r'
#define MOVE_START 's'
#define TASK_OK 'o'
#define TASK_ERROR 'e'
#define MOVE_REQUEST 't'
#define SPEED_REQUEST 'p'
#define INTERPOLATIONS_REQUEST 'i'
#define BUFFER_SIZE 13

class Controller{
    private:
        int serial_port, buffer_size;
        struct termios tty;
        char read_b[BUFFER_SIZE], write_b[BUFFER_SIZE];
        bool verbose;

        bool readXBytes(unsigned int, void*);
        void set_tty();
        inline void buffer_to_array(char *, float *, int);
        inline void array_to_buffer(char *, float *, int);
    public:
        Controller(const char *);
        ~Controller();
        int set_angles(float [3]);
        int set_angles(float [3], float [3]);
        int set_joints_max_speeds(float [3]);
        int set_joints_i_funcs(float [3]);
};

#endif