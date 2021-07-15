#include "Controller.h"

Controller::Controller(const char *port){
    this->serial_port = open(port, O_RDWR);
    // Check for errors
    if (this->serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }
    this->set_tty();
    this->verbose = true;
}

Controller::~Controller(){
    close(this->serial_port);
}

bool Controller::readXBytes(unsigned int x, void* buffer){
    unsigned int bytesRead = 0;
    int result;
    while (bytesRead < x){
        result = read(this->serial_port, buffer + bytesRead, x - bytesRead);
        if (result < 1 ){
            return false;
        }
        bytesRead += result;
    }
    return true;
}



void Controller::set_tty(){
    // Reference material:
    // https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    if(tcgetattr(this->serial_port, &(this->tty)) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    this->tty.c_cflag &= ~PARENB;// Clear parity bit, disabling parity
    this->tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
    this->tty.c_cflag &= ~CSIZE; // Clear all the size bits
    this->tty.c_cflag |= CS8; // 8 bits per byte (most common)
    this->tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    this->tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    this->tty.c_lflag &= ~ICANON; // Disable canonical mode (input is processed when a new line character is received)
    this->tty.c_lflag &= ~ECHO; // Disable echo
    this->tty.c_lflag &= ~ECHOE; // Disable erasure
    this->tty.c_lflag &= ~ECHONL; // Disable new-line echo
    this->tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    this->tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    this->tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    this->tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    this->tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    this->tty.c_cc[VTIME] = 100;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    this->tty.c_cc[VMIN] = 0;
    cfsetispeed(&(this->tty), B57600);
    cfsetospeed(&(this->tty), B57600);

    // Save tty settings, also checking for error
    if (tcsetattr(this->serial_port, TCSANOW, &(this->tty)) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

inline void Controller::buffer_to_array(char *buffer, float *array, int array_size){
    for(int i = 0; i < array_size; i++)
        memcpy(&array[i], buffer + sizeof(float)*i, sizeof(float));
}

inline void Controller::array_to_buffer(char *buffer, float *array, int array_size){
    for(int i = 0; i < array_size; i++)
        memcpy(buffer + sizeof(float)*i, &array[i], sizeof(float));
}

int Controller::set_angles(float target_angles[3]){
    float c_j_angles[3];
    this->set_angles(target_angles, c_j_angles);
}

int Controller::set_angles(float target_angles[3], float current_angles[3]){
    char msg_type;
    // float c_j_angles[3];

    this->write_b[0] = MOVE_REQUEST;
    this->array_to_buffer(&(this->write_b[1]), target_angles, 3);                    
    // Sending move request
    write(serial_port, this->write_b, BUFFER_SIZE);
    // Waiting for the start message:
    readXBytes(BUFFER_SIZE, &(this->read_b));     
    memcpy(&msg_type, &read_b[0], sizeof(char));            
    if(msg_type != MOVE_START){
        if(this->verbose)
            printf("Board did not send start message (received '%c'). Aborting...\n", msg_type);
        return 1;
    }
    if(this->verbose)
        printf("Moving gimbal to position...\n");
    while(msg_type != TASK_OK){
        readXBytes(BUFFER_SIZE, &(this->read_b));
        memcpy(&msg_type, &(this->read_b[0]), sizeof(char));        
        if(msg_type == INFO_REPORT){                        
            buffer_to_array(&read_b[1], current_angles, 3);
            if(this->verbose)
                printf("\tCurrent joint positions: %.2f, %.2f, %.2f\n", current_angles[0], current_angles[1], current_angles[2]);
        }
    }    
    return 0;
}

int Controller::set_joints_max_speeds(float joint_max_speeds[3]){
    char msg_type;

    this->write_b[0] = SPEED_REQUEST;
    array_to_buffer(&(this->write_b)[1], joint_max_speeds, 3);                    
    // Sending move request
    // printf("%c\n", write_buff[0]);
    write(serial_port, this->write_b, BUFFER_SIZE);
    // Waiting for the ok message:
    readXBytes(BUFFER_SIZE, &(this->read_b));
    // cmd_from_buffer(&msg_type, read_buf);  
    memcpy(&msg_type, &(this->read_b[0]), sizeof(char));            
    if(msg_type != TASK_OK){
        return 1;
    }
    return 0;
}

int Controller::set_joints_i_funcs(float int_fs[3]){
    char msg_type;

    this->write_b[0] = INTERPOLATIONS_REQUEST;
    array_to_buffer(&(this->write_b)[1], int_fs, 3);                    
    // Sending move request
    // printf("%c\n", write_buff[0]);
    write(serial_port, this->write_b, BUFFER_SIZE);
    // Waiting for the ok message:
    readXBytes(BUFFER_SIZE, &(this->read_b));
    // cmd_from_buffer(&msg_type, read_buf);  
    memcpy(&msg_type, &(this->read_b[0]), sizeof(char));            
    if(msg_type != TASK_OK){
        return 1;
    }
    return 0;
}