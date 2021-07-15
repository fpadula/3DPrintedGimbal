#include <Arduino.h>
#include <Joint.h>
#include <Interpolations.h>

#include "tones.h"

#define INFO_REPORT 'r'
#define MOVE_START 's'
#define TASK_OK 'o'
#define TASK_ERROR 'e'
#define MOVE_REQUEST 't'
#define SPEED_REQUEST 'p'
#define INTERPOLATIONS_REQUEST 'i'

Joint joints[] = {
    Joint(5, -90.0f, 90.0f),
    Joint(3, -70.0f, 70.0f),
    Joint(4, -73.0f, 73.0f)
};

float (*int_fs[])(float, float, float) = {
    Interpolations::linear,
    Interpolations::expo,
    Interpolations::bounce_out,
    Interpolations::inout_cubic
};

const byte numBytes = 13; //1 byte for info + 4 bytes (32 bits) for each joint angle = 13 bytes total
byte receivedBytes[numBytes];
byte send_buffer[numBytes];
byte numReceived = 0;
boolean newData = false;
float f_array[3], joint_speeds[3];
unsigned long last_report, report_period = 500;

int melody[] = {
    NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};
int noteDurations[] = {
    4, 8, 8, 4, 4, 4, 4, 4
};

void play_melody(){
      // iterate over the notes of the melody:
    for (int thisNote = 0; thisNote < 8; thisNote++) {
        // to calculate the note duration, take one second divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / noteDurations[thisNote];
        tone(7, melody[thisNote], noteDuration);

        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        // stop the tone playing:
        noTone(8);
    }
}

inline void buffer_to_array(unsigned char *buffer, float *array, int array_size){
    for(int i = 0; i < array_size; i++)
        memcpy(&array[i], buffer + sizeof(float)*i, sizeof(float));
}

inline void array_to_buffer(unsigned char *buffer, float *array, int array_size){
    for(int i = 0; i < array_size; i++)
        memcpy(buffer + sizeof(float)*i, &array[i], sizeof(float));
}

bool readXBytes(size_t x, uint8_t* buffer){
    size_t bytesRead = 0;
    int result;
    while (bytesRead < x){
        result = Serial.readBytes(buffer + bytesRead, x - bytesRead);
        if (result < 1 ){
            return false;
        }
        bytesRead += result;
    }
    return true;
}

void setup() {
    Serial.begin(57600);
    pinMode(LED_BUILTIN, OUTPUT);
    // Serial.begin(115200);
    for(int i = 0; i < 3;i++){
        joints[i].init();
        joints[i].set_position(0);
    }
    // delay(1000);
    memcpy(send_buffer, '\0', numBytes);
    last_report = millis();
    // play_melody();
}

void loop() {
    bool all_reached;
    int i;
    char msg_type;
    float (*int_f)(float, float, float);

    if(Serial.available() > 0) {
        readXBytes(numBytes, receivedBytes);
        // Reading command
        memcpy(&msg_type, &receivedBytes[0], sizeof(char));
        switch (msg_type){
            case MOVE_REQUEST:
                // Reading target angles
                buffer_to_array(&receivedBytes[1], f_array, 3);
                for(i = 0; i < 3;i++)
                    joints[i].set_interpolation_target(f_array[i]);
                all_reached = false;
                send_buffer[0] = MOVE_START;
                Serial.write(send_buffer, numBytes);
                while(!all_reached){
                    all_reached = true;
                    for(i = 0; i < 3;i++)
                        all_reached &= joints[i].interpolation_step();
                    // Checking if we need to report current joint angles
                    if((all_reached) || (millis() - last_report >= report_period)){
                        last_report = millis();
                        for(i = 0; i < 3;i++)
                            f_array[i] = joints[i].get_position();
                        send_buffer[0] = INFO_REPORT;
                        array_to_buffer(&send_buffer[1], f_array, 3);
                        Serial.write(send_buffer, numBytes);
                    }
                    delay(1);
                }                
                // We do not detect errors right now, so every end is a success
                send_buffer[0] = TASK_OK;
                // Reporting task finish to controller
                Serial.write(send_buffer, numBytes);
                break;
            case SPEED_REQUEST:
                // Reading target velocities
                buffer_to_array(&receivedBytes[1], f_array, 3);
                for(i = 0; i < 3;i++)
                    joints[i].set_speed(f_array[i]);
                send_buffer[0] = TASK_OK;
                // Reporting task finish to controller
                Serial.write(send_buffer, numBytes);
                break;
            case INTERPOLATIONS_REQUEST:
                // Reading target interpolations
                buffer_to_array(&receivedBytes[1], f_array, 3);
                for(i = 0; i < 3;i++){
                    int_f = int_fs[(int)f_array[i]];
                    joints[i].set_interpolation_function(int_f);
                }
                send_buffer[0] = TASK_OK;
                // Reporting task finish to controller
                Serial.write(send_buffer, numBytes);
                break;
            default:
                break;
        }
    }
}