#include <stdio.h>
#include <stdlib.h>

#include "Controller.h" // write(), read(), close()

int main(){    
    char cmd;
    float f_array[3];
    bool run, verbose;
    Controller c = Controller("/dev/ttyUSB0");

    // Sleeping for 3s, waiting arduino to get ready
    usleep(3 * 1000000);

    run = true;
    verbose = true;    
    while(run){
        printf(">");
        scanf(" %c", &cmd);
        switch (cmd){
            case 't':
                // Reading target angles
                scanf("%f %f %f", &f_array[0], &f_array[1], &f_array[2]);
                if(verbose)
                    printf("Sending move request to gimbal. Angles: %.2f, %.2f, %.2f\n", f_array[0], f_array[1], f_array[2]);
                c.set_angles(f_array);
                if(verbose)
                    printf("Done!\n");
                break;
            case 's':
                scanf("%f %f %f", &f_array[0], &f_array[1], &f_array[2]);
                if(verbose)
                    printf("Setting joints max speeds to: %.2f, %.2f, %.2f\n", f_array[0], f_array[1], f_array[2]);
                c.set_joints_max_speeds(f_array);
                if(verbose)
                    printf("Done!\n");
                break;
            case 'i':
                scanf("%f %f %f", &f_array[0], &f_array[1], &f_array[2]);                
                if(verbose)
                    printf("Setting joints interpolation functions to: %.0f, %.0f, %.0f\n", f_array[0], f_array[1], f_array[2]);
                c.set_joints_i_funcs(f_array);
                if(verbose)
                    printf("Done!\n");
                break;
            case 'h':
                printf("h\t\t Display this help message\n");
                printf("t A1 A2 A3\t Move gimbal joints to position (A1, A2, A3). Angles are between -90 and 90 \n");
                printf("s S1 S2 S3\t Set gimbal joint's speed to (S1, S2, S3). Speeds are in degrees/s\n");
                printf("v\t\t Toggle verbose (default on)\n");
                printf("q\t\t Exit program\n");
                break;
            case 'v':
                verbose = !verbose;
                printf("Verbose mode: %s\n", verbose? "on": "off");
                break;
            case 'q':
                run = false;
                if(verbose)
                    printf("Exiting...\n");
                break;
            default:
                if(cmd != '\n')
                    printf("Command %c not recognized.\n", cmd);
                break;
        }
    }
    return 0;
}