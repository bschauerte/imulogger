#include <stdio.h>
#include <math.h>

#include "ip_connection.h"
#include "brick_imu.h"

#define HOST "localhost"
#define PORT 4223
#define UID "aeoUQwwyAvY" // Change to your UID

float qx, qy, qz, qw;
float roll, pitch, yaw;

// Quaternion callback
void 
cb_quaternion(float x, float y, float z, float w) {
    // store the quaternion information
	qx = x; 
    qy = y; 
    qz = z; 
    qw = w;

    // convert and store the roll/pitch/yaw information
    roll  = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
    pitch = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
    yaw   = asin(2*x*y + 2*z*w);

    //printf("x: %f\ny: %f\nz: %f\nw: %f\n", x, y, z, w);
    printf("roll: %f, pitch: %f, yaw: %f (x: %f, y: %f, z: %f, w: %f)\n", roll, pitch, yaw, x, y, z, w);
}

int 
main(int argc, char* argv[]) 
{
    char *s_uid  = UID;
    int   s_port = PORT;
    char *s_host = HOST;

    // 
    if (argc > 1)
        s_uid = argv[1];

	// Create IP connection to brickd
	IPConnection ipcon;
	if(ipcon_create(&ipcon, s_host, s_port) < 0) {
		fprintf(stderr, "Could not create connection\n");
		exit(1);
	}

	// Create device object
	IMU imu;
	imu_create(&imu, s_uid); 

	// Add device to IP connection
	if(ipcon_add_device(&ipcon, &imu) < 0) {
		fprintf(stderr, "Could not connect to Brick\n");
		exit(1);
	}
	// Don't use device before it is added to a connection

	// Set period for quaternion callback to 1s
	imu_set_quaternion_period(&imu, 1000);

	// Register "quaternion callback" to cb_quaternion
	imu_register_callback(&imu, 
	                      IMU_CALLBACK_QUATERNION, 
	                      cb_quaternion);

	printf("Press key to exit\n");
	getchar();
	ipcon_destroy(&ipcon);
}
