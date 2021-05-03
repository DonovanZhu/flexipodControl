#ifndef __HOST_H
#define __HOST_H


#define MOTOR_NUM				   	12			// Number of ESCs

#define USB_UART_SPEED     		1000000		// Baudrate of the teeensy USB serial link

#define HOST_ERROR_FD         	-1			// Non existant file descriptor
#define HOST_ERROR_DEV        	-2			// Non existant serial device
#define HOST_ERROR_WRITE_SER  	-3			// Write error on serial
#define HOST_ERROR_BAD_PK_SZ  	-4			// Bad incoming packet size error

//#define  HOST_MODEMDEVICE		"usb-Teensyduino_USB_Serial_8784100-if00"
#define HOST_MODEMDEVICE    	"/dev/ttyACM0"	// Name of USB port
#define HOST_DEV_SERIALNB		8784100			// Serial number of the teensy, check this number by using terminal
#define HOST_DEV_SERIALLG		10				// Max length of a serial number
#define HOST_SERIAL_DEV_DIR		"/dev/serial/by-id/"
#define HOST_BAUDRATE       	B1000000		// Serial baudrate
#define DRIVE_RATIO 			8.0 		// Drive ratio of motor gear box


// Teensy->host communication data structure
// sizeof(ESCPID_comm)=64 to match USB 1.0 buffer size
typedef struct {
  float    joint_pos[MOTOR_NUM];      // Motors rotation angle
  float    joint_vel[MOTOR_NUM];     // Motors rpm
  float	   torque[MOTOR_NUM];     // Motors torque
  float    acc[3];						  // Acceleration in X Y Z direction, m/s^2
  float    gyr[3];						  // Gyroscope in X Y Z direction, deg/s
  float    mag[3];             // Magnetometer in X Y Z, uT
  float    eular[3];
  float    timestamps;
} Teensycomm_struct_t;

// Host->teensy communication data structure
// sizeof(RPi_comm)=64 to match USB 1.0 buffer size
typedef struct {
  float    comd[MOTOR_NUM];        		// Desired Speed, rpm
} Jetson_comm_struct_t;

// Prototypes
char *Host_name_from_serial(uint32_t);
int   Host_get_fd(uint32_t);
int   Host_init_port(uint32_t);
void  Host_release_port(uint32_t);
int   Host_comm_update(uint32_t, Teensycomm_struct_t**);

#endif
