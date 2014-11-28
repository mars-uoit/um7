 /**
 *
 *  \file
 *  \brief   Copied directly from the UM6_config.h file, available online here:
 *           http://sourceforge.net/p/um6firmware/code/34/tree/trunk/UM6%20Firmware/UM6_config.h#l14
 *  \author  CH Robotics 
 */


// Define the firmware revision
#define	UM7_FIRMWARE_REVISION		(('U' << 24) | ('7' << 16) | ('1' << 8) | 'C') //CHECK THIS!!!!

// CONFIG_ARRAY_SIZE and DATA_ARRAY_SIZE specify the number of 32 bit configuration and data registers used by the firmware
// (Note: The term "register" is used loosely here.  These "registers" are not actually registers in the same sense of a 
// microcontroller register.  They are simply index locations into arrays stored in global memory.  Data and configuration
// parameters are stored in arrays because it allows a common communication protocol to be used to access all data and
// configuration.  The software communicating with the sensor needs only specify the register address, and the communication
// software running on the sensor knows exactly where to find it - it needn't know what the data is.  The software communicatin
// with the sensor, on the other hand, needs to know what it is asking for (naturally...)
// This setup makes it easy to make more data immediately available when needed - simply increase the array size, add code in
// the firmware that writes data to the new array location, and then make updates to the firmware definition on the PC side.
#define	CONFIG_ARRAY_SIZE	27	
#define	DATA_ARRAY_SIZE		52
#define	COMMAND_COUNT			10

// 
#define	CONFIG_REG_START_ADDRESS  0
#define	DATA_REG_START_ADDRESS    85
#define	COMMAND_REG_START_ADDRESS     170

// These preprocessor definitions make it easier to access specific configuration parameters in code
// They specify array locations associated with each register name.  Note that in the comments below, many of the values are
// said to be 32-bit IEEE floating point.  Obviously this isn't directly the case, since the arrays are actually 32-bit unsigned 
// integer arrays.  Bit for bit, the data does correspond to the correct floating point value.  Since you can't cast ints as floats,
// special conversion has to happen to copy the float data to and from the array.
// Starting with configuration register locations...
#define	UM7_CREG_COM_SETTINGS		      CONFIG_REG_START_ADDRESS				// General communications settings
#define	UM7_CREG_COM_RATES1 		      (CONFIG_REG_START_ADDRESS + 1)  // Broadcast rate settings
#define	UM7_CREG_COM_RATES2 		      (CONFIG_REG_START_ADDRESS + 2)  // Broadcast rate settings
#define	UM7_CREG_COM_RATES3 		      (CONFIG_REG_START_ADDRESS + 3)  // Broadcast rate settings
#define	UM7_CREG_COM_RATES4 		      (CONFIG_REG_START_ADDRESS + 4)  // Broadcast rate settings
#define	UM7_CREG_COM_RATES5 		      (CONFIG_REG_START_ADDRESS + 5)  // Broadcast rate settings
#define	UM7_CREG_COM_RATES6 		      (CONFIG_REG_START_ADDRESS + 6)  // Broadcast rate settings
#define	UM7_CREG_COM_RATES7 		      (CONFIG_REG_START_ADDRESS + 7)  // Broadcast rate settings
#define	UM7_MISC_SETTINGS			        (CONFIG_REG_START_ADDRESS + 8)  // Misc. settings
#define	UM7_CREG_HOME_NORTH		        (CONFIG_REG_START_ADDRESS + 9)	// GPS north position to consider 0
#define	UM7_CREG_HOME_EAST		        (CONFIG_REG_START_ADDRESS + 10)	// GPS east position to consider 0
#define	UM7_CREG_HOME_UP		          (CONFIG_REG_START_ADDRESS + 11)	// GPS altitute to consider 0
#define	UM7_CREG_GYRO_TRIM_X		      (CONFIG_REG_START_ADDRESS + 12) // Bias trim for x-axis rate gyro
#define	UM7_CREG_GYRO_TRIM_Y  	      (CONFIG_REG_START_ADDRESS + 13) // Bias trim for y-axis rate gyro
#define	UM7_CREG_GYRO_TRIM_Z		      (CONFIG_REG_START_ADDRESS + 14) // Bias trim for z-axis rate gyro	
#define	UM7_CREG_MAG_CAL_11	  			  (CONFIG_REG_START_ADDRESS + 15)	// Row 1, column 1 of magnetometer calibration matrix
#define	UM7_CREG_MAG_CAL_12		  		  (CONFIG_REG_START_ADDRESS + 16)	// Row 1, column 2 of magnetometer calibration matrix
#define	UM7_CREG_MAG_CAL_13				    (CONFIG_REG_START_ADDRESS + 17)	// Row 1, column 3 of magnetometer calibration matrix
#define	UM7_CREG_MAG_CAL_21			  	  (CONFIG_REG_START_ADDRESS + 18)	// Row 2, column 1 of magnetometer calibration matrix
#define	UM7_CREG_MAG_CAL_22			  	  (CONFIG_REG_START_ADDRESS + 19)	// Row 2, column 2 of magnetometer calibration matrix
#define	UM7_CREG_MAG_CAL_23			  	  (CONFIG_REG_START_ADDRESS + 20)	// Row 2, column 3 of magnetometer calibration matrix
#define	UM7_CREG_MAG_CAL_31			  	  (CONFIG_REG_START_ADDRESS + 21)	// Row 3, column 1 of magnetometer calibration matrix
#define	UM7_CREG_MAG_CAL_32				    (CONFIG_REG_START_ADDRESS + 22)	// Row 3, column 2 of magnetometer calibration matrix
#define	UM7_CREG_MAG_CAL_33				    (CONFIG_REG_START_ADDRESS + 23)	// Row 3, column 3 of magnetometer calibration matrix
#define UM7_CREG_MAG_BIAS_X           (CONFIG_REG_START_ADDRESS + 24) // Magnetometer x-axis bias
#define UM7_CREG_MAG_BIAS_Y           (CONFIG_REG_START_ADDRESS + 25) // Magnetometer y-axis bias
#define UM7_CREG_MAG_BIAS_Z           (CONFIG_REG_START_ADDRESS + 26) // Magnetometer z-axis bias

// Continuing with data register locations...
#define UM7_DREG_HEALTH               DATA_REG_START_ADDRESS          // Information about the health and status of the UM7
#define UM7_DREG_GYRO_RAW_XY          (DATA_REG_START_ADDRESS + 1)    // Raw x and y rate gyro data
#define UM7_DREG_GYRO_RAW_Z           (DATA_REG_START_ADDRESS + 2)    // Raw z rate gyro data
#define UM7_DREG_GYRO_TIME            (DATA_REG_START_ADDRESS + 3)    // Time at which the rate gyro data was acquired
#define UM7_DREG_ACCEL_RAW_XY         (DATA_REG_START_ADDRESS + 4)    // Raw x and y accelerometer data
#define UM7_DREG_ACCEL_RAW_Z          (DATA_REG_START_ADDRESS + 5)    // Raw z accelerometer data
#define UM7_DREG_ACCEL_TIME           (DATA_REG_START_ADDRESS + 6)    // Time at which the accelerometer data was acquired
#define UM7_DREG_MAG_RAW_XY           (DATA_REG_START_ADDRESS + 7)    // Raw x and y magnetometer data
#define UM7_DREG_MAG_RAW_Z            (DATA_REG_START_ADDRESS + 8)    // Raw z magnetometer data
#define UM7_DREG_MAG_RAW_TIME         (DATA_REG_START_ADDRESS + 9)    // Time at which the magnetometer data was acquired
#define UM7_DREG_TEMPERATURE          (DATA_REG_START_ADDRESS + 10)   // Temperature data
#define UM7_DREG_TEMPERATURE_TIME     (DATA_REG_START_ADDRESS + 11)   // Time at which the temperature data was acquired
#define UM7_DREG_GYRO_PROC_X          (DATA_REG_START_ADDRESS + 12)   // Processed x rate gyro data
#define UM7_DREG_GYRO_PROC_Y          (DATA_REG_START_ADDRESS + 13)   // Processed y rate gyro data
#define UM7_DREG_GYRO_PROC_Z          (DATA_REG_START_ADDRESS + 14)   // Processed z rate gyro data
#define UM7_DREG_GYRO_PROC_TIME       (DATA_REG_START_ADDRESS + 15)   // Time at which the rate gyro data was acquired
#define UM7_DREG_ACCEL_PROC_X         (DATA_REG_START_ADDRESS + 16)   // Processed x accelerometer data
#define UM7_DREG_ACCEL_PROC_Y          (DATA_REG_START_ADDRESS + 17)   // Processed y rate gyro data
#define UM7_DREG_ACCEL_PROC_Z         (DATA_REG_START_ADDRESS + 18)   // Processed z accelerometer data
#define UM7_DREG_ACCEL_PROC_TIME      (DATA_REG_START_ADDRESS + 19)   // Time at which the accelerometer data was acquired
#define UM7_DREG_MAG_PROC_X           (DATA_REG_START_ADDRESS + 20)   // Processed x magnetometer data
#define UM7_DREG_MAG_PROC_Y          (DATA_REG_START_ADDRESS + 21)   // Processed y rate gyro data
#define UM7_DREG_MAG_PROC_Z           (DATA_REG_START_ADDRESS + 22)   // Processed z magnetometer data
#define UM7_DREG_MAG_PROC_TIME        (DATA_REG_START_ADDRESS + 23)   // Time at which the magnetometer data was acquired
#define UM7_DREG_QUAT_AB              (DATA_REG_START_ADDRESS + 24)   // Quaternion elements A and B
#define UM7_DREG_QUAT_CD              (DATA_REG_START_ADDRESS + 25)   // Quaternion elements C and D
#define UM7_DREG_QUAT_TIME            (DATA_REG_START_ADDRESS + 26)   // Time at which the sensor was at the specified quaternion rotation
#define UM7_DREG_EULER_PHI_THETA      (DATA_REG_START_ADDRESS + 27)   // Roll and pitch angles
#define UM7_DREG_EULER_PSI            (DATA_REG_START_ADDRESS + 28)   // Yaw angle
#define UM7_DREG_EULER_PHI_THETA_DOT  (DATA_REG_START_ADDRESS + 29)   // Roll and pitch angle rates
#define UM7_DREG_EULER_PSI_DOT        (DATA_REG_START_ADDRESS + 30)   // Yaw rate
#define UM7_DREG_EULER_TIME           (DATA_REG_START_ADDRESS + 31)   // Time of computed Euler attitude and rates
#define UM7_DREG_POSITION_NORTH       (DATA_REG_START_ADDRESS + 32)   // North position in metres
#define UM7_DREG_POSITION_EAST        (DATA_REG_START_ADDRESS + 33)   // East position in metres
#define UM7_DREG_POSITION_UP          (DATA_REG_START_ADDRESS + 34)   // Altitude in metres
#define UM7_DREG_POSITION_TIME        (DATA_REG_START_ADDRESS + 35)   // Time of estimated position
#define UM7_DREG_VELOCITY_NORTH       (DATA_REG_START_ADDRESS + 36)   // North velocity
#define UM7_DREG_VELOCITY_EAST        (DATA_REG_START_ADDRESS + 37)   // East velocity
#define UM7_DREG_VELOCITY_UP          (DATA_REG_START_ADDRESS + 38)   // Altitude velocity
#define UM7_DREG_VELOCITY_TIME        (DATA_REG_START_ADDRESS + 39)   // Time of estimated velocity
#define UM7_DREG_GPS_LATITUDE         (DATA_REG_START_ADDRESS + 40)   // GPS latitude
#define UM7_DREG_GPS_LONGITUDE        (DATA_REG_START_ADDRESS + 41)   // GPS longitude
#define UM7_DREG_GPS_ALTITUDE         (DATA_REG_START_ADDRESS + 42)   // GPS altitude
#define UM7_DREG_GPS_COURSE           (DATA_REG_START_ADDRESS + 43)   // GPS course
#define UM7_DREG_GPS_SPEED            (DATA_REG_START_ADDRESS + 44)   // GPS speed
#define UM7_DREG_GPS_TIME             (DATA_REG_START_ADDRESS + 45)   // GPS time (UTC time of day in seconds)
#define UM7_DREG_GPS_SAT_1_2          (DATA_REG_START_ADDRESS + 46)   // GPS satellite information
#define UM7_DREG_GPS_SAT_3_4          (DATA_REG_START_ADDRESS + 47)   // GPS satellite information
#define UM7_DREG_GPS_SAT_5_6          (DATA_REG_START_ADDRESS + 48)   // GPS satellite information
#define UM7_DREG_GPS_SAT_7_8          (DATA_REG_START_ADDRESS + 49)   // GPS satellite information
#define UM7_DREG_GPS_SAT_9_10         (DATA_REG_START_ADDRESS + 50)   // GPS satellite information
#define UM7_DREG_GPS_SAT_11_12        (DATA_REG_START_ADDRESS + 51)   // GPS satellite information

// Finally the command registers
#define UM7_GET_FW_REVISION           COMMAND_REG_START_ADDRESS       // Responds with a packet containing the current firmware revision
#define UM7_FLASH_COMMIT              (COMMAND_REG_START_ADDRESS + 1) // Writes all current configuration settings to flash
#define UM7_RESET_TO_FACTORY          (COMMAND_REG_START_ADDRESS + 2) // Resets all settings to factory defaults
#define UM7_ZERO_GYROS                (COMMAND_REG_START_ADDRESS + 3) // Causes the rate gyro biases to be calibrated
#define UM7_SET_HOME_POSITION         (COMMAND_REG_START_ADDRESS + 4) // Sets the current GPS location as position (0,0)
#define UM7_RESERVED_1                (COMMAND_REG_START_ADDRESS + 5) // RESERVED
#define UM7_SET_MAG_REFERENCE         (COMMAND_REG_START_ADDRESS + 6) // Sets the magnetometer reference vector
#define UM7_RESERVED_2                (COMMAND_REG_START_ADDRESS + 7) // RESERVED
#define UM7_RESERVED_3                (COMMAND_REG_START_ADDRESS + 8) // RESERVED
#define UM7_RESET_EKF                 (COMMAND_REG_START_ADDRESS + 9) // Resets the EKF

// Definitions for specific registers
// UM7_CREG_COM_SETTINGS register 0x00
#define UM7_SAT_ENABLED               (1 << 4)    // Transmit satellite details when provided by GPS
#define UM7_GPS_ENABLED               (1 << 8)    // Transmit GPS data automatically whenever new GPS data is received
#define UM7_GPS_BAUD_START_BIT        24          // Specifies the start location of the GPS baud rate bits
#define UM7_GPS_BAUD_RATE_MASK        (0x07)      // Mask specifying the number of bits used to set the GPS baud rate
#define UM7_BAUD_START_BIT            28          // Specifies the start location of the serial baud rate bits
#define UM7_BAUD_RATE_MASK            (0x0F)      // Mask specifying the number of bits used to set the serial baud rate

// UM7_CREG_COM_RATES1 0x01
#define UM7_RAW_MAG_RATE_START_BIT    8           // Specifies the start bit of the magnetometer broadcast rate
#define UM7_RAW_MAG_RATE_MASK         (0x000FF)   // Mask specifying the desired raw magnetometer broadcast rate in Hz
#define UM7_RAW_GYRO_RATE_START_BIT   16          // Specifies the start bit of the gyro broadcast rate
#define UM7_RAW_GYRO_RATE_MASK        (0x000FF)   // Mask specifying the desired raw gyro broadcast rate in Hz
#define UM7_RAW_ACCEL_RATE_START_BIT  24          // Specifies the start bit of the accelerometer broadcast rate
#define UM7_RAW_ACCEL_RATE_MASK       (0x000FF)   // Mask specifying the desired raw accelerometer broadcast rate in Hz

// UM7_CREG_COM_RATES2 0x02 (If these values are set UM7_CREG_COM_RATES1 is ignored)
#define UM7_ALL_RAW_RATE_START_BIT    0           // Specifies the start bit for the broadcast rate of all raw data
#define UM7_ALL_RAW_RATE_MASK         (0x000FF)   // Mask specifying the desired broadcast rate in Hz of all raw data
#define UM7_TEMP_RATE_START_BIT       24          // Specifies the start bit of the temp data broadcast rate
#define UM7_TEMP_RATE_MASK            (0x000FF)   // Mask specifying the desired temp data broadcast rate in Hz

// UM7_CREG_COM_RATES3 0x03
#define UM7_PROC_MAG_RATE_START_BIT   8           // Specifies the start bit of the magnetometer broadcast rate
#define UM7_PROC_MAG_RATE_MASK        (0x000FF)   // Mask specifying the desired processed magnetometer broadcast rate in Hz
#define UM7_PROC_GYRO_RATE_START_BIT  16          // Specifies the start bit of the gyro broadcast rate
#define UM7_PROC_GYRO_RATE_MASK       (0x000FF)   // Mask specifying the desired processed gyro broadcast rate in Hz
#define UM7_PROC_ACCEL_RATE_START_BIT 24          // Specifies the start bit of the accelerometer broadcast rate
#define UM7_PROC_ACCEL_RATE_MASK      (0x000FF)   // Mask specifying the desired processed accelerometer broadcast rate in Hz

// UM7_CREG_COM_RATES4 0x04 (If these values are set UM7_CREG_COM_RATES3 is ignored)
#define UM7_ALL_PROC_RATE_START_BIT   0           // Specifies the start bit for the broadcast rate of all processed data
#define UM7_ALL_PROC_RATE_MASK        (0x000FF)   // Mask specifying the desired broadcast rate in Hz of all processed data

// UM7_CREG_COM_RATES5 0x05
#define UM7_VELOCITY_RATE_START_BIT   0           // Specifies the start bit for the broadcast rate for velocity data
#define UM7_VELOCITY_RATE_MASK        (0x000FF)   // Mask specifying the desired broadcast rate in Hz for velocity data
#define UM7_POSITION_RATE_START_BIT   8           // Specifies the start bit for broadcast rate for position data
#define UM7_POSITION_RATE_MASK        (0x000FF)   // Mask specifying the desired broadcast rate in Hz for position data
#define UM7_EULER_RATE_START_BIT      16          // Specifies the start bit for the broadcast rate for Euler angle data
#define UM7_EULER_RATE_MASK           (0x000FF)   // Mask specifying the desired broadcast rate in Hz for Euler angle data
#define UM7_QUAT_RATE_START_BIT       24          // Specifies the start bit for the broadcast rate for quaternion data
#define UM7_QUAT_RATE_MASK            (0x000FF)   // Mask specifying the desired broadcast rate in Hz for quaternion data

// UM7_CREG_COM_RATES6 0x06
#define UM7_HEALTH_RATE_START_BIT     16          // Specifies the start bit for the broadcast rate for the sensor health packet
#define UM7_HEALTH_RATE_MASK          (0x0F)      // Mask specifying the desired broadcast rate in Hz for the sensor health packet
#define UM7_POSE_RATE_START_BIT       24          // Specifies the start bit for broadcast rate for pose data
#define UM7_POSE_RATE_MASK            (0x000FF)   // Mask specifying the desired broadcast rate in Hz for pose data

// UM7_CREG_COM_RATES7 0x07 This is the same as previous register values, but for NMEA instead of binary
#define UM7_NMEA_QUAT_RATE_START_BIT      4           // Specifies the start bit for the broadcast rate for quaternion packets
#define UM7_NMEA_QUAT_RATE_MASK           (0x0F)      // Mask specifying the desired broadcast rate for quaternion packets
#define UM7_NMEA_GPS_POSE_RATE_START_BIT  8           // Specifies the start bit for broadcast rate for GPS pose packets
#define UM7_NMEA_GPS_POSE_RATE_MASK       (0x0F)      // Mask specifying the desired broadcast rate in for GPS pose packets
#define UM7_NMEA_RATES_RATE_START_BIT     12          // Specifies the start bit for the broadcast rate for rate data packets
#define UM7_NMEA_RATES_RATE_MASK          (0x0F)      // Mask specifying the desired broadcast rate in for rate data packets
#define UM7_NMEA_SENSOR_RATE_START_BIT    16          // Specifies the start bit for the broadcast rate for sensor data packets
#define UM7_NMEA_SENSOR_RATE_MASK         (0x0F)      // Mask specifying the desired broadcast rate in for sensor data packets
#define UM7_NMEA_ATTITUDE_RATE_START_BIT  20          // Specifies the start bit for the broadcast rate for attitude data packets
#define UM7_NMEA_ATTITUDE_RATE_MASK       (0x0F)      // Mask specifying the desired broadcast rate in for attitude data packets
#define UM7_NMEA_POSE_RATE_START_BIT      24          // Specifies the start bit for the broadcast rate for pose data packets
#define UM7_NMEA_POSE_RATE_MASK           (0x0F)      // Mask specifying the desired broadcast rate in for pose data packets
#define UM7_NMEA_HEALTH_RATE_START_BIT    28          // Specifies the start bit for the broadcast rate for sensor health data packets
#define UM7_NMEA_HEALTH_RATE_MASK         (0x0F)      // Mask specifying the desired broadcast rate in for sensor health data packets

// UM7_CREG_MISC_SETTINGS 0x08
#define UM7_MAG_ENABLED     (1 << 0)      // The magnetometer will be used in state updates
#define UM7_Q_ENABLED       (1 << 1)      // The sensor will run in quaternion mode rather than Euler angle mode
#define UM7_ZG_ENABLED      (1 << 2)      // The sensor will attempt to measure the rate gyro bias on startup
#define UM7_PPS_ENABLED     (1 << 8)      // Use the TX2 pin on the IO expansion header as the PPS input from an external GPS module
