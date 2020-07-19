#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>      // File control definitions
#include <errno.h>    // Error number definitions
#include <assert.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

#include <glib.h>
#include <gio/gio.h>

#include "micro/microstrain_comm.h"
#include "micro/util.h"

#include "libbot/timestamp.h"
#include "libbot/rotations.h"
#include "libbot/small_linalg.h"
#include "libbot/ringbuf.h"

using namespace std;

// Global reference to Glib main loop
static GMainLoop* mainloop = NULL;

// Global ROS publisher
ros::Publisher imu_data_pub_;

// Core app self structure
class app_t
{
public:
    // Communication variables
    int comm, status;
    char comm_port_name[255];
    unsigned int baud_rate, data_rate;

    // dt for angular rate and acceleration computation
    double delta_t;

    // Window size for gyro and accelerometer digital filter
    int filter_window_size;

    // Input buffer variables
    Byte input_buffer[INPUT_BUFFER_SIZE];
    Byte message_mode;
    BotRingBuf* read_buffer;
    char current_segment;
    int expected_segment_length;

    // Packet variables
    int message_size;
    int message_start_byte;

    // State flags (not currently used, but may be in future)
    bool changed_baud_rate;
    bool changed_data_rate;
    bool in_continuous_mode;

    // boolean setting flags
    bool quiet, verbose;

    bool little_endian;

    int64_t utime_prev;

    string channel;

    // Our imu message
    sensor_msgs::Imu reading;

    // Libbot time sync (currently not used)
    bot_timestamp_sync_state* sync;
    bool do_sync;
};

/**
 * Callback function that will exit the glib loop
 */
static void sig_action(int signal, siginfo_t* s, void* user)
{
    // kill the glib main loop...
    if (g_main_loop_is_running(mainloop)) {
        g_main_loop_quit(mainloop);
    }
}

/**
 * Set our signal callback, and have it called on
 * SIGINT, SIGTERM, SIGKILL, SIGHUP
 */
void install_signal_handler()
{
    struct sigaction action;
    action.sa_sigaction = sig_action;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;

    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGKILL, &action, NULL);
    sigaction(SIGHUP, &action, NULL);
}

/**
 * This function scans for active serial ports, that could have the microstrain device on it
 * Once the device is selected, the port name is found, and updated
 */
bool scandev(char* comm_port_name)
{
    FILE* instream;
    char devnames[255][255]; // allows for up to 256 devices with path links up to 255 characters long each
    int devct = 0;           // counter for number of devices
    int i = 0;
    int j = 0;
    int userchoice = 0;

    // The command we want to execute
    // char command[] = "find /dev/serial -print | grep -i microstrain";
    char command[] = "ls /dev/ttyACM*";

    printf("Searching for devices...\n");

    // Execute piped command in read mode
    instream = popen(command, "r");

    // If unable to open the pipe, return the method
    if (!instream) {
        printf("ERROR BROKEN PIPELINE %s\n", command);
        return false;
    }

    // Load the char array for each device
    for (i = 0; i < 255 && (fgets(devnames[i], sizeof(devnames[i]), instream)); i++) {
        ++devct;
    }

    for (i = 0; i < devct; i++) {
        for (j = 0; j < sizeof(devnames[i]); j++) {
            if (devnames[i][j] == '\n') {
                devnames[i][j] = '\0'; // replaces newline inserted by pipe reader with har array terminator character
                break;                 // breaks loop after replacement
            }
        }
        printf("Device Found:\n%d: %s\n", i, devnames[i]);
    }

    // Have the user select what device they want to use
    // If there is not a valid device, return false
    if (devct > 0) {
        printf("Number of devices = %d\n", devct);
        if (devct > 1) {
            printf("Please choose the number of the device to connect to (0 to %i):\n", devct - 1);
            // Check that there's input and in the correct range
            while (scanf("%i", &userchoice) == 0 || userchoice < 0 || userchoice > devct - 1) {
                printf("Invalid choice...Please choose again between 0 and %d:\n", devct - 1);
                // Clear carriage return from keyboard buffer after invalid choice
                getchar();
            }
        }
        strcpy(comm_port_name, devnames[userchoice]);
        return true;
    } else {
        printf("No MicroStrain devices found.\n");
        return false;
    }
}

/**
 * This method takes a communications port, and configures it
 * Based on the com port passed from `open_com_port` the baudRate is set
 */
int setup_com_port(int comPort, speed_t baudRate)
{
    // Get the current options for the port...
    struct termios options;
    tcgetattr(comPort, &options);

    // Set the desired baud rate (default for MicroStrain is 115200)
    // int baudRate = B115200;
    cfsetospeed(&options, baudRate);
    cfsetispeed(&options, baudRate);

    // Set the number of data bits.
    options.c_cflag &= ~CSIZE; // Mask the character size bits
    options.c_cflag |= CS8;

    // Set the number of stop bits to 1
    options.c_cflag &= ~CSTOPB;

    // Set parity to None
    options.c_cflag &= ~PARENB;

    // Set for non-canonical (raw processing, no echo, etc.)
    options.c_iflag = IGNPAR; // ignore parity check close_port(int
    options.c_oflag = 0;      // raw output
    options.c_lflag = 0;      // raw input

    // Time-Outs -- won't work with NDELAY option in the call to open
    options.c_cc[VMIN] = 0;    // block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 100; // Inter-Character Timer -- i.e. timeout= x*.1 s

    // Set local mode and enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);

    tcflush(comPort, TCIOFLUSH);

    // Set the new options for the port...
    int status = tcsetattr(comPort, TCSANOW, &options);

    // For error message
    if (status != 0) {
        printf("Configuring comport failed\n");
        return status;
    }

    // Purge serial port buffers
    tcflush(comPort, TCIOFLUSH);

    return comPort;
}

/**
 * This method opens the communications port with the imu (3DM-GX3-25 sensor)
 * After the port is open, the configuration method is called to set the baudrate
 */
int open_com_port(const char* comPortPath, speed_t baudRate)
{
    int comPort = open(comPortPath, O_RDWR | O_NOCTTY);

    if (comPort == -1) {
        // Opening of port failed
        printf("Unable to open com Port %s\n Errno = %i\n", comPortPath, errno);
        return -1;
    }

    return setup_com_port(comPort, baudRate);
}

/*
 * This function computes the checksum to veryif the imu packets
 * It is the  "sum of all preceding bytes with rollover from 65535 to 0"
 */
unsigned short cksum(const Byte* packet_bytes, int packet_length)
{
    unsigned short check_sum_val = 0;
    for (int ii = 0; ii < packet_length - 2; ii++) {
        check_sum_val += packet_bytes[ii];
    }
    return check_sum_val;
}

/*
 * This method sets the imu into continuous mode
 * This means that the imu will edmit data events on the serial port
 */
bool set_continuous_mode(app_t* app)
{
    unsigned char set_mode_string[] = {
        0xC4,
        0xC1,
        0x29,
        app->message_mode
    };

    if (app->verbose)
        cout << "Setting continuous mode" << endl;

    // Send our command to the imu
    if (write(app->comm, set_mode_string, LENGTH_CONTINUOUS_MODE) != LENGTH_CONTINUOUS_MODE) {
        cerr << "Error writing command to set continuous mode" << endl;
        return false;
    }
    return true;
}

 /*
 * This method stops the imu's continues mode
 * The imu does not return a response
 */
void stop_continuous_mode(app_t* app)
{
    unsigned char stop_mode_string[] = {
        0xFA,
        0x75,
        0xB4
    };

    // Send our command to the imu
    if (write(app->comm, stop_mode_string, 3) != 3) {
        cerr << "Error writing command to stop continuous mode" << endl;
    }
}

/*
 * Does a soft device reset
 * This will return the imu settings to default
 * The imu does not return a reponse
 */
void soft_reset(app_t* app)
{
    unsigned char soft_reset_string[] = {
        0xFE,
        0x9E,
        0x3A
    };

    // Send our command to the imu
    if (write(app->comm, soft_reset_string, 3) != 3) {
        cerr << "Error writing command to stop continuous mode" << endl;
    }
}

/**
 * This method sets the baud rate
 * The baudrate is the "bandwidth" of the serial pipe
 */
bool set_comms_baud_rate(app_t* app)
{
    Byte baud0, baud1, baud2, baud3;

    // Convert our int baud rate, into 4 seperate bytes
    makeUnsignedInt32(app->baud_rate, &baud3, &baud2, &baud1, &baud0);

    unsigned char set_comms_baud_rate_string[] = {
        COMMS_SETTINGS_COMMAND, // Byte  1  : command
        0xC3, // Bytes 2-3: confirm intent
        0x55,
        0x01,  // Byte  4  : port selector
        0x01,  // Byte  5  : temporary change
        baud3, // Bytes 6-9: baud rate
        baud2,
        baud1,
        baud0,
        0x02, // Byte  10 : port config
        0x00  // Byte  11 : reserved (zero)
    };

    if (app->verbose)
        cout << "Setting baud rate" << endl;

    // Send our command to the imu
    if (write(app->comm, set_comms_baud_rate_string, LENGTH_COMMS_SETTINGS) != LENGTH_COMMS_SETTINGS) {
        cerr << "Error writing command to set comms baud rate" << endl;
        return false;
    }
    return true;
}

/**
 * This methods updates the imu settings based on the user's preference
 * The datarate (frequency) and filter window size is updated
 */
bool set_sampling_settings(app_t* app)
{
    Byte ocsb;       // orientation, coning & sculling byte
    Byte decu, decl; // decimation value upper & lower bytes
    Byte wndb;       // digital filter window size byte

    unsigned int decimation = 1000 / app->data_rate;
    makeUnsignedInt16(decimation, &decu, &decl);

    // don't compute orientation if we're running at max rate
    if (app->data_rate == DATA_RATE_HIGH)
        ocsb = 0x02;
    else
        ocsb = 0x03;

    wndb = static_cast<Byte>(app->filter_window_size);

    unsigned char set_sampling_params_string[] = {
        SAMPLING_SETTINGS_COMMAND, // Byte  1    : command
        0xA8,                      // Bytes 2-3  : confirm intent
        0xB9,
        0x01, // Byte  4    : change params
        decu, // Bytes 5-6  : decimation value
        decl,
        0x00, // Bytes 7-8  : flags - orient
        ocsb,
        wndb, // Byte  9    : gyro/accel window size
        0x11, // Byte  10   : magneto window size
        0x00, // Byte  11-12: up compensation
        0x0A,
        0x00, // Byte  13-14: north compensation
        0x0A,
        0x01, // Byte  15   : low magneto power
        0x00, // Bytes 16-20: reserved (zeros)
        0x00, 0x00, 0x00, 0x00
    };

    if (app->verbose)
        cout << "Setting sampling settings" << endl;

    // Send our command to the imu
    if (write(app->comm, set_sampling_params_string, LENGTH_SAMPLING_SETTINGS) != LENGTH_SAMPLING_SETTINGS) {
        cerr << "Error writing command to set sampling settings" << endl;
        return false;
    }
    return true;
}

/**
 * This method handles all messages sent from the imu
 * The configuration steps are also feed through this method
 * Once the message type is determined, this method is called
 */
bool handle_message(app_t* app)
{

    // Our core timer, and sync time
    int ins_timer;
    int64_t utime = bot_timestamp_now();

    float vals[9];

    if (app->verbose) {
        fprintf(stderr, "Received data packet:\n");
        print_array_char_hex((unsigned char*)app->input_buffer, app->message_size);
    }

    bool success = true;

    // Go through each message type, and see if we have a way to handle it
    switch (app->message_start_byte) {
    case ACC_ANG_MAG_ROT: {
        if (!app->quiet)
            printf("error, received ACC_ANG_MAG_ROT instead of ACC_ANG_MAG\n");
        break;
    }
    case ACC_ANG_MAG: {
        if (!app->quiet)
            printf("error, received ACC_ANG_MAG instead of ACC_ANG_MAG_ROT (no quat received)\n");
        break;
    }
    case ACC_STAB: {
        if (!app->quiet)
            printf("error: received unexpected ACC_STAB message\n");
        break;
    }
    case DANG_DVEL_MAG: {
        if (!app->quiet)
            printf("error: received unexpected DANG_DVEL_MAG message\n");
        break;
    }
    case ACCEL_ANGRATE_ORIENT: {
        // Get our linear acceleration
        unpack32BitFloats(vals, &app->input_buffer[1], 3, app->little_endian);
        app->reading.linear_acceleration.x = vals[0] * GRAVITY;
        app->reading.linear_acceleration.y = vals[1] * GRAVITY;
        app->reading.linear_acceleration.z = vals[2] * GRAVITY;

        // Get our angular velocity
        unpack32BitFloats(vals, &app->input_buffer[13], 3, app->little_endian);
        app->reading.angular_velocity.x = vals[0];
        app->reading.angular_velocity.y = vals[1];
        app->reading.angular_velocity.z = vals[2];

        // Skip out magnetometer readings, we don't have a way to edmit those

        // Get our orientation matrix, and convert it to quat
        unpack32BitFloats(vals, &app->input_buffer[37], 9, app->little_endian);
        // This libbot2 function is faulty:
        // bot_matrix_to_quat(rot, ins_message.quat);
        // Workaround (from mfallon, oct2011)
        float ms_rpy[] = { 0, 0, 0 };
        float q[] = { 0, 0, 0, 0 };
        ms_rpy[0] = atan2(vals[5], vals[8]); // roll
        ms_rpy[1] = asin(-vals[2]); // pitch
        ms_rpy[2] = atan2(vals[1], vals[0]); // yaw
        roll_pitch_yaw_to_quat(ms_rpy, q);

        // Set the calculated quat values
        app->reading.orientation.x = q[0];
        app->reading.orientation.y = q[1];
        app->reading.orientation.z = q[2];
        app->reading.orientation.w = q[3];

        // Append current ros timestamp
        app->reading.header.stamp = ros::Time::now();

        // Publish to our topic
        imu_data_pub_.publish(app->reading);

        break;
    }
    case CONTINUOUS_MODE_COMMAND: {
        fprintf(stderr, "Received continuous mode command echo\n");
        app->in_continuous_mode = true;
        break;
    }
    case SAMPLING_SETTINGS_COMMAND: {
        if (app->verbose)
            fprintf(stderr, "Recieved sampling settings command echo\n");

        if (app->data_rate != DATA_RATE_DEFAULT) {
            app->changed_data_rate = true;
        }

        success = set_continuous_mode(app);
        break;
    }
    case COMMS_SETTINGS_COMMAND: {
        if (app->verbose)
            fprintf(stderr, "Recieved comms baud rate command echo\n");

        // received echo at the current baud rate, now switch to desired baud rate
        if (setup_com_port(app->comm, app->baud_rate) < 0) {
            success = false;
        } else {
            app->changed_baud_rate = true;
            success = set_sampling_settings(app);
        }
        break;
    }
    default: {
        if (!app->quiet)
            fprintf(stderr, "Unknown message start byte: %d\n", app->message_start_byte);
        break;
    }
    }

    return success;
}

/**
  * This methods gets the packets from the  circular buffer
  * It has either 2 states, looking for headers, or looking for data + the checksum bytes
  * If it is either of these two cases, it handles the message, and calls on the `handle_message` method
  */
void unpack_packets(app_t* app)
{
    while (bot_ringbuf_available(app->read_buffer) >= app->expected_segment_length) {
        switch (app->current_segment) {
        case 's':
            bot_ringbuf_peek(app->read_buffer, 1, (uint8_t*)&app->message_start_byte);

            if (app->verbose)
                fprintf(stderr, "received message start byte: id=%d\n", app->message_start_byte);

            app->current_segment = 'p';

            switch (app->message_start_byte) {
            case ACCEL_ANGRATE_ORIENT:
                app->expected_segment_length = LENGTH_ACCEL_ANGRATE_ORIENT;
                break;
            case ACC_ANG_MAG:
                app->expected_segment_length = LENGTH_ACC_ANG_MAG;
                break;
            case ACC_ANG_MAG_ROT:
                app->expected_segment_length = LENGTH_ACC_ANG_MAG_ROT;
                break;
            case ACC_STAB:
                app->expected_segment_length = LENGTH_ACC_STAB;
                break;
            case DANG_DVEL_MAG:
                app->expected_segment_length = LENGTH_DANG_DVEL_MAG;
                break;
            case CONTINUOUS_MODE_COMMAND:
                app->expected_segment_length = LENGTH_CONTINUOUS_MODE_ECHO;
                break;
            case SAMPLING_SETTINGS_COMMAND:
                app->expected_segment_length = LENGTH_SAMPLING_SETTINGS_ECHO;
                break;
            case COMMS_SETTINGS_COMMAND:
                app->expected_segment_length = LENGTH_COMMS_SETTINGS_ECHO;
                break;
            default:
                if (!app->quiet) {
                    fprintf(stderr, "no match for message start byte %d\n", app->message_start_byte);
                }
                // read a byte and continue if we don't have a match
                bot_ringbuf_read(app->read_buffer, 1, (uint8_t*)&app->message_start_byte);
                app->current_segment = 's';
                break;
            }
            break;
        case 'p':
            bot_ringbuf_read(app->read_buffer, app->expected_segment_length, app->input_buffer);
            unsigned short transmitted_cksum = make16UnsignedInt(&app->input_buffer[app->expected_segment_length - 2], app->little_endian);
            unsigned short computed_cksum = cksum(app->input_buffer, app->expected_segment_length);
            if (computed_cksum != transmitted_cksum) {
                if (!app->quiet)
                    fprintf(stderr, "Failed check sum! got: %d, expected: %d\n", transmitted_cksum, computed_cksum);
                break;
            }

            if (app->verbose)
                fprintf(stderr, "Passed checksum, handling message\n");

            bool message_success = handle_message(app);
            if (!message_success && !app->quiet)
                fprintf(stderr, "Message handling failed\n");

            app->current_segment = 's';
            app->expected_segment_length = 1;
            break;
        }
    }
}

/**
  * This is the callback function from the glib main loop
  * Reads serial bytes from ardu as they become available from g_io_watch
  * These bytes are then writen to a circular buffer and the `unpack_packets` method is called
  */
static gboolean serial_read_handler(GIOChannel* source, GIOCondition condition, void* user)
{

    // Check to see if the user has requested a stop
    if (!ros::ok()) {
        g_main_loop_quit(mainloop);
        return true;
    }

    app_t* app = (app_t*)user;

    static uint8_t middle_buffer[INPUT_BUFFER_SIZE];

    // get number of bytes available
    int available = 0;

    if (ioctl(app->comm, FIONREAD, &available) != 0) {
        if (!app->quiet)
            fprintf(stderr, "ioctl check for bytes available didn't return 0, breaking read\n");
        return true;
    }

    if (available > INPUT_BUFFER_SIZE) {
        if (!app->quiet)
            fprintf(stderr, "too many bytes available: %d, flushing input buffer\n", available);
        tcflush(app->comm, TCIFLUSH);
        return true;
    }

    int num_read = read(app->comm, middle_buffer, available);

    if (num_read != available) {
        if (!app->quiet)
            fprintf(stderr, "warning, read %d of %d available bytes\n", num_read, available);
    }

    if (num_read > 0) {
        bot_ringbuf_write(app->read_buffer, num_read, middle_buffer);
    }

    unpack_packets(app);

    return true;
}

/**
 * Our main method
 * This method first reads in all parameter information
 * After setting the configuration of the driver, the imu is set to continuous mode
 * This main glib loop is what waits for data from the imu untill the proccess is ended
 */
int main(int argc, char** argv)
{
    app_t* app = new app_t();
    app->little_endian = systemLittleEndianCheck();

    ros::init(argc, argv, "microstrain_comm");
    ros::NodeHandle nh("~");

    // Our publisher
    ros::NodeHandle imu_node_handle("imu");
    imu_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data", 100);

    // Defualt message mode
    app->message_mode = ACCEL_ANGRATE_ORIENT;

    // Default settings
    string user_comm_port_name;
    string data_rate;

    bool acc_ang_mag_rot;
    bool acc_ang_mag;
    bool acc_stab;

    // Get our params from the config file, or command line
    nh.param("verbose", app->verbose, false);
    nh.param("quiet", app->quiet, false);
    nh.param("com_port", user_comm_port_name, string(""));
    nh.param("rate", data_rate, string("low"));
    nh.param("window", app->filter_window_size, FILTER_WINDOW_SIZE_DEFAULT);
    nh.param("quat", acc_ang_mag_rot, false);
    nh.param("no_delta", acc_ang_mag, false);
    nh.param("filter", acc_stab, false);
    nh.param("time_sync", app->do_sync, true);

    // Data rate (which also determines baud rate)
    if (data_rate == "low") {
        app->data_rate = DATA_RATE_DEFAULT;
        app->baud_rate = BAUD_RATE_DEFAULT;
        app->delta_t = DELTA_ANG_VEL_DT_DEFAULT;
    } else if (data_rate == "medium") {
        app->data_rate = DATA_RATE_MED;
        app->baud_rate = BAUD_RATE_MED;
        app->delta_t = DELTA_ANG_VEL_DT_MED;
    } else if (data_rate == "high") {
        app->data_rate = DATA_RATE_HIGH;
        app->baud_rate = BAUD_RATE_HIGH;
        app->delta_t = DELTA_ANG_VEL_DT_HIGH;
    } else {
        cerr << "Unknown update rate flag - using default rate" << endl;
    }

    if (!app->quiet)
        cout << "Setting data rate to " << app->data_rate << " Hz" << endl;

    // Make sure filter window size isn't too big or small
    if (app->filter_window_size < FILTER_WINDOW_SIZE_MIN) {
        app->filter_window_size = FILTER_WINDOW_SIZE_DEFAULT;
        cerr << "Digital filter window size too small, using default size" << endl;
    } else if (app->filter_window_size > FILTER_WINDOW_SIZE_MAX) {
        app->filter_window_size = FILTER_WINDOW_SIZE_DEFAULT;
        cerr << "Digital filter window size too large, using default size" << endl;
    }

    if (!app->quiet)
        cout << "Setting digital filter window size to " << app->filter_window_size << endl;

    // Modes are mutually exlusive
    if (acc_stab) {
        app->message_mode = ACC_STAB;
    } else if (acc_ang_mag) {
        app->message_mode = ACC_ANG_MAG;
    } else if (acc_ang_mag_rot) {
        if (app->data_rate != DATA_RATE_HIGH) {
            app->message_mode = ACC_ANG_MAG_ROT;
        } else {
            cout << "Can't compute orientation at high speed, using \"Acceleration, Angular Rate & Orientation Matrix\" mode" << endl;
            app->message_mode = ACCEL_ANGRATE_ORIENT;
        }
    }

    if (!app->quiet)
        fprintf(stderr, "Little endian = %d\n", (int)app->little_endian);

    mainloop = g_main_loop_new(NULL, FALSE);
    app->utime_prev = bot_timestamp_now();
    app->sync = bot_timestamp_sync_init(62500, (int64_t)68719 * 62500, 1.001);
    app->read_buffer = bot_ringbuf_create(INPUT_BUFFER_SIZE);

    // Use user specified port if there is one
    if (user_comm_port_name == "")
        scandev(app->comm_port_name);
    else
        strcpy(app->comm_port_name, user_comm_port_name.c_str());

    // Initialize comm port at default baud rate
    app->comm = open_com_port(app->comm_port_name, BAUD_RATE_DEFAULT);
    if (app->comm < 0) {
        exit(1);
    }

    // Install signal handler
    install_signal_handler();

    // Simple state machine
    if (app->data_rate == DATA_RATE_DEFAULT && app->filter_window_size == FILTER_WINDOW_SIZE_DEFAULT) {
        // Set continous mode and we're done
        if (!set_continuous_mode(app)) {
            exit(1);
        }
    } else if (app->data_rate == DATA_RATE_DEFAULT) {
        // Set filter window size, then set continous mode
        if (!set_sampling_settings(app)) {
            exit(1);
        }
    } else {
        // Set baud rate, then sampling settings, then continuous mode
        if (!set_comms_baud_rate(app)) {
            exit(1);
        }
    }

    // Set our current segment for unpacking
    app->current_segment = 's';
    app->expected_segment_length = 1;
    
    // Create a glib channel and main thread
    GIOChannel* ioc = g_io_channel_unix_new(app->comm);
    g_io_add_watch_full(ioc, G_PRIORITY_HIGH, G_IO_IN, (GIOFunc)serial_read_handler, (void*)app, NULL);
    g_main_loop_run(mainloop);

    // Received signal - soft reset to cleanup before quitting
    soft_reset(app);

    // Close our imu port
    close(app->comm);
    return 0;
}
