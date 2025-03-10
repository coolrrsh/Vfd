#include <stdio.h>
#include <stdlib.h>
#include <modbus.h>
#include <errno.h>
#include <string.h>

// VFD Modbus Configuration
#define VFD_SLAVE_ID 100           // VFD's Modbus slave ID
#define VFD_CONTROL_REGISTER 0x2000 // Register 8192 (0x2000 in hex)

// Control Commands
#define RESET_COMMAND 0x0000       // Reset command (neutral state)
#define START_COMMAND 0x0002       // Start command without direction
#define FORWARD_COMMAND 0x0012     // Start + Forward command
#define REVERSE_COMMAND 0x0022     // Start + Reverse command
#define STOP_COMMAND 0x0001        // Stop command

// Function to send commands to the VFD
void send_vfd_command(modbus_t *mb, uint16_t command) {
    int rc = modbus_write_register(mb, VFD_CONTROL_REGISTER, command);
    if (rc == -1) {
        fprintf(stderr, "Failed to send command: %s\n", modbus_strerror(errno));
    } else {
        printf("Command sent successfully: 0x%04X\n", command);
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s [start|forward|reverse|stop]\n", argv[0]);
        return EXIT_FAILURE;
    }

    // Initialize Modbus RTU connection
    modbus_t *mb = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);
    if (!mb) {
        fprintf(stderr, "Unable to create Modbus context: %s\n", modbus_strerror(errno));
        return EXIT_FAILURE;
    }

    // Set the Modbus slave ID
    if (modbus_set_slave(mb, VFD_SLAVE_ID) == -1) {

        fprintf(stderr, "Failed to set slave ID: %s\n", modbus_strerror(errno));
        modbus_free(mb);
        return EXIT_FAILURE;
    }

    // Connect to the VFD
    if (modbus_connect(mb) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(mb);
        return EXIT_FAILURE;
    }

    // Handle user input and send the appropriate command
    if (strcmp(argv[1], "start") == 0) {
        printf("Resetting to neutral state...\n");
        send_vfd_command(mb, RESET_COMMAND); // Reset the VFD to neutral state
        printf("Starting VFD...\n");
        send_vfd_command(mb, START_COMMAND);
    } else if (strcmp(argv[1], "forward") == 0) {
        printf("Setting Forward direction...\n");
        send_vfd_command(mb, FORWARD_COMMAND);
    } else if (strcmp(argv[1], "reverse") == 0) {
        printf("Setting Reverse direction...\n");
        send_vfd_command(mb, REVERSE_COMMAND);
    } else if (strcmp(argv[1], "stop") == 0) {
        printf("Stopping VFD...\n");
        send_vfd_command(mb, STOP_COMMAND);
    } else {
        fprintf(stderr, "Invalid command. Use 'start', 'forward', 'reverse', or 'stop'.\n");
        modbus_close(mb);
        modbus_free(mb);
        return EXIT_FAILURE;
    }

    // Close the Modbus connection
    modbus_close(mb);
    modbus_free(mb);

    return EXIT_SUCCESS;
}
