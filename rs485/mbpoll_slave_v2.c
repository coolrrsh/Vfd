#include <stdio.h>
#include <modbus/modbus.h>
#include <errno.h>
#include <unistd.h>

//#ADDED NEW
//Reply to the master 

int main() {
    modbus_t *ctx;
    modbus_mapping_t *mb_mapping;
    uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];

    // Create a new Modbus RTU context
    ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);
    if (ctx == NULL) {
        fprintf(stderr, "Unable to create Modbus context: %s\n", modbus_strerror(errno));
        return -1;
    }

    // Enable debugging
    modbus_set_debug(ctx, TRUE);

    // Set the Modbus slave ID
    if (modbus_set_slave(ctx, 100) == -1) {
        fprintf(stderr, "Failed to set slave ID: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // Open the Modbus connection
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // Set response timeout
    struct timeval response_timeout;
    response_timeout.tv_sec = 5;  // 5 seconds
    response_timeout.tv_usec = 0;
    modbus_set_response_timeout(ctx, response_timeout.tv_sec, response_timeout.tv_usec);

    // Create a Modbus mapping for holding registers
    mb_mapping = modbus_mapping_new(0, 0, 100, 0);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // Initialize holding registers with some test data
    for (int i = 0; i < 100; i++) {
        mb_mapping->tab_registers[i] = i * 2;  // Example: Even numbers
    }

    printf("Modbus RTU Slave is running. Listening for requests...\n");

    while (1) {
        printf("Waiting for Modbus request...\n");
        int rc = modbus_receive(ctx, query);
        
        if (rc > 0) {
            // Print the received frame
            printf("Received frame: ");
            for (int i = 0; i < rc; i++) {
                printf("%02X ", query[i]);
            }
            printf("\n");

            // Process the Modbus request and send a reply
            rc = modbus_reply(ctx, query, rc, mb_mapping);
            if (rc == -1) {
                fprintf(stderr, "Failed to send Modbus response: %s\n", modbus_strerror(errno));
            }
        } else if (rc == -1) {
            fprintf(stderr, "Error receiving data: %s\n", modbus_strerror(errno));
            sleep(1);  // Retry after a short delay
        }
    }

    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}

