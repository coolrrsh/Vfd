#include <stdio.h>
#include <modbus/modbus.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>

#define SLAVE_ID 100
#define REGISTER_LOGIC_COMMAND 0x2000  // 8192 in decimal
#define REGISTER_SIZE 100

int main() {
    modbus_t *ctx;
    modbus_mapping_t *mb_mapping;
    uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];

    // Create Modbus RTU Context
    ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);
    if (ctx == NULL) {
        fprintf(stderr, "Unable to create Modbus context: %s\n", modbus_strerror(errno));
        return -1;
    }

    // Enable debugging
    modbus_set_debug(ctx, TRUE);

    // Set Modbus Slave ID
    if (modbus_set_slave(ctx, SLAVE_ID) == -1) {
        fprintf(stderr, "Failed to set slave ID: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // Open Modbus Connection
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // Set Response Timeout
    struct timeval response_timeout;
    response_timeout.tv_sec = 5;  
    response_timeout.tv_usec = 0;
    modbus_set_response_timeout(ctx, response_timeout.tv_sec, response_timeout.tv_usec);

    // Create Modbus Mapping (Holding Registers)
    mb_mapping = modbus_mapping_new(0, 0, REGISTER_SIZE, 0);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // Initialize Holding Registers
    for (int i = 0; i < REGISTER_SIZE; i++) {
        mb_mapping->tab_registers[i] = 0;
    }

    printf("Modbus RTU Slave is running. Listening for requests...\n");

    while (1) {
        printf("Waiting for Modbus request...\n");
        int rc = modbus_receive(ctx, query);
        
        if (rc > 0) {
            // Print Received Frame
            printf("Received frame: ");
            for (int i = 0; i < rc; i++) {
                printf("%02X ", query[i]);
            }
            printf("\n");

            // Extract Register Address
            uint16_t received_register = (query[2] << 8) | query[3];

            if (received_register == REGISTER_LOGIC_COMMAND) {
                printf("Writing to REGISTER_LOGIC_COMMAND (0x2000)\n");

                // Extract Data (MSB first)
                uint16_t data_value = (query[4] << 8) | query[5];
                mb_mapping->tab_registers[8192 - 8192] = data_value;  // Store in Register 0

                printf("Received Data: 0x%04X (MSB: 0x%02X, LSB: 0x%02X)\n", 
                        data_value, query[4], query[5]);
            }

            // Reply to Modbus Master
            rc = modbus_reply(ctx, query, rc, mb_mapping);
            if (rc == -1) {
                fprintf(stderr, "Failed to send Modbus response: %s\n", modbus_strerror(errno));
            }
        } else if (rc == -1) {
            fprintf(stderr, "Error receiving data: %s\n", modbus_strerror(errno));
            sleep(1);
        }
    }

    // Free Memory
    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}

