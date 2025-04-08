#include <modbus/modbus.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <stdio.h>

int main() {
    modbus_t *ctx;
    int rc;
    uint16_t value = 0x0006; // Value to write (0x0006)

    // Create Modbus RTU context
    ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);
    if (ctx == NULL) {
        fprintf(stderr, "Failed to create RTU context: %s\n", modbus_strerror(errno));
        return -1;
    }

    // Set slave ID (0x64 = 100)
    modbus_set_slave(ctx, 0x64);

    // Enable debug mode (to see raw packets)
    modbus_set_debug(ctx, TRUE);

     //Connect to the device
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
     //   modbus_free(ctx);
     //   return -1;
    }

    while(1){
    // Write register 0x2000 (8192) with value 0x0006
    rc = modbus_write_register(ctx, 0x2000, value);
    if (rc == -1) {
        fprintf(stderr, "Write failed: %s\n", modbus_strerror(errno));
    } else {
        printf("Write successful!\n");
    }
	 sleep(1000);
    }

    // Close connection
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}
