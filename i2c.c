#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>

#define RK3328_I2C_BASE_ADDR    0xFF150000  

// Replace these with actual register offsets
#define RKI2C_CON               0x0000          // control register
#define RKI2C_CLKDIV            0x0004          // clock divider register
#define RKI2C_MRXADDR           0x0008          // slave address access for master rx mode
#define RKI2C_MRXRADDR          0x000C          // slave register address accessed for master tx mode
#define RKI2C_MTXCNT            0x0010          // master transmit count
#define RKI2C_MRXCNT            0x0014          // master rx count
#define RKI2C_IEN               0x0018          // interrupt enable register
#define RKI2C_IPD               0x001c          // interrupt pending register
#define RKI2C_FCNT              0x0020          // finished count
#define RKI2C_TXDATA0           0x0100          // I2C tx data register 0
#define RKI2C_TXDATA1           0x0104          // I2C tx data register 1
#define RKI2C_TXDATA2           0x0108          // I2C tx data register 2
#define RKI2C_TXDATA3           0x010c          // I2C tx data register 3
#define RKI2C_TXDATA4           0x0110          // I2C tx data register 4
#define RKI2C_TXDATA5           0x0114          // I2C tx data register 5
#define RKI2C_TXDATA6           0x0118          // I2C tx data register 6
#define RKI2C_TXDATA7           0x011c          // I2C tx data register 7
#define RKI2C_RXDATA0           0x0200          // I2C tx data register 0
#define RKI2C_RXDATA1           0x0204          // I2C tx data register 1
#define RKI2C_RXDATA2           0x0208          // I2C tx data register 2
#define RKI2C_RXDATA3           0x020c          // I2C tx data register 3
#define RKI2C_RXDATA4           0x0200          // I2C tx data register 4
#define RKI2C_RXDATA5           0x0214          // I2C tx data register 5
#define RKI2C_RXDATA6           0x0218          // I2C tx data register 6
#define RKI2C_RXDATA7           0x021c          // I2C tx data register 7

#define PAGE_SIZE (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

volatile uint32_t *i2c_map;  // Pointer to I2C registers

// Function to map I2C registers to virtual memory
int map_i2c_registers() {
    int mem_fd;
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        perror("Failed to open /dev/mem");
        return -1;
    }

    i2c_map = (uint32_t *)mmap(
        NULL,
        PAGE_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        RK3328_I2C_BASE_ADDR
    );

    if (i2c_map == MAP_FAILED) {
        perror("Failed to map I2C registers");
        return -1;
    }

    close(mem_fd);
    return 0;
}

// Function to initialize I2C
void i2c_init() {
    // Perform I2C initialization, configure control registers, etc.
    // ...

    // Example: Set the I2C slave address
    // i2c_map[I2C_SLAVE_ADDR_REG] = 0x50;
}

// Function to write data to I2C
void i2c_write(uint8_t data) {
    // Write data to I2C
    // ...

    // Example: Load data into the data register and trigger write
    // i2c_map[I2C_DATA_REG] = data;
    // i2c_map[RKI2C_Con       ] |= (1 << 0);  // Set the write bit in the control register
}

// Function to read data from I2C
uint8_t i2c_read() {
    // Read data from I2C
    // ...

    // Example: Trigger read and wait for status
    // i2c_map[RKI2C_Con       ] &= ~(1 << 0);  // Clear the write bit in the control register
    // while (!(i2c_map[I2C_STATUS_REG] & (1 << 7)));  // Wait until data is ready

    // // Example: Return the read data
    // return i2c_map[I2C_DATA_REG];
}

// Function to cleanup I2C resources
void i2c_cleanup() {
    // Unmap the I2C registers
    munmap((void *)i2c_map, PAGE_SIZE);
}

int main() {
    if (map_i2c_registers() != 0) {
        fprintf(stderr, "Failed to map I2C registers\n");
        return -1;
    }

    i2c_init();

    // Example: Write and read operations
    i2c_write(0x55);
    uint8_t read_data = i2c_read();

    printf("Read data: 0x%X\n", read_data);

    i2c_cleanup();

    return 0;
}
