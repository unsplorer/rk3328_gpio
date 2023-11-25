#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// Define register addresses
#define GRF_BASE                 0xff100000
#define GRF_GPIO1D_IOMUX_OFFSET  0x1c

// Function to read a 32-bit register
unsigned int readRegister(volatile void *addr) {
    return *((volatile unsigned int *)addr);
}

// Function to write a 32-bit value to a register
void writeRegister(volatile void *addr, unsigned int value) {
    *((volatile unsigned int *)addr) = value;
}

int main() {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        perror("Error opening /dev/mem");
        return -1;
    }

    // Map the GRF registers into user space
    void *grfMap = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, GRF_BASE);
    if (grfMap == MAP_FAILED) {
        perror("Error mapping /dev/mem");
        close(mem_fd);
        return -1;
    }

    // Read the current GPIO1D IOMUX control register value
    volatile void *gpio1dIomux = (volatile void *)((char *)grfMap + GRF_GPIO1D_IOMUX_OFFSET);
    unsigned int gpio1dIomuxValue = readRegister(gpio1dIomux);
    gpio1dIomuxValue |= 0xFFFF0000;


    // Set gpio1_d4_sel bits to 0x01
    gpio1dIomuxValue &= ~(0x3 << 8);  // Clear bits 9:8
    // gpio1dIomuxValue |= (0x01 << 8);   // Set bits 9:8 to 0x01

    // Write the updated value back to the GPIO1D IOMUX control register
    writeRegister(gpio1dIomux, gpio1dIomuxValue);

    // Unmap memory and close file descriptor
    munmap(grfMap, sysconf(_SC_PAGESIZE));
    close(mem_fd);

    return 0;
}
