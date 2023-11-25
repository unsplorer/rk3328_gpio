#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// Define register addresses
#define GRF_BASE                 0xff100000
#define GRF_GPIO1D_IOMUX_OFFSET  0x001c
#define GRF_GPIO2A_IOMUX_OFFSET  0x0020
#define GRF_GPIO2D_IOMUX_OFFSET  0x0034


// Function to read a 32-bit register
unsigned int readRegister(volatile void *addr) {
    return *((volatile unsigned int *)addr);
}

// Assumes little endian
void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
    
    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("");
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

    // Read the GPIO1D IOMUX control register
    volatile void *gpio1dIomux = (volatile void *)((char *)grfMap + GRF_GPIO1D_IOMUX_OFFSET);
    volatile void *gpio2dIomux = (volatile void *)((char *)grfMap + GRF_GPIO2D_IOMUX_OFFSET);
    volatile void *gpio2aIomux = (volatile void *)((char *)grfMap + GRF_GPIO2D_IOMUX_OFFSET);
    unsigned int gpio1dIomuxValue = readRegister(gpio1dIomux);
    unsigned int gpio2dIomuxValue = readRegister(gpio2dIomux);
    unsigned int gpio2aIomuxValue = readRegister(gpio2aIomux);

    // Extract specific bits from the GPIO1D IOMUX control register
    unsigned int gpio1dSel = (gpio1dIomuxValue >> 8) & 0x3;
    unsigned int gpio2dSel = gpio2dIomuxValue & 0x3;
    __uint8_t gpio2_a4Sel = (gpio2aIomuxValue >> 8) & 0x3;
    __uint8_t gpio2_a5Sel = (gpio2aIomuxValue >> 10) & 0x3;

    // Print the result
    // printBits(sizeof(gpio2_a4Sel),&gpio2a4Sel);
    printf("GPIO1D[4] IOMUX Select: 0x%x\n", gpio1dSel);
    printf("GPIO2A[4] IOMUX Select: 0x%x\n", gpio2_a4Sel);
    printf("GPIO2A[5] IOMUX Select: 0x%x\n", gpio2_a5Sel);
    // printf("GPIO2D[0] IOMUX Select: 0x%x\n", gpio2dSel);

    // Unmap memory and close file descriptor
    munmap(grfMap, sysconf(_SC_PAGESIZE));
    close(mem_fd);

    return 0;
}