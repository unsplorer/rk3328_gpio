#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include "c_gpio.h"


#define GPIO_BASE_OFFSET           0xFF210000
#define GPIO_ADDRESS_WIDTH         0x00010000
#define GPIO_SWPORTA_DR            0x0000
#define GPIO_SWPORTA_DDR           0x0004
#define GPIO_INTEN                 0x0030
#define GPIO_INTMASK               0x0034
#define GPIO_INTTYPE_LEVEL         0x0038
#define GPIO_INT_POLARITY          0x003C
#define GPIO_INT_STATUS            0x0040
#define GPIO_INT_RAWSTATUS         0x0044
#define GPIO_DEBOUNCE              0x0048
#define GPIO_PORTA_EOI             0x004C
#define GPIO_EXT_PORTA             0x0050
#define GPIO_LS_SYNC               0x0060

#define PAGE_SIZE  (4*1024)
#define BLOCK_SIZE (4*1024)

static volatile uint32_t *pio_map;
static volatile uint32_t *gpio_map;


// void short_wait(void)
// {
//     int i;

//     for (i=0; i<150; i++) {    // wait 150 cycles
//         asm volatile("nop");
//     }
// }

int setup(int gpio_bank)
{
    int mem_fd;
    uint8_t *gpio_mem;
    uint32_t gpio_base;

    // Open /dev/mem to access physical memory
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
        return SETUP_MMAP_FAIL;

    // Allocate memory for GPIO block and align to page size
    if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE - 1))) == NULL)
        return SETUP_MALLOC_FAIL;

    if ((uint32_t)gpio_mem % PAGE_SIZE)
        gpio_mem += PAGE_SIZE - ((uint32_t)gpio_mem % PAGE_SIZE);

    // Calculate GPIO base address
    gpio_base = GPIO_BASE_OFFSET + (gpio_bank * GPIO_ADDRESS_WIDTH);

    // Map GPIO memory to user space
    gpio_map = (uint32_t *)mmap((void *)gpio_mem, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem_fd, gpio_base);

    if ((uint32_t)gpio_map < 0)
        return SETUP_MMAP_FAIL;

    return SETUP_OK;
}

void clear_event_detect(int gpio)
{

}

int eventdetected(int gpio)
{

}

void set_rising_event(int gpio, int enable)
{

}

void set_falling_event(int gpio, int enable)
{

}

void set_high_event(int gpio, int enable)
{

}

void set_low_event(int gpio, int enable)
{

}

void set_pullupdn(int gpio, int pud)
{

  
}

void setup_gpio(int gpio, int direction, int pud)
{
    // Assuming GPIO_SWPORTA_DDR bit for the GPIO pin
    int bit_position = gpio % 32;

    // Set pull-up/down for the GPIO pin
    // set_pullupdn(gpio, pud);

    // Update the data direction register based on the specified direction
    if (direction == OUTPUT) {
        *(gpio_map + GPIO_SWPORTA_DDR / 4) = (*(gpio_map + GPIO_SWPORTA_DDR / 4) & ~(1 << bit_position)) | (1 << bit_position);
    } else {  // direction == INPUT
        *(gpio_map + GPIO_SWPORTA_DDR / 4) &= ~(1 << bit_position);
    }
  
}


void output_gpio(int gpio, int value)
{
    // Assuming the corresponding GPIO_SWPORTA_DR bit for the GPIO pin
    int bit_position = gpio % 32;

    // Set the corresponding bit in the SWPORTA_DDR register to configure the pin as output
    *(gpio_map + GPIO_SWPORTA_DDR / 4) |= (1 << bit_position);

    // Set or clear the corresponding bit in the SWPORTA_DR register based on the desired value
    if (value)
        *(gpio_map + GPIO_SWPORTA_DR / 4) |= (1 << bit_position);
    else
        *(gpio_map + GPIO_SWPORTA_DR / 4) &= ~(1 << bit_position);
  
}

int input_gpio(int gpio)
{
    // Assuming the corresponding GPIO_EXT_PORTA bit for the GPIO pin
    int bit_position = gpio % 32;

    // Read the value from the GPIO_EXT_PORTA register
    return (*(gpio_map + GPIO_EXT_PORTA / 4) >> bit_position) & 1;
  
}

void cleanup(void)
{
    munmap((void *)gpio_map, BLOCK_SIZE);
}
