/*
Copyright (c) 2012-2015 Ben Croston

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

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
#define MAP_SIZE        (4096*2)
#define MAP_MASK        (MAP_SIZE - 1)

typedef struct sunxi_gpio {
    unsigned int CFG[4];
    unsigned int DAT;
    unsigned int DRV[2];
    unsigned int PULL[2];
} sunxi_gpio_t;

/* gpio interrupt control */
typedef struct sunxi_gpio_int {
    unsigned int CFG[3];
    unsigned int CTL;
    unsigned int STA;
    unsigned int DEB;
} sunxi_gpio_int_t;

typedef struct sunxi_gpio_reg {
    struct sunxi_gpio gpio_bank[9];
    unsigned char res[0xbc];
    struct sunxi_gpio_int gpio_int;
} sunxi_gpio_reg_t;

#define GPIO_BANK(pin)  ((pin) >> 5)
#define GPIO_NUM(pin)   ((pin) & 0x1F)

#define GPIO_CFG_INDEX(pin)     (((pin) & 0x1F) >> 3)
#define GPIO_CFG_OFFSET(pin)    ((((pin) & 0x1F) & 0x7) << 2)

#define GPIO_PUL_INDEX(pin)     (((pin) & 0x1F )>> 4)
#define GPIO_PUL_OFFSET(pin)    (((pin) & 0x0F) << 1)

int pinea64_found = 0;

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


// int gpio_function(int gpio)
// {
//     int offset = FSEL_OFFSET + (gpio/10);
//     int shift = (gpio%10)*3;
//     int value = *(gpio_map+offset);
//     value >>= shift;
//     value &= 7;
//     return value; // 0=input, 1=output, 4=alt0
//   }


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
