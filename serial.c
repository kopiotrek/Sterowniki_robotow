#include "stm32l476xx.h"
#include <stdio.h>

#define DHT11_PIN 0  // Pin PC0

void delay_us(uint32_t us) {
    uint32_t i;
    for (i = 0; i < us * 5; i++) {  // assuming 80 MHz clock
        __NOP();
    }
}

void send_start_signal() {
    GPIOC->MODER |= GPIO_MODER_MODE0_0;  // set PC0 as output
    GPIOC->ODR &= ~GPIO_ODR_OD0;         // set PC0 low
    delay_us(18000);                     // delay for at least 18 ms
    GPIOC->ODR |= GPIO_ODR_OD0;          // set PC0 high
    delay_us(20);                        // delay for 20 us
}

uint8_t read_byte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        GPIOC->MODER &= ~GPIO_MODER_MODE0;  // set PC0 as input
        uint32_t count = 0;
        while ((GPIOC->IDR & GPIO_IDR_ID0) && count < 100) {  // wait for low signal
            delay_us(1);
            count++;
        }
        delay_us(30);  // wait for 30 us
        if ((GPIOC->IDR & GPIO_IDR_ID0) == 0) {  // check if signal is low
            byte |= (1 << (7 - i));  // set bit in byte if signal is low
        }
        while (!(GPIOC->IDR & GPIO_IDR_ID0)) {  // wait for signal to go high
            delay_us(1);
        }
    }
    return byte;
}

void read_data(uint8_t* data) {
    send_start_signal();
    for (int i = 0; i < 5; i++) {
        data[i] = read_byte();
    }
}

int main(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;  // enable clock for GPIOC
    while (!(RCC->AHB2ENR & RCC_AHB2ENR_GPIOCEN)) { }  // wait for clock to stabilize

    uint8_t data[5];
    read_data(data);

    uint8_t humidity = data[0];
    uint8_t temperature = data[2];

    printf("Humidity: %d%%\n", humidity);
    printf("Temperature: %dC\n", temperature);

    while (1) { }
}
