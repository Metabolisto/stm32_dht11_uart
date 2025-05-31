/**
 * @file dht11.c
 * @brief Driver for DHT11 temperature and humidity sensor (STM32 + bare-metal)
 *
 * This module implements a simple bit-banging protocol to communicate
 * with the DHT11 sensor over a single GPIO line.
 *
 * Functions:
 * - DHT11_GetData(): main function to retrieve temperature and humidity
 *
 * Note:
 * - This code assumes 72 MHz system clock (STM32F103)
 * - Delay timings are approximate, and may need tuning
 * - No CRC libraries are used; checksum is verified manually
 *
 */
#include "main.h"
#include "dht11.h"

// Define the GPIO port and pin connected to the DHT11 sensor
#define DHT11_PORT GPIOB
#define DHT11_PIN  5

// Delay in microseconds (approximate based on 72 MHz clock)
static void Delay_us(uint32_t us) {
    us *= 8; // Calibrated multiplier for 72 MHz system clock
    while (us--) __NOP(); // No Operation instruction for delay
}

// Configure the DHT11 pin as output
static void GPIO_SetOutput(void) {
    DHT11_PORT->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5); // Clear mode and config bits
    DHT11_PORT->CRL |= GPIO_CRL_MODE5_1; // Output mode, 2 MHz push-pull
}

// Configure the DHT11 pin as input (floating)
static void GPIO_SetInput(void) {
    DHT11_PORT->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5); // Clear mode and config bits
    DHT11_PORT->CRL |= GPIO_CRL_CNF5_0; // Input mode, floating
}

// Read a single byte from the DHT11 sensor
static uint8_t DHT11_ReadByte(void) {
    uint8_t byte = 0;

    for (uint8_t i = 0; i < 8; i++) {
        // Wait for the data line to go HIGH (start of bit transmission)
        while (!(DHT11_PORT->IDR & (1 << DHT11_PIN)));

        Delay_us(40); // Wait for 40 us

        // If line is still HIGH after 40 us, it's a '1', otherwise '0'
        if (DHT11_PORT->IDR & (1 << DHT11_PIN)) {
            byte |= (1 << (7 - i));
        }

        // Wait for the line to go LOW again (end of bit transmission)
        while (DHT11_PORT->IDR & (1 << DHT11_PIN));
    }

    return byte;
}

// Main function to get temperature and humidity from DHT11
// Returns 1 if data was received successfully, 0 otherwise
uint8_t DHT11_GetData(uint8_t *temperature, uint8_t *humidity) {
    uint8_t data[5] = {0};

    // Start signal: pull the line LOW for at least 18 ms
    GPIO_SetOutput();
    DHT11_PORT->BSRR = (1 << (DHT11_PIN + 16)); // Pull pin LOW
    Delay_us(18000); // 18 ms delay

    DHT11_PORT->BSRR = (1 << DHT11_PIN); // Pull pin HIGH
    Delay_us(30); // Wait 30 us
    GPIO_SetInput(); // Release the bus and wait for response

    // Check if the sensor responds with LOW signal
    if (!(DHT11_PORT->IDR & (1 << DHT11_PIN))) {
        // Wait for the sensor's HIGH and LOW signals (response sequence)
        while (!(DHT11_PORT->IDR & (1 << DHT11_PIN)));
        while (DHT11_PORT->IDR & (1 << DHT11_PIN));

        // Read 5 bytes of data: humidity integer + decimal, temperature integer + decimal, checksum
        for (uint8_t i = 0; i < 5; i++) {
            data[i] = DHT11_ReadByte();
        }

        // Verify checksum
        if (data[0] + data[1] + data[2] + data[3] == data[4]) {
            *humidity = data[0];       // Integer part of humidity
            *temperature = data[2];    // Integer part of temperature
            return 1; // Success
        }
    }

    return 0; // Failure (no response or checksum error)
}
