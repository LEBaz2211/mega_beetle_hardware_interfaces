#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"
#include <stdint.h>


MCP2515 can0;
struct can_frame rx;
#define CAN_MAX_DLEN 8

typedef uint32_t canid_t;
typedef uint8_t __u8;


void blink_led(uint delay_ms, uint LED_PIN) {
        gpio_put(LED_PIN, 1); // Turn the LED on
        sleep_ms(delay_ms);   // Wait for 'delay_ms' milliseconds
        gpio_put(LED_PIN, 0); // Turn the LED off
        sleep_ms(delay_ms);   // Wait for 'delay_ms' milliseconds
        return;
}

int main() {
    stdio_init_all();
    // pico_enable_stdio_usb(NULL, 0);

    //Initialize interface
    can0.reset();
    can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    can0.setNormalMode();

    struct can_frame test_frame;
    test_frame.can_id = 0x50;
    test_frame.can_dlc = 8;
    test_frame.data[0] = 0xDE;
    test_frame.data[1] = 0xAD;
    test_frame.data[2] = 0xBE;
    test_frame.data[3] = 0xEF;
    test_frame.data[4] = 0xCA;
    test_frame.data[5] = 0xFE;
    test_frame.data[6] = 0xBA;
    test_frame.data[7] = 0xBE;

     // Fix: Swap the second and third arguments
    // send works
    // SO on pin 16, SI on 19


    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        if(can0.sendMessage(&test_frame) == MCP2515::ERROR_OK) { 
            blink_led(10, LED_PIN);
            printf("New frame from ID: %10x\n", rx.can_id, rx.can_dlc);
        }
       
    }

    return 0;
}
