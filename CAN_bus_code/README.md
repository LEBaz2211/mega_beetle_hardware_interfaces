# CAN BUS 

##  Message format

CANBUS communications uses a frame of data made of three parts
- An ID ex: 0x10
- Data Length Code(dlc) determines the length of the DATA ex: 8
- The data, an array of bytes.

## CAN for Raspbery PICO
### SETUP
To begin you can install the [pico setup installer](https://github.com/raspberrypi/pico-setup-windows/releases/latest/download/pico-setup-windows-x64-standalone.exe) to be have the right environment(windows)

AND the [CAN interface library](https://github.com/adamczykpiotr/pico-mcp2515)

In the project folder you will find a CmakeLists file that will define the libraries and sdk used for the projects<br><br>
Setup the project and add the libraries.

```
add_executable(pico-mcp2515
    # Library executables
    include/mcp2515/mcp2515.cpp

    # Main executalbes
    src/pico-mcp2515.cpp
)
# Add MCP2515 Lib
target_include_directories(pico-mcp2515 PUBLIC include/)

pico_set_program_name(pico-mcp2515 "pico-mcp2515")
pico_set_program_version(pico-mcp2515 "0.1")

pico_enable_stdio_uart(pico-mcp2515 0)
pico_enable_stdio_usb(pico-mcp2515 1)

# Add any user requested libraries
target_link_libraries(pico-mcp2515
    pico_stdlib
    hardware_spi
)

pico_enable_stdio_usb(pico-mcp2515 1) 
pico_enable_stdio_uart(pico-mcp2515 0) 


pico_add_extra_outputs(pico-mcp2515)
```
### UPLOAD FILE TO THE PICO CARD
In your project folder create a folder called **build**

In a terminal use the cd command to go into that folder

Use the command
```
cmake ../ -G "Unix Makefiles"
```
This will create files in the build folder. 
<br>
Still in the same terminal use the *make* command to create the file to upload to the pico.
```
make
```
<br>
This will create multiple additional files. The one you need is the file with *.uf2* extension.
<br>
Plug the pico in boot mode and upload the file.

### PIN CONFIGURATIONS
| MCP    |PICO PIN |
| -------- | ------- |
|SCK_PIN|18|
|SI_PIN|19|
|SO_PIN|16|
|CSN_PIN|17|

### PRODUCE
In the C++ file include the libraries
```
#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"
#include <stdint.h>

MCP2515 can0;
struct can_frame rx;
```

And in the main() function initialize the CAN
```
stdio_init_all();
// pico_enable_stdio_usb(NULL, 0);

//Initialize interface
can0.reset();
can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
can0.setNormalMode();
```
Be sure to change the BitRate parameters accordingly.

We create a function that creates a can frame.

```
void produce_can_frame(canid_t can_id, __u8 can_dlc, __u8 data[]) {
    struct can_frame rx;
    rx.can_id = can_id;
    rx.can_dlc = can_dlc;
    for (int i = 0; i < can_dlc; i++) {
        rx.data[i] = data[i];
    }
    return rx;
}

int main() {
    stdio_init_all();

    //Initialize interface
    can0.reset();
    can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    can0.setNormalMode();
    // Create a test frame
    test_frame = produce_can_frame(0x50, 8, {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE});

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        if(can0.sendMessage(&test_frame) == MCP2515::ERROR_OK) {
        // blinks led if message is sent properly
            blink_led(100, LED_PIN);
        }
       
    }

    return 0;
}
```

## CAN for Arduino
### SETUP
In your ARDUINO IDE install the [AA_MCP2515 library](https://github.com/codeljo/AA_MCP2515)

### PIN configurations

| MCP    | PINS |
| -------- | ------- |
|SCK_PIN|13|
|SI_PIN|11|
|SO_PIN|12|
|CS_PIN|10|
|INT|2|

### CONSUME

include the library and configure the bitrate and pins
```
#include "AA_MCP2515.h"

// TODO: modify CAN_BITRATE, CAN_PIN_CS(Chip Select) pin, and CAN_PIN_INT(Interrupt) pin as required.
const CANBitrate::Config CAN_BITRATE = CANBitrate::Config_8MHz_500kbps;
const uint8_t CAN_PIN_CS = 10;
const int8_t CAN_PIN_INT = 2;
```
In void setup 
```
while(CAN.begin(CANController::Mode::Normal) != CANController::OK) {
    // Serial.println("CAN begin FAIL - delaying for 1 second");
    delay(10);
  }
```

In the loop create a frame object and read data into it
```
CANFrame frame;
CANController::IOResult rxResult = CAN.read(frame);

id = frame.getId()
dlc = frame.getDlc()
data = frame.getData()

```
