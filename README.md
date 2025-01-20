# PID Control for ESP32 DevKit V1

This repository contains an Arduino sketch for implementing a PID (Proportional-Integral-Derivative) control system on an **ESP32 DevKit V1** microcontroller. The code reads an analog input from a potentiometer to set the desired temperature and controls an analog output signal accordingly.

## Features

- **PID Control Algorithm:** Efficient control for temperature management.
- **Analog Input and Output:** Reads temperature setpoints and outputs control signals.
- **ESP32 Compatibility:** Optimized for the ESP32 DevKit V1 board.
- **Customizable PID Parameters:** Adjustable proportional, integral, and derivative values.

---

## Required Hardware

- **ESP32 DevKit V1** (ESP-WROOM-32)
- **10KΩ potentiometer** (for input)
- **Temperature sensor DS18B20** (for temperature monitoring)
- **Resistor 4.7KΩ** (for DS18B20 pull-up)
- **Connections:**
  - **Potentiometer:** Connect to pin `34` (ADC1).
  - **Temperature sensor:** Connect to pin `5` with a pull-up resistor to 3.3V.
  - **Output load:** Connect to pin `25` (DAC1).

---

## Required Software and Libraries

Make sure you have the following software and libraries installed:

### 1. **Arduino IDE Setup**
Download and install the [Arduino IDE](https://www.arduino.cc/en/software). Then, follow these steps:

- Install ESP32 board support:
  1. Go to **File > Preferences**, and in the `Additional Board Manager URLs`, add:

      ```
      https://dl.espressif.com/dl/package_esp32_index.json
      ```

  2. Go to **Tools > Board > Boards Manager**, search for **ESP32**, and install the latest version of the package.

- Select the correct board settings under **Tools:**
  - **Board:** `ESP32 Dev Module`
  - **Flash Size:** `4MB (32Mb)`
  - **Upload Speed:** `115200`
  - **Partition Scheme:** `Default 4MB with spiffs`
  - **Port:** Select the correct COM port (e.g., COM3, /dev/ttyUSB0)

### 2. **Required Libraries**

Install the following libraries from the Arduino Library Manager (**Sketch > Include Library > Manage Libraries...**):

- **OneWire** (for DS18B20 temperature sensor)
- **DallasTemperature** (for temperature data processing)

To install via Arduino IDE:


#include <OneWire.h>
#include <DallasTemperature.h>

Alternatively, install manually using the command line:

arduino-cli lib install OneWire
arduino-cli lib install DallasTemperature

Installation
1.	Clone the repository:
git clone https://github.com/your-username/PID_ESP32_Controller.git
cd PID_ESP32_Controller

2.	Open the project:
	Open PID_ESP32_ANAOUT_POT.ino with Arduino IDE.
3.	Compile and upload the sketch:
	Select the correct ESP32 board under Tools > Board.
	Compile and upload the code to the ESP32 DevKit V1.
Usage
4.	Adjust the potentiometer to change the temperature setpoint.
5.	Monitor the serial output to check the current temperature and control output.
6.	Fine-tune the PID parameters in the code for better performance:
double Kp = 30.0;  // Proportional gain
double Ki = 15.0;  // Integral gain
double Kd = 5.0;   // Derivative gain

7.	Serial Monitor Output Example:

22.5 C | Setpoint: 23.0 C | Output: 3200 (78%)

8. Project File Structure
     
PID_ESP32_Controller/
│-- PID_ESP32_ANAOUT_POT.ino  # Main Arduino sketch
│-- README.md                 # Project documentation
│-- LICENSE                   # License file 

##Troubleshooting

If you encounter issues, check the following:
	•	Ensure the correct board and COM port are selected in the Arduino IDE.
	•	Verify wiring connections for the potentiometer and temperature sensor.
	•	If upload issues occur, try pressing the “BOOT” button on the ESP32 when uploading.
	•	Adjust the baud rate in the serial monitor to 115200.

 License

This project is licensed under the MIT License - see the LICENSE file for details.

Author

Fabio Bridarolli
GitHub (https://github.com/Superbrida) | LinkedIn

Enjoy and feel free to contribute!
 
