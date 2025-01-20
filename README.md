# PID_ESP32_Controller

This repository contains an Arduino sketch for implementing a PID (Proportional-Integral-Derivative) control system on an ESP32 microcontroller. The code is designed to manage an analog output signal based on a potentiometer input, allowing precise control of system parameters.

## Features

- **PID Control Algorithm:** Implementation of a PID control system for precise tuning.
- **Analog Input and Output:** Reads setpoint values from a potentiometer and controls the system output accordingly.
- **ESP32 Compatibility:** Designed specifically for ESP32 microcontroller platforms.
- **Customizable Parameters:** Adjustable PID tuning for different applications.

## Requirements

To run this project, you will need the following:

- **Hardware:**
  - ESP32 development board
  - Potentiometer (for input)
  - Load (for output control)

- **Software:**
  - [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board support installed
  - Required libraries:
    - `PID_v1` (for PID control)
    - `AnalogWrite` (for ESP32 analog output control)

## Installation

1. Clone this repository to your local machine:

    ```bash
    git clone https://github.com/your-username/PID_ESP32_Controller.git
    cd PID_ESP32_Controller
    ```

2. Open the `PID_ESP32_ANAOUT_POT.ino` file in the Arduino IDE.

3. Install the required libraries via the Arduino Library Manager.

4. Compile and upload the code to your ESP32 board.

## Usage

1. Connect the potentiometer to the analog input pin of the ESP32.
2. Adjust the potentiometer to change the setpoint.
3. Monitor the analog output to verify the PID response.
4. Fine-tune the PID parameters as needed in the code:

    ```cpp
    double Kp = 2.0;  // Proportional gain
    double Ki = 5.0;  // Integral gain
    double Kd = 1.0;  // Derivative gain
    ```

## File Structure
PID_ESP32_Controller/
│– PID_ESP32_ANAOUT_POT.ino  # Main Arduino sketch
│– README.md                 # Project documentation
│– LICENSE                   # License file 

## Troubleshooting

If you experience issues, consider the following:

- Ensure the correct board is selected in the Arduino IDE (`ESP32 Dev Module`).
- Check the correct COM port is selected.
- Verify potentiometer and load connections.
- Adjust PID values if the response is unstable.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

**Fabio Bridarolli**  
[LinkedIn](https://www.linkedin.com/) | [GitHub](https://github.com/your-username)

---

Enjoy and feel free to contribute!
