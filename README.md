# **Arduino Motor Control System with Sensors**

This Arduino code controls a robot with four motors using PID control. It integrates encoders for RPM measurement, a QMC5883L compass for orientation, and an AS5600 magnetic encoder for angular position. 

## **Features**
- PID control for four motor wheels.
- RPM calculation using encoders.
- Orientation measurement with a QMC5883L compass.
- Angular position tracking with an AS5600 magnetic encoder.
- Communication over the serial port.

---

## **Hardware Setup**

### **Required Components**
1. Arduino Mega 2560
2. QMC5883L compass
3. AS5600 magnetic encoder
4. L298N motor drivers (4 units)
5. Motors with encoders
6. PCA9548A I2C multiplexer
7. Jumper wires and power supply

### **Wiring**

#### **Motor Connections**
- **Back Right (BR):**
  - IN1, IN2, BR_PWM, BR_ENCODER_1, BR_ENCODER_2
- **Back Left (BL):**
  - IN3, IN4, BL_PWM, BL_ENCODER_1, BL_ENCODER_2
- **Front Right (FR):**
  - IN5, IN6, FR_PWM, FR_ENCODER_1, FR_ENCODER_2
- **Front Left (FL):**
  - IN7, IN8, FL_PWM, FL_ENCODER_1, FL_ENCODER_2

#### **Sensor Connections**
- **QMC5883L Compass:**
  - Connected to PCA9548A I2C multiplexer, channel `COMPASS_CHANNEL`.
- **AS5600 Magnetic Encoder:**
  - Connected to PCA9548A I2C multiplexer, channel `AS5600_CHANNEL`.

#### **PCA9548A Multiplexer**
- **Address:** `0x70`
- Connect SDA and SCL to Arduino I2C pins.

---

## **Software Requirements**

### **Libraries Used**
- [TimerOne](https://github.com/PaulStoffregen/TimerOne)
- [QMC5883LCompass](https://github.com/keepworking/Arduino-QMC5883L)
- [AS5600](https://github.com/jcbrnld/AS5600)

Install these libraries through the Arduino Library Manager or download them from their respective repositories.

---

## **Setup Instructions**

### **1. Upload Code**
- Open the provided `.ino` file in the Arduino IDE.
- Select the correct board (`Arduino Mega 2560`) and port under `Tools`.
- Click `Upload`.

### **2. Power the Circuit**
- Ensure proper power connections for the Arduino, motor drivers, and sensors.

### **3. Serial Connection**
- Connect the Arduino to your Linux PC using a USB cable.

---

## **Serial Communication on Linux**

### **Find the Arduino Port**
Run the following command in the terminal:
```bash
ls /dev/tty*
```
Look for a device named similar to `/dev/ttyUSB0` or `/dev/ttyACM0`.

### **Connect to the Serial Port**
Use `screen` or a similar terminal application:
```bash
screen /dev/ttyUSB0 115200
```
Replace `/dev/ttyUSB0` with the actual port name.

To exit the `screen` session, press:
```
Ctrl + A, then type :quit
```

---

## **How It Works**

### **1. Motor Control**
- Each motor's speed is controlled by a PID algorithm based on the target RPM.

### **2. Sensor Readings**
- The QMC5883L compass provides orientation data.
- The AS5600 encoder tracks the robot's angular position.

### **3. Serial Interface**
- The robot's angular position, motor RPM, and other diagnostic data are sent to the serial port at 10 Hz.
- You can send commands to the Arduino in the format:
  ```
  linear_speed,angular_speed\n
  ```
  **Example:**
  ```
  1.0,0.5\n
  ```

---

## **Troubleshooting**

### **1. No Serial Output**
- Ensure the correct serial port is selected.
- Verify the baud rate is set to `115200`.

### **2. Incorrect Sensor Readings**
- Double-check wiring and connections.
- Verify calibration values for the QMC5883L compass.

### **3. Motor Issues**
- Ensure proper power supply to motor drivers.
- Check PID parameters for stability.
