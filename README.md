# Safety Bracelet for Seniors  

## Overview  
The **Safety Bracelet for Seniors** is a wearable device designed to enhance the safety and security of elderly individuals. It offers critical features such as fall detection, real-time location tracking, and an emergency button to send SMS notifications to a designated contact.  

This system is built on the **STM32F411RE microcontroller**, programmed in **C**, and leverages **FreeRTOS** for multitasking, ensuring efficient and reliable performance.  

---

## Features  
- **Fall Detection**: Detects sudden movements using the **ADXL345 accelerometer** and sends an SMS with the wearer’s location via the GSM module.  
- **Location Tracking**: Provides real-time location updates using the **NEO-6M GPS Module**.  
- **Emergency Alert**: If the wearer presses the emergency button, an SMS with their location is sent to a predefined contact.  

---

## Hardware Configuration  
- **ADXL345 (Accelerometer)**  
  - `PB8` → I2C1_SCL  
  - `PB9` → I2C1_SDA  

- **NEO-6M GPS Module**  
  - `PA9` → USART1_TX  
  - `PA10` → USART1_RX  

- **GSM Module (SIM800L)**  
  - `PC6` → USART6_TX  
  - `PC7` → USART6_RX  

- **Serial Monitor (e.g., TeraTerm)**  
  - By default, the STM32F411RE board connects `USART2` to the PC via USB. Simply connect the board to the PC with a USB cable and open a serial monitor to view debugging logs.  

---

## System Architecture  
- **Microcontroller**: STM32F411RE for task management and signal processing.  
- **Accelerometer**: ADXL345 to detect falls through movement analysis.  
- **GPS Module**: NEO-6M for location tracking.  
- **Communication**: GSM module (SIM800L) to send SMS alerts.  
- **Multitasking**: FreeRTOS manages concurrent tasks for seamless operation.  

---

## Technologies and Tools  
- **Programming Language**: C  
- **RTOS**: FreeRTOS  
- **Sensors**: ADXL345 Accelerometer, NEO-6M GPS Module  
- **Development Board**: STM32F411RE  
- **Communication**: GSM Module (SIM800L)  

---

## How It Works  
1. **Fall Detection**:  
   - The **ADXL345 accelerometer** monitors movements.  
   - Upon detecting a fall, the system sends an SMS with the wearer’s location via the **GSM module**.  

2. **Emergency Alert**:  
   - Pressing the emergency button sends an SMS with the wearer’s location to a predefined contact.  

3. **Location Tracking**:  
   - The **NEO-6M GPS Module** provides real-time location data used in SMS alerts.  

---

## Installation and Usage  
1. **Clone the repository**:  
   ```bash
   git clone https://github.com/Houssem70/STM32F4_FreeRTOS_Project.git
2. **Flash the firmware** to the STM32F411RE microcontroller.  
3. **Connect the hardware components**: ADXL345, NEO-6M, and GSM module as per the hardware configuration described above.  
4. **Configure the emergency contact number** in the source code.  
5. **Connect the STM32F411RE board** to your PC with a USB cable. Open a serial monitor (e.g., TeraTerm) to view debugging logs on `USART2`.  
6. **Power up the device** and start using it.  

---

## Future Enhancements  
- Add sensors for advanced health monitoring (e.g., heart rate, SPO2).  
- Improve power efficiency to extend battery life.  
- Integrate with a mobile app for real-time notifications and remote monitoring.

---

Feel free to explore, contribute, or reach out with feedback!
