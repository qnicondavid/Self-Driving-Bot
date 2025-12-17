# How to Run the Self-Driving Bot
---
## 📋 Prerequisites

| Software | Version | Purpose |
|----------|---------|---------|
| **Java Development Kit (JDK)** | 17 or higher | Runs the JavaFX GUI application |
| **Maven** | 3.6 or higher | Builds and manages the Java project |
| **Arduino IDE** | Latest recommended | Uploads firmware to the robot |
| **Hardware** | Robot with compatible sensors/motors | Physical robot platform |

---

## 🚀 Step-by-Step Setup

###  1. Clone the Repository
```bash
git clone https://github.com/qnicondavid/Self-Driving-Bot
cd Self-Driving-Bot
```

### 2. Configure and Upload the Robot Code
- Open the `.ino` file in the Arduino IDE.
- Adjust the pin definitions to match your robot's wiring.
- Connect your robot to the PC via USB.
- Select the correct board (Arduino Feather M0) and port in the Arduino IDE.
- Upload the sketch to the robot.

### 3. Power and Connect to the Robot
- Plug the robot.
- On your PC, connect to the Wi-Fi network.
- SSID: `Group 19`
- Password: `ShutDown1`

### 4. Build and Run the GUI Application
- Navigate to the GUI project folder in your terminal.
- Compile the project: `bash mvn compile`
- Launch the JavaFX application: `mvn javafx:run`
---
## 💡 Notes
- Ensure the robot and PC are on the same Wi-Fi network.
- Verify the correct COM port is selected in the Arduino IDE before uploading.
- If you encounter Maven dependency issues, try: `mvn clean install`
