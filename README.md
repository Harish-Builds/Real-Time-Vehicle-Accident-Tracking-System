# 🚨 Accident Tracker System

## 📌 Overview
The **Accident Tracker System** is an IoT-based safety solution designed to detect vehicle accidents in real time and notify emergency contacts with precise location details.  

It combines **embedded systems, real-time cloud communication, and a live dashboard** to improve emergency response time and road safety.

---

## ⚙️ Key Features
- 🚗 Real-time accident detection using accelerometer (impact & jerk analysis)
- 📍 Live GPS tracking with accurate coordinates
- 📩 Instant SMS alert system via GSM module
- ☁️ Cloud integration using Firebase Realtime Database
- 🗺️ Interactive web dashboard with map visualization (Leaflet.js)
- 📊 Continuous monitoring of speed, direction, and system status
- 🗂️ Historical accident logging for analysis

---

## 🧰 Technology Stack

### 🔌 Hardware
- ESP32 Microcontroller  
- GY-61 Accelerometer  
- NEO-7M GPS Module  
- SIM900A GSM Module  

### 💻 Software
- Arduino IDE (Embedded Programming)  
- HTML5, CSS3, JavaScript (Frontend)  
- Firebase Realtime Database (Backend)  
- Leaflet.js (Map Visualization)  

---

## 📂 Project Structure
Accident-Tracker/
│
├── AccidentTracker_ESP32.ino # Embedded firmware for ESP32
├── AccidentTracker.html # Web-based monitoring dashboard
├── README.md # Project documentation

---

## 🚀 System Workflow

flowchart TD
A[Accelerometer Data] --> B{Impact Detected?}
B -->|Yes| C[Fetch GPS Location]
C --> D[Send Data to Firebase]
D --> E[Trigger SMS via GSM]
E --> F[Update Web Dashboard]
B -->|No| A

⚡ Execution Steps
The accelerometer continuously monitors motion and vibration
A threshold-based algorithm detects sudden impact or jerk
GPS module retrieves real-time location data
ESP32 transmits data to Firebase Realtime Database
GSM module sends an emergency SMS with location link
Web dashboard updates instantly with accident details

🔧 Setup & Installation Guide
1️⃣ Hardware Configuration
Connect accelerometer to ESP32 analog input pins
Interface GPS module using UART communication
Connect GSM module via UART (hardware/software serial)
Ensure stable power supply for all modules

2️⃣ ESP32 Firmware Setup
->// Open Arduino IDE
->// Load the firmware file
->AccidentTracker_ESP32.ino
->Install Required Libraries
->TinyGPS++
->Firebase ESP32 Client
->Configure Credentials


#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"
#define FIREBASE_HOST "your_firebase_url"
#define FIREBASE_AUTH "your_firebase_key"
#define EMERGENCY_NUMBER "+91XXXXXXXXXX"

3️⃣ Firebase Configuration
Create a project in Firebase Console
Enable Realtime Database
Test Rules (Development Only)
{
  "rules": {
    ".read": true,
    ".write": true
  }
}
Copy Firebase configuration into:
ESP32 firmware
Web dashboard (AccidentTracker.html)


4️⃣ Running the Dashboard
# Open the dashboard in browser
AccidentTracker.html
Ensure active internet connection
Firebase must be properly configured

📊 Dashboard Capabilities
📍 Real-time GPS tracking on interactive map
🚨 Instant accident alert notifications
🚗 Live speed and heading visualization
🔧 Device and module status monitoring
📜 Scrollable accident history logs


🔐 Best Practices
Use environment variables for sensitive credentials
Restrict Firebase rules in production
Validate sensor data to reduce false positives
Ensure proper GSM signal strength
Implement debounce logic for accident detection


🔮 Future Enhancements
📱 Mobile application (Android/iOS)
🤖 AI-based accident validation
🚑 Integration with emergency services (API-based)
🔔 Push notifications & alert escalation
📈 Advanced cloud analytics dashboard


🤝 Contribution Guidelines
# Fork the repository

# Create a feature branch
git checkout -b feature/YourFeature

# Commit changes
git commit -m "Add YourFeature"

# Push to branch
git push origin feature/YourFeature
Open a Pull Request 🚀

📜 License

This project is licensed under the MIT License.

👨‍💻 Author

Harish K

⭐ Support

If you find this project useful, consider giving it a ⭐ on GitHub!
