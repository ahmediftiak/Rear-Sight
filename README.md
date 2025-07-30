# RearSight Firmware

**RearSight** is a blind-spot detection system for bicycles and motorcycles using TF-Luna LiDAR sensors.

This project is a modern C++ firmware rewrite of our original Arduino-based senior capstone, now built with [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/) for improved performance, modularity, and maintainability.

---

## 🔧 Features

- ESP32-based firmware (C++)
- Modular I2C bus driver (`I2CBus`)
- Multi-sensor TF-Luna (I2C) support
- I2C device scanning
- Foundation for distance-based LED alerts

---

## 🚲 Use Case

Enhances rider safety by detecting vehicles or obstacles in the blind spot and providing LED-based warnings.

---

## 📁 Project Structure

```text
/main
  ├── main.cpp        # Entry point with I2C scan
  ├── I2CBus.cpp/.hpp # C++ I2C driver
  ├── TFLuna.hpp      # TF-Luna class stub (WIP)
```

---

## 🚧 Coming Soon

- TF-Luna protocol: read distance, change address
- FreeRTOS tasks for sensor and LED control
- Real-time alert logic

---

## 🧠 Credits

Originally developed as part of our 2024 senior capstone project 
https://github.com/dwang312/Blind-Sight

---

## 📜 License
MIT License



