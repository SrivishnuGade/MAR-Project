# 🎭 Puppet Show Project

A fun and interactive robotics project where a puppet performs animated shows synced with audio, complete with servo-controlled body movements and tempo-based dancing!

---

## 📌 Overview

This project showcases a **robotic puppet** that comes to life with the help of servo motors, synchronized audio, and motion logic. It is ideal for educational demonstrations, tech fairs, or just a cool side project!

---

## ✨ Features

- 🧠 **Arduino-controlled servo motors** for lifelike limb and head movement.
- 🔊 **External audio playback** for puppet dialogues and music.
- 💃 **Tempo-based dancing** — the puppet now moves rhythmically in sync with the beat/tempo of the music!
- 🧍‍♂️ **Puppet structure includes**:
  - Round Head  
  - Cylindrical Body  
  - Two Arms  
  - Two Legs
- 🕹️ **Controlled choreography** of movements through scripted patterns.

---

## 🔧 Hardware Components

- 5x Servo Motors (Head, 2 Arms, 2 Legs)
- Arduino Uno / Nano
- External Speaker Module or Bluetooth Speaker
- Wires, Breadboard, Power Source
- Puppet Body (custom crafted from lightweight materials)
- *(Optional)* MPU6050 sensor for future interactivity

---

## 📁 Project Structure


---

## 🔄 How It Works

- The **Arduino** controls all servo motors based on predefined scripts.
- **Audio** is played from an external speaker.
- For dancing, the `tempo_analyzer.py` script:
  - Extracts **tempo (BPM)** from audio files.
  - Generates **servo movement sequences** mapped to tempo.
  - Sends commands to Arduino via **Serial/Bluetooth**.

---

## 🚀 Getting Started

### 1. Upload Arduino Script

Flash `puppet_controller.ino` onto your Arduino board using the Arduino IDE.

### 2. Set Up the Audio

Place your audio files inside the `audio/` folder. Play them via a connected speaker during the performance.

### 3. Run the Tempo Analyzer

Install dependencies:
bash
pip install librosa pyserial

### 4. Watch the Puppet Dance!
Sit back and enjoy as the puppet dances in sync with the music's tempo 🎶

### 📽️ Demo
[📹 Insert link to a demo video here if available]
### 📚 Future Enhancements

🎵 Real-time audio beat detection
🎭 Multiple puppets with synchronized shows
🎙️ Voice-controlled puppet interaction
🧍‍♀️ Unity-based virtual puppet mirror

### 🙌 Authors

Simonna Anna Dcosta
Srivishnu Muni Gade
Sushma EJ
Tejasree C J

