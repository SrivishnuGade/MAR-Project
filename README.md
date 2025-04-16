# 🎭 Puppet Show Project

A fun robotics project where a puppet performs animated shows synced with audio, complete with servo-controlled body movements and tempo-based dancing!

---

## 📌 Overview

This project showcases a **robotic puppet** that comes to life with the help of servo motors, synchronized audio, and motion logic. The project features both hardware implementation and software simulation:

### 🤖 Hardware Implementation
- Servo-controlled puppet with 5 degrees of freedom (head, arms, legs)
- Real-time audio synchronization and tempo-based movements
- Arduino-based motion control with smooth interpolation

### 💻 Software Simulation
- ROS2-based visualization in RViz
- Digital twin of the puppet for testing choreography
- Real-time visualization of joint positions and movements
- Simulated puppet control using the same core logic as hardware

The dual implementation allows for safe testing and choreography development in simulation before deploying to the physical puppet.

---

## ✨ Features

- 🧠 **Arduino-controlled servo motors** for lifelike limbs and head movement.
- 🔊 **External audio playback** for puppet dialogues and music.
- 💃 **Tempo-based dancing** — the puppet now moves rhythmically in sync with the beat/tempo of the music!
- 🧍‍♂️ **Puppet structure includes**:
  - Round Head  
  - Body  
  - Two Arms  
  - Two Legs
- 🕹️ **Controlled choreography** of movements through scripted patterns.

---

## 🔧 Hardware Components

- 5x Servo Motors (Head, 2 Arms, 2 Legs)
- Arduino Uno
- Computer for audio and Serial control
- Wires, Breadboard, Power Source
- Puppet Body (custom crafted from lightweight materials)

---

## 📁 Project Structure

```
MAR-Project/
├── my_puppet/                  # ROS2 package for software simulation
│   ├── launch/                 # Launch files
│   │   └── display.launch.py
│   ├── my_puppet/             # Python package
│   │   ├── puppet_controller.py
│   ├── package.xml
│   └── setup.py
│
├── puppetSounds/               # Audio files for puppet performance
│   ├── mtr_type_beat.mp3
│   ├── nefarious.mp3
│   ├── scene1_line1.mp3
│   ├── scene1_line2.mp3
│   └── ... (other audio files)
│
├── puppetController.py         # Main puppet control script
├── puppetArdSpeed.ino         # Arduino firmware
├── tempo.ipynb                # Music analysis notebook
└── README.md                  # Project documentation
```

### 📝 Key Files

- `my_puppet/`: ROS2 package for software simulation in RViz
- `puppetController.py`: Main Python script for controlling puppet movements and audio
- `puppetArdSpeed.ino`: Arduino code for servo control with smooth interpolation
- `tempo.ipynb`: Jupyter notebook for analyzing music tempo and creating dance moves




---

## 🔄 How It Works

- The **Arduino** controls all servo motors based on Serial inputs from computer.
- **Audio** is played from computer.
- For dancing, the `puppetController.py` script:
  - Extracts **tempo (BPM)** from audio files.
  - Generates **servo movement sequences** mapped to tempo.
  - Sends commands to Arduino via **Serial**.

---

## 🚀 Getting Started

### 💻 Software Simulation Path
1. **Install ROS2 Humble** (if not already installed)

2. **Create ROS2 Workspace**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```
3. **Install Dependencies**
   ```bash
   pip3 install ros2 colcon-common-extensions
   pip3 install librosa pyserial pygame numpy
   ```

4. **Add Puppet Package**
   ```bash
   # Copy the package to your ROS2 workspace
   cp -r /path/to/MAR-Project/my_puppet ~/ros2_ws/src/
   ```

5. **Build & Launch Simulation**
   ```bash
   # Navigate to workspace
   cd ~/ros2_ws

   # Build the package
   colcon build --symlink-install

   # Source the setup files
   source install/setup.bash

   # Launch the simulation
   ros2 launch my_puppet display.launch.py

6. **Verify Installation**
   - RViz window should open
   - Puppet model should be visible
   - Puppet should start dancing to music in 5seconds

Now you can develop choreography safely in simulation before deploying to hardware! 🎭

### 🤖 Hardware Implementation Path
1. **Install Dependencies**
   ```bash
   # Install required Python packages
   pip install librosa pyserial pygame numpy
   ```

2. **Arduino Setup**
   - Install Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software)
   - Open Arduino IDE
   - Load `puppetArdSpeed.ino`
   - Select "Arduino Uno" from Tools > Board
   - Select port from Tools > Port (usually `/dev/ttyUSB0` on Linux, `/dev/tty.usbmodem*` on macOS, `COM*` on Windows)
   - Click Upload button (→) to flash the sketch

3. **Hardware Connection**
   - Connect 5 servo motors:
     - Left Hand: Pin 3
     - Right Hand: Pin 4
     - Head: Pin 5
     - Left Leg: Pin 6
     - Right Leg: Pin 7
   - Connect Arduino to computer via USB
   - Optional: Use external 5V power supply for servos

4. **Run Controller**
   ```bash
   # First identify your Arduino port
   `/dev/ttyUSB0`   `/dev/tty.usbmodem*`    `COM*`
   
   # Edit puppetController.py to set correct port
   # Change PORT = "COM3" to your port, e.g.:
   # PORT = "/dev/cu.usbmodem14201"
   
   # Run the controller
   python puppetController.py
   ```
5. **Verify Connection**
   - Check serial monitor output
   - Servos should move to initial position
   - Try `scene_1()` by uncommenting it in `puppetController.py`



<!-- ### 📽️ Demo
./demo.mov -->

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

