import serial
import time
import pygame
import librosa
import pygame
import os
import time

def findDelay_music(audio_path):
    """
    Create an ASCII art animation that dances to the music's tempo while playing the audio
    
    Parameters:
        audio_path (str): Path to the audio file
    """
    # Initialize pygame mixer
    pygame.mixer.init()
    pygame.mixer.music.load(audio_path)
    
    # Load and analyze the audio
    y, sr = librosa.load(audio_path)
    onset_env = librosa.onset.onset_strength(y=y, sr=sr)
    tempo = librosa.beat.tempo(onset_envelope=onset_env, sr=sr)[0]
    
    # Calculate delay between frames based on tempo
    delay = 60.0 / tempo
    
    return delay







# Replace 'COM3' with the actual port (e.g., '/dev/ttyUSB0' on Linux, '/dev/tty.usbmodemXXXX' on macOS)
PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
DEFAULT_DURATION = 1.0  # default time in ms

def send_angles(data, duration=DEFAULT_DURATION):
    message = ",".join(map(str, data + [int(duration*1000)])) + "\n"
    arduino.write(message.encode())
    print('sent:', data, f'duration: {duration}ms')
    time.sleep(duration)

def play_audio(file_path):
    pygame.mixer.init()
    pygame.mixer.music.load(file_path)
    pygame.mixer.music.play()
    
    # Keep the program running while the audio is playing
    while pygame.mixer.music.get_busy():
        continue

def scene_1():
    print("\nScene1")
    send_angles([180,0,0,180,0])
    send_angles([120,60,15,150,30])
    play_audio("puppetSounds/scene1_line1.mp3")
    print("Whoa.....")
    send_angles([120,60,15,150,30],1.5)

    send_angles([180,0,15,180,0],0.7)
    send_angles([120,60,15,180,0],0.7)
    send_angles([180,0,15,180,0],0.7)
    send_angles([120,60,15,180,0],0.7)
    send_angles([180,0,15,180,0],0.7)

    play_audio("puppetSounds/scene1_line2.mp3")
    print("Haha...")
    time.sleep(1)

    send_angles([180,60,15,180,0],0.7)
    send_angles([120,0,15,180,0],0.7)
    send_angles([180,0,15,150,0],0.7)
    send_angles([180,0,15,180,0],0.7)

def scene_2():
    print("\nScene2")
    play_audio("puppetSounds/scene2_line1.mp3")
    print('Okay its time.....')
    send_angles([180,0,15,180,0])

    send_angles([180,0,15,150,0],0.7)
    send_angles([180,0,15,180,30],0.7)
    send_angles([180,0,15,180,0],0.7)

    play_audio("puppetSounds/scene2_line2.mp3")
    print('Walking is.....')
    time.sleep(3.5)

    play_audio("puppetSounds/scene2_line3.mp3")
    print('Look at .......')
    time.sleep(1)

    for _ in range(3):  # Repeat walking cycle 3 times
        send_angles([180, 0, 15, 150, 0], 0.5)
        send_angles([180, 0, 15, 0, 30], 0.5)

def scene_4():
    print("\nScene4")
    play_audio("puppetSounds/scene4_line1.mp3")
    send_angles([180,0,15,180,0],2.0)

    send_angles([120,60,15,140,40],0.5)
    send_angles([180,0,15,180,0],0.5)

    play_audio("puppetSounds/scene4_line2.mp3")
    print('Woohoo.....')
    time.sleep(6)

def scene_5():
    print("\nScene5")
    send_angles([180,0,15,180,0])
    play_audio("puppetSounds/scene5_line1.mp3")
    print('Oh no its.......')
    time.sleep(4)
    play_audio("puppetSounds/scene5_line2.mp3")
    print('Okay friends.......')
    
    for _ in range(3):
        send_angles([180, 60, 15, 180, 0])
        send_angles([180, 0, 15, 180, 0])

import random

def dance(audio_path):
    delay = findDelay_music(audio_path)
    play_audio(audio_path)
    print("Dancing to the music...")
    
    # Define the available dance steps as functions
    dance_steps = [
        lambda: [send_angles([180, 60, 15, 180, 0],delay), send_angles([120, 0, 15, 180, 0],delay)],
        lambda: [send_angles([180, 0, 15, 150, 0],delay), send_angles([120, 60, 15, 180, 30],delay)],
        lambda: [send_angles([120, 0, 15, 175, 0],delay), send_angles([150, 60, 15, 180, 15],delay)],
        lambda: [send_angles([180, 60, 15, 175, 0],delay), send_angles([120, 0, 15, 180, 30],delay)],
        lambda: [send_angles([180, 0, 15, 180, 30],delay), send_angles([180, 60, 15, 180, 0],delay)]
    ]
    
    while pygame.mixer.music.get_busy():
        # Randomly select a dance step
        selected_step = random.choice(dance_steps)
        
        # Perform the selected dance step
        for _ in range(5):  # Repeat the selected step 5 times
            selected_step()
            time.sleep(delay)
        

def scene_6():
    print("\nScene6")
    send_angles([180,0,15,180,0])
    play_audio("puppetSounds/scene6_line1.mpeg")
    print('Fredie v jasom.....')
    time.sleep(4)
    
    for _ in range(5):
        send_angles([180, 60, 15, 180, 0])
        send_angles([120, 0, 15, 180, 0])

    for _ in range(5):
        send_angles([180, 0, 15, 150, 0])
        send_angles([120, 60, 15, 180, 30])
    
    for _ in range(5):
        send_angles([120, 0, 15, 175, 0])
        send_angles([150, 60, 15, 180, 15])
    
    for _ in range(5):
        send_angles([180, 60, 15, 175, 0])
        send_angles([120, 0, 15, 180, 30])

    for _ in range(2):
        send_angles([180, 0, 15, 180, 30])
        send_angles([180, 60, 15, 180, 0])

try:
    # Open serial connection
    arduino = serial.Serial(PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for connection to establish
    
    print("Connected to Arduino")

    # Send data
    arduino.write(b'Hello Arduino\n')

    # Read response (if any)
    response = arduino.readline().decode().strip()
    if response:
        print("Arduino says:", response)

    scene_1()
    time.sleep(2.0)
    scene_2()
    time.sleep(2.0)
    scene_6()
    time.sleep(2.0)
    scene_4()
    time.sleep(2.0)
    scene_5()
    time.sleep(1.0)
    send_angles([180,0,15,180,0])
    # Close connection
    # arduino.close()

except serial.SerialException as e:
    print("Error:", e)