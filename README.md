# Solis Robot - SoBot
![](https://github.com/SolisTecnologia/SoBot-Learning-Mode/blob/master/png/SoBotControlF710.png)
# Introduction

AMR (autonomous mobile robotics) platform equipped with a camera system, ultrasonic and photoelectric sensors, works with a high rate of precision and repeatability of its movements, as it uses stepper motors in its movement and navigation, the SoBot also can be termed as a research and development interface, as it facilitates the practical experimentation of algorithms from the simplest to the most complex level.

This product was developed 100% by Solis Tecnologia, and has a lot of technology employing cutting-edge concepts, such as:

The motors can be controlled simultaneously or individually.
The user can select different accessories to implement to the robot.
Several programming languages can be used to connect via API.

# Components

* Main structure in aluminum
* Robot Control Driver
* Raspberry Pi 4B board <img align="center" height="30" width="40" src="https://github.com/devicons/devicon/blob/master/icons/raspberrypi/raspberrypi-original.svg">
* 2x NEMA-23 Stepper Motors
* 2x 12V/5A battery
* USB control  <img align="center" height="40" width="40" src="https://github.com/SolisTecnologia/SoBot-USB-Control/blob/master/png/control.png">

# Programming Example
## Learning Mode - [LearningMode.py](https://github.com/SolisTecnologia/SoBot-Learning-Mode/blob/master/LearningMode.py)

For this programming, the SoBot was equipped with a camera and a Logitech F710 wireless controller. The robot is capable of recording and executing commands based on inputs received from the controller, allowing the movements to be replicated later. In addition, the SoBot's camera provides a real-time view of the path taken, which facilitates navigation and supervision of the robot in real time.

Controller button functions:  
>     BTN_X – Get the time for the delay command  
>     BTN_Y – Enables learning mode  
>     BTN_A – Executes the recorded command queue  
>     BTN_B – Saves the executed command in the queue  
>     BTN_START – Enables wheel motors  
>     BTN_R1 – Control the lift up  
>     BTN_R2 – Control the lift down  
>     BTN_L1 – Controls digital output 6  
>     BTN_L2 – Controls digital output 7  
>     BTN_UP – Moves the robot forward  
>     BTN_DOWN – Moves the robot backwards  
>     BTN_LEFT – Moves the robot to the left  
>     BTN_RIGTH – Moves the robot to the right  
  

### Programming Language

* Python  <img align="center" height="30" width="40" src="https://raw.githubusercontent.com/devicons/devicon/master/icons/python/python-original.svg">

### Required Libraries

~~~python
import multiprocessing
import evdev
from evdev import InputDevice, categorize, ecodes
from time import sleep, perf_counter
import serial
import serial.tools.list_ports
import cv2
import numpy as np
~~~

* **Multiprocessing:** To handle simultaneous processes, such as reading devices and executing commands.  
* **Evdev:** To capture events generated by the Logitech controller.  
* **Serial:** For serial communication between the SoBot and the Raspberry.  
* **Cv2 (OpenCV):** For capturing and processing video from the robot's integrated camera.  
* **Numpy:** For manipulating arrays.  

### Code Description

The programming was developed using the **Multiprocessing** library to execute different tasks in parallel, thus being able to create modular functions, where each function executes a specific task (reading the control, communicating with the robot, capturing video, etc.) and **events** (Event()) are used to synchronize the execution of tasks between processes and **queues** (Queue()) are also used to store commands and allow communication between processes.

* #### MAIN FUNCTION

Function developed to:
>* Find the serial device and initialize communication;
>* Create and initialize multiprocessing functions to execute tasks in parallel;
>* Create synchronization mechanisms for different processes (Event);
>* Create data structures that will be shared in a safe and organized manner (Queue);


* #### AUXILIARY FUNCTIONS

1. Status Value
~~~python
"""
Function for reading current status value
"""
def status_value (serial,ms_queue):
~~~

Function to read the current movement status of the SoBot. It processes the received command by evaluating the direction and movement based on status signals and returns the current value and movement flags.  

2. Video Capture
~~~python
"""
Function to capture video from the camera
"""
def VideoCapture ( ):
~~~

This function captures and displays continuous frames from the camera using OpenCV.  
The function enters a loop where the frames are captured and displayed in a window called "Original".  
The loop ends if the 'q' key is pressed, which closes the window.  
This function is useful for viewing in real time what the robot's camera is capturing.  

3. Read Gamepad
~~~python
"""
Function to read the remote control
"""
def Read_Gamepad(ev_Enable,ev_Learn_Mov,serialUSB,cmd_queue,ms_queue):
~~~

This is the main reading function of the gamepad (Logitech F710). It reads the input events from the remote control and performs the corresponding actions to control the robot, sending commands through the serial interface.

**`Controller Check:`**  
* Initially, the code lists all available input devices and checks if the controller is connected. If the controller is not found, it outputs a message.

~~~python
Name_Control = 'Logitech Gamepad F710'
device_path = ''

# Lists all available input device
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

for device in devices:
    # Checks if device name contains control name
    if Name_Control in device.name:
        print(f"Control Found: {device.name} in {device.path}")
        device_path = device.path
        break

    # Caso não encontre o controle
    else:
        print(f"Control '{Name_Control}' not found.")

gamepad = InputDevice(device_path)
~~~

**`Gamepad Events:`**  
* The code uses evdev to continuously read controller events in a loop (gamepad.read_loop()). It filters out type 1 (button presses) and type 3 (analog movement) events.

~~~python
#evdev takes care of polling the controller in a loop
for event in gamepad.read_loop():
    print(event)

    #filters by event type
    if event.type == 1 or event.type == 3:
~~~

**`Actions Based on Buttons:`**  
* When the Start button (BTN_START) is pressed, it switches the SoBot programming mode. If programming mode is active, the code activates the motors by sending the “MT0 ME1” and “CR1” commands to enable the return command of the executed actions and sets the “flag_start” to enable the identification of the other SoBot control buttons.

~~~python
# Check if the Start button is pressed
if event.code == BTN_START and event.value == 1:
    print(event)
    if flag_start == 0:
        print("PROGRAMMING ENABLED")
        flag_start = 1
        serialUSB.write(b"MT0 ME1")
        sleep(0.2)
        serialUSB.write(b"CR1")
        serialUSB.write(b"LT E1 RD0 GR0 BL70")
    else:
        print("PROGRAMMING DISABLED")
        flag_start = 0
        serialUSB.write(b"CR0")
        serialUSB.write(b"MT0 ME0")
        serialUSB.write(b"LT E0 RD0 GR0 BL0")
~~~

* Actions related to the direction buttons (left, right, up, and down) send commands to the robot to move in the specified directions. For example, the Down button sends the command “MT0 MB” to move the robot backward. To teach the turn command, the first command must be the direction of the turn, and then the forward and backward buttons to tell whether it is a forward or backward turn.  

* Other buttons, such as R1 and R2, send additional commands to control the elevator module.

* The code also implements a "learning mode" logic, where button actions are recorded in a command queue (cmd_queue), and these commands can be re-executed later.  
If learning mode is active (activated by the **Y** button), movement commands sent to the SoBot are recorded in a queue (cmd_queue) when pressing **button B** and executed later when pressing **button A**. If they are not movement commands, simply executing them will be saved in the command queue sequence.  
The code also handles timers to capture the time that certain buttons (such as X, R1 and R2) remain pressed, recording these times as command delay values.

4. Send Sequence
~~~python
"""
Function to send the commands stored in the queue to control the SoBot
"""
def send_seq(ev_rec_OK,ev_Enable,ev_Learn_Mov,serialUSB,cmd_queue):
~~~

The send_seq function is responsible for sending commands stored in the command queue to the SoBot in a continuous cycle until the queue is finished. It uses events to ensure that commands are sent and that responses are received correctly.  

5. Read Seria lUSB
~~~python
"""
Function to read commands received via the USB serial port
"""
def Read_SerialUSB(ev_rec_OK,serialUSB,ms_queue,ev_Learn_Mov):
~~~

The Read_SerialUSB function is responsible for reading data received from the serial port sent by SoBot. It checks if there is data available for reading, and, if there is, it processes the response by checking whether it is confirmation of the return command, triggering the ev_rec_ok event or whether it is a command related to the motor (MT0) and, if it is in active learning mode (ev_Learn_Mov), it adds the command to the ms_queue queue.  

6. Serial Device Finder
~~~python
"""
Function to find the serial port that the SoBot board is connected to
"""
def serial_device_finder (name_device):
~~~

This function finds the serial port that the SoBot device is connected to by comparing the device name with the descriptions of the ports available in the system.  




  
For more information about the commands used, check the Robot Commands Reference Guide.

  
  
  
# Reference Link
[SolisTecnologia website](https://solistecnologia.com/produtos/robotsingle)

# Please Contact Us
If you have any problem when using our robot after checking this tutorial, please contact us.

### Phone:
+55 1143040786

### Technical support email: 
contato@solistecnologia.com.br

![](https://github.com/SolisTecnologia/SoBot-USB-Control/blob/master/png/logo.png)