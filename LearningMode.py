#!/usr/bin/python3
"""
Solis Robot - SoBot

LearningMode.py: Programming example to teach SoBot
to move using the Logitech F710 controller.

Created By   : Vinicius M. Kawakami
Version      : 1.0

Company: Solis Tecnologia

Controller button functions:
    BTN_X – Get the time for the delay command
    BTN_Y – Enables learning mode
    BTN_A – Executes the recorded command queue
    BTN_B – Saves the executed command in the queue
    BTN_START – Enables wheel motors
    BTN_R1 – Control the lift up
    BTN_R2 – Control the lift down
    BTN_L1 – Controls digital output 6
    BTN_L2 – Controls digital output 7
    BTN_UP – Moves the robot forward
    BTN_DOWN – Moves the robot backwards
    BTN_LEFT – Moves the robot to the left
    BTN_RIGTH – Moves the robot to the right

"""

import multiprocessing
import evdev
from evdev import InputDevice, categorize, ecodes
from time import sleep, perf_counter
import serial
import serial.tools.list_ports
import cv2
import numpy as np

'''
###################################
        Global Variables
###################################
'''

# Variables for capturing video from the camera
camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

gamma = 35/100
lookUpTable = np.empty((1,256), np.uint8)

camera.set(3, 640)
camera.set(4, 360)
camera.set(5, 20)  #set frame
camera.set(cv2.CAP_PROP_BRIGHTNESS, 128)
camera.set(cv2.CAP_PROP_CONTRAST, 128)
camera.set(cv2.CAP_PROP_AUTO_WB, 1)
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

Name_device_serial = "VCOM"

'''
###################################
        Auxiliary Functions
###################################
'''

"""
Function for reading current status value
"""
def status_value (serial,ms_queue):
    data_seq = [0,0,0,0,0,0,0,0,0,0,0,0]
    flag_rd = 0
    flag_lf = 0
    flag_bk = 0
    current_value = 0

    print(str(serial.write(b"MT0 MS")))
    sleep(0.2)                              # Time to return the movement status value
    
    if ms_queue.qsize() >= 1:               # Check if there is a command in the queue
        data_seq  = ms_queue.get()          # if yes, add command to the queue

    print(data_seq)

    if data_seq[8] == 45:       # Check if it is a signal "-"
        flag_bk = 1
    else:
        flag_bk = 0
    data_seq = data_seq[9:14]
    print(data_seq)

    if data_seq[0] != 82 and data_seq[0] != 76:     # Checks if it is curved
        current_value = int(data_seq)
    else:                                         
        current_value = int(data_seq[1:5])
        if data_seq[0] == 82:       # 'R'
            flag_rd = 1
        elif data_seq[0] == 76:     # 'L'
            flag_lf = 1
    print(str(current_value))

    return current_value,flag_bk,flag_rd,flag_lf


"""
Function to capture video from the camera
"""
def VideoCapture ():
    while True:
    # Get frames from the image
        (grabbed, Frame) = camera.read()

        cv2.imshow("Original", Frame)

        buttonKey = cv2.waitKey(1)

        if buttonKey == ord('q'):
            break


"""
Function to read the remote control
"""
def Read_Gamepad(ev_Enable,ev_Learn_Mov,serialUSB,cmd_queue,ms_queue):

    flag_start = 0
    flag_learn = 0
    flag_l1 = 0
    flag_l2 = 0
    flag_left = 0
    flag_right = 0
    flag_fb = 0
    BTN_LEFT = 16
    BTN_RIGHT = 16
    BTN_UP = 17
    BTN_DOWN = 17
    BTN_START = 315
    BTN_A = 304
    BTN_X = 307
    BTN_Y = 308
    BTN_B = 305
    BTN_R1 = 311
    BTN_R2 = 5
    BTN_L1 = 310
    BTN_L2 = 2
    start_X = 0
    start_R1_time = 0
    start_R2_time = 0
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

    #evdev takes care of polling the controller in a loop
    for event in gamepad.read_loop():
        print(event)

        #filters by event type
        if event.type == 1 or event.type == 3:
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
                    # serialUSB.write(b"CR0")
                    serialUSB.write(b"MT0 ME0")
                    serialUSB.write(b"LT E0")

            if flag_start == 1:
                # Check if any button has been pressed
                if event.value == 1 or event.value == -1 or event.value == 255:
                    # Check the Left button
                    if(event.code == BTN_LEFT) and event.value == -1: 
                        if flag_right == 0 and flag_fb == 0:
                            flag_left = 1                          # Set flag of left button pressed
                            print("flag_left")

                    # Check the Right button
                    elif event.code == BTN_RIGHT and event.value == 1: 
                        if flag_left == 0 and flag_fb == 0:
                            flag_right = 1                          # Set flag of right button pressed
                            print("flag_rigth")

                    # Check the Down button
                    elif event.code == BTN_DOWN and event.value == 1: 
                        if flag_right == 0 and flag_left == 0:      # Checks that the curve button has not been pressed
                            flag_fb = 1
                            serialUSB.write(b"MT0 MB")              # Send the command back
                            print("MT0 MB - MOVING BACK")
                        elif flag_right:                            # If right turn
                            serialUSB.write(b"MT0 MR-")             # Send the command right back
                            print("MT0 MR- | MOVING RIGHT BACK")
                        elif flag_left:                             # If left turn
                            serialUSB.write(b"MT0 ML-")             # Send the command left back
                            print("MT0 ML- | MOVING LEFT BACK")

                    # Check the Up button
                    elif event.code == BTN_UP and event.value == -1: 
                        if flag_right == 0 and flag_left == 0:      # Checks that the curve button has not been pressed
                            flag_fb = 1
                            serialUSB.write(b"MT0 MF")              # Send the command forward
                            print("MT0 MF - MOVING FORWARD")
                        elif flag_right:                            # If right turn
                            serialUSB.write(b"MT0 MR")              # Send the command right forward
                            print("MT0 MR - MOVING RIGHT FORWARD")
                        elif flag_left:                             # If left turn
                            serialUSB.write(b"MT0 ML")              # Send the command left forward
                            print("MT0 ML - MOVING LEFT FORWARD")

                    # Check the Y button
                    if event.code == BTN_X:
                        start_X = perf_counter()        # Get the start time of the X button
                        print("BTN_X")
                    
                    # Check the R1 button
                    if event.code == BTN_R1:
                        serialUSB.write(b"EL UP")           # Send the command
                        start_R1_time = perf_counter()      # Get the start time of the R1 button
                        if ev_Learn_Mov.is_set():           # Checks if learning mode is active
                            cmd_queue.put("EL UP")          # Add the command to the queue

                    # Check the R2 button
                    if event.code == BTN_R2 and event.value == 255:
                        serialUSB.write(b"EL DN")           # Send the command 
                        start_R2_time = perf_counter()      # Get the start time of the R2 button
                        if ev_Learn_Mov.is_set():           # Checks if learning mode is active
                            cmd_queue.put("EL DN")          # Add the command to the queue
                    
                    # Check the L1 button
                    if event.code == BTN_L1:
                        if flag_l1 == 0:
                            flag_l1 = 1
                            serialUSB.write(b"DO6 E1")      # Send the command 
                            sleep(0.3)
                            if ev_Learn_Mov.is_set():       # If learning mode is active, saves commands in the queue
                                serialUSB.write(b"LT E1 RD50 GR70 BL0")
                                cmd_queue.put("DO6 E1")
                                cmd_queue.put("DL300")
                                sleep(0.2)
                                serialUSB.write(b"LT E1 RD0 GR70 BL0")
                        else:
                            flag_l1 = 0
                            serialUSB.write(b"DO6 E0")      # Send the command 
                            sleep(0.3)
                            if ev_Learn_Mov.is_set():       # If learning mode is active, saves commands in the queue
                                serialUSB.write(b"LT E1 RD50 GR50 BL0")
                                cmd_queue.put("DO6 E0")
                                cmd_queue.put("DL300")
                                sleep(0.2)
                                serialUSB.write(b"LT E1 RD0 GR70 BL0")
                    
                    # check the L2 button
                    if event.code == BTN_L2 and event.value == 255:
                        if flag_l2 == 0:
                            flag_l2 = 1
                            serialUSB.write(b"DO7 E1")      # Send the command
                            sleep(0.3)
                            if ev_Learn_Mov.is_set():       # If learning mode is active, saves commands in the queue
                                serialUSB.write(b"LT E1 RD50 GR50 BL0")
                                cmd_queue.put("DO7 E1")
                                cmd_queue.put("DL300")
                                sleep(0.2)
                                serialUSB.write(b"LT E1 RD0 GR70 BL0")
                        else:
                            flag_l2 = 0
                            serialUSB.write(b"DO7 E0")      # Send the command
                            sleep(0.3)
                            if ev_Learn_Mov.is_set():       # If learning mode is active, saves commands in the queue
                                serialUSB.write(b"LT E1 RD50 GR50 BL0")
                                cmd_queue.put("DO7 E0")
                                cmd_queue.put("DL300")
                                sleep(0.2)
                                serialUSB.write(b"LT E1 RD0 GR70 BL0")

                # Check if any button has been released
                if event.value == 0:
                    # check the A button
                    if event.code == BTN_A:
                        serialUSB.write(b"MT0 MC LM0")      # Send command to disable learning mode
                        ev_Learn_Mov.clear()
                        flag_learn = 0
                        ev_Enable.set()                     # Enable event to send saved commands
                        serialUSB.write(b"LT E1 RD0 GR0 BL70")
                    
                    # check the X button
                    if event.code == BTN_X:
                        finish_X = perf_counter()
                        X_time = "DL{}".format(int((finish_X - start_X)*1000))
                        print(X_time)
                        if ev_Learn_Mov.is_set():
                            serialUSB.write(b"LT E1 RD50 GR50 BL0")
                            cmd_queue.put(X_time)
                            sleep(0.2)
                            serialUSB.write(b"LT E1 RD0 GR70 BL0")

                    # check the Y button
                    if event.code == BTN_Y:
                        if flag_learn == 0:
                            flag_learn = 1
                            serialUSB.write(b"MT0 MC LM1")
                            ev_Learn_Mov.set()
                            serialUSB.write(b"LT E1 RD0 GR70 BL0")
                        else:
                            flag_learn = 0
                            serialUSB.write(b"MT0 MC LM0")
                            ev_Learn_Mov.clear()
                            serialUSB.write(b"LT E1 RD0 GR0 BL70")

                    # check the B button
                    if event.code == BTN_B:
                        if ev_Learn_Mov.is_set():       # If learning mode is active, get the status value of the current move
                            serialUSB.write(b"LT E1 RD50 GR50 BL0")
                            sleep(0.1)
                            flag_rd = 0
                            flag_lf = 0
                            flag_bk = 0
                            current_value = 0
                            flag_left = 0
                            flag_right = 0
                            flag_fb = 0
                            send_cmd_temp = ""

                            current_value,flag_bk,flag_rd,flag_lf = status_value(serialUSB,ms_queue)

                            if current_value == 0:
                                print("current_value = 0")
                                pass
                            else:                           # Formats the command string with the resulting value
                                if flag_lf or flag_rd:
                                    send_cmd_temp = "MT0 D{} DF RI80 V10"
                                else:
                                    send_cmd_temp = "MT0 D{} AT000 DT000 V10"
                                if flag_bk:
                                    current_value = -current_value
                                print(str(current_value))
                                send_cmd_temp = send_cmd_temp.format(current_value)
                                if flag_rd:
                                    flag_rd = 0
                                    send_cmd_temp = send_cmd_temp[:len(send_cmd_temp)] + " R"
                                if flag_lf:
                                    flag_lf = 0
                                    send_cmd_temp = send_cmd_temp[:len(send_cmd_temp)] + " L"
                                print(send_cmd_temp)

                                cmd_queue.put(send_cmd_temp)    # Add the configured status command to the list

                            serialUSB.write(b"LT E1 RD0 GR70 BL0")
                        else:
                            serialUSB.write(b"MT0 MS")
                            sleep(0.2)
                            flag_left = 0
                            flag_right = 0
                            flag_fb = 0

                    # check the R1 button
                    if event.code == BTN_R1:
                        serialUSB.write(b"EL ST")           # Send the command
                        finish_R1_time = perf_counter()     # Get the final time count value
                        R1_time = "DL{}".format(int((finish_R1_time - start_R1_time)*1000))     # Calculates and sets the delay time value
                        print(R1_time)
                        if ev_Learn_Mov.is_set():                       # If learning mode is active, saves commands in the queue
                            serialUSB.write(b"LT E1 RD50 GR50 BL0")
                            cmd_queue.put(R1_time)
                            cmd_queue.put("EL ST")
                            sleep(0.2)
                            serialUSB.write(b"LT E1 RD0 GR70 BL0")

                    # check the R2 button
                    if event.code == BTN_R2:
                        serialUSB.write(b"EL ST")           # Send the command
                        finish_R2_time = perf_counter()     # Get the final time count value
                        R2_time = "DL{}".format(int((finish_R2_time - start_R2_time)*1000))     # Calculates and sets the delay time value
                        print(R2_time)
                        if ev_Learn_Mov.is_set():                       # If learning mode is active, saves commands in the queue
                            serialUSB.write(b"LT E1 RD50 GR50 BL0")
                            cmd_queue.put(R2_time)
                            cmd_queue.put("EL ST")
                            sleep(0.2)
                            serialUSB.write(b"LT E1 RD0 GR70 BL0")

                    # If no direction button is pressed, send Pause Movement command one time
                    if event.code == BTN_LEFT or event.code == BTN_RIGHT or event.code == BTN_DOWN or event.code == BTN_UP:
                        serialUSB.write(b"MT0 MP")


"""
Function to send the commands stored in the queue to control the SoBot
"""
def send_seq(ev_rec_OK,ev_Enable,ev_Learn_Mov,serialUSB,cmd_queue):

    Seq_cmd = []
    cmd_temp = ""
    x = 0
    cmd_send = 0
    ctrl_learn = 0
    flag_MT0 = 0

    while 1:
        if ev_Enable.is_set():          # Check if the enable event was set by the control
            if cmd_send == 0:           # Check if the command has already been sent
                cmd_send = 1
                ctrl_learn = 0
                if x >= len(Seq_cmd):
                    ev_Enable.clear()
                    x = 0
                    cmd_send = 0
                    print(cmd_temp)
                    print("finish seq")
                else:
                    # Routine to identify the start and end of wheel movement to insert
                    # acceleration and deceleration time
                    if x < len(Seq_cmd):

                        if Seq_cmd[x].find("MT0") != -1:    # If MT0 command
                            print("find CMD MT0")
                            if(flag_MT0 == 0):              # if it is the frist command
                                flag_MT0 = 1                # add aceleration time
                                Seq_cmd[x] = Seq_cmd[x].replace("AT000", "AT400")
                                print("Include AT400")

                            if x == (len(Seq_cmd)-1):       # If it is the last command
                                if Seq_cmd[x].find("MT0") != -1:
                                    print("find last CMD MT0")
                                    flag_MT0 = 0                   # add deceleration time
                                    Seq_cmd[x] = Seq_cmd[x].replace("DT000", "DT400")

                            elif Seq_cmd[x+1].find("MT0") == -1:    # Or if the next command is not MT0
                                flag_MT0 = 0                        # add deceleration time
                                Seq_cmd[x] = Seq_cmd[x].replace("DT000", "DT400")
                                print("Include DT400")

                    print("send seq{}".format(x))
                    serialUSB.write(bytes(Seq_cmd[x], 'utf-8'))
                    cmd_temp = Seq_cmd[x]           # Saves the sent command in a temporary variable
                    x += 1
                    print(cmd_temp)

        if ev_rec_OK.is_set():      # Check if the return command was received in the serial read task
            cmd_send = 0            # Clear flag of sent command
            ev_rec_OK.clear()

        if ev_Learn_Mov.is_set():
            if ctrl_learn == 0:         # Check if this is the first time this functions is executing, if yes,
                ctrl_learn = 1          # clear the variable to start reading the queue
                Seq_cmd.clear()         

            if cmd_queue.qsize() >= 1:              # Check if there is a command in the queue
                Seq_cmd.append(cmd_queue.get())     # if yes, add the command to the list of commands to be executed


"""
Function to read commands received via the USB serial port
"""
def Read_SerialUSB(ev_rec_OK,serialUSB,ms_queue,ev_Learn_Mov):

    data = [0,0,0,0,0,0,0,0,0,0,0,0]

    while 1:
        if(serialUSB.in_waiting):
            data = serialUSB.readline()     # Reading received data
            if (data != b''):
                if data[0] == 67 and data[3] == 79 and data[4] == 75:     # Check if it is a return command "C" e "O" "k"
                    ev_rec_OK.set()                 # seta evento de comando de retorno OK

                if data[0] == 77 and data[1] == 84 and data[2] == 48:     # If it is motor return command "MT0"
                    if ev_Learn_Mov.is_set():       # If learning mode is active, saves commands in the queue
                        ms_queue.put(data)

                data = [0,0,0,0,0,0,0,0,0,0,0,0]


"""
Function to find the serial port that the SoBot board is connected to
"""
def serial_device_finder (name_device):
    # List all available serial ports
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        # Check if the port description contains the device name
        if name_device in port.description:
            print(f"Device found: {port.device} - {port.description}")
            return port.device

    # If can't find the device
    print(f"Device '{name_device}' not found.")
    return None


'''
###################################
        Main Function
###################################
'''

if __name__ == '__main__':
    
    serialUSB = serial.Serial()
    # Attempts to locate the device by name or description
    serial_device = serial_device_finder(Name_device_serial)

    if serial_device:
        # Connect to the found device
        serialUSB = serial.Serial(serial_device, baudrate=57600, timeout=0, dsrdtr=False)
        serialUSB.flush()
        print(f"Connected to device: {serial_device}")
    else:
        print("No device was connected.")

    #serialUSB.write(b"CR0")
    serialUSB.write(b"MT0 MC MD1 RI80 AT100 DT100 V8")  # Parameter settings for continuous mode
    serialUSB.write(b"LT CR0")

    ev_rec_OK = multiprocessing.Event()             # Event to indicate receipt of return command OK
    ev_Enable = multiprocessing.Event()             # Event to indicate that movement has been enabled 
    ev_Learn_Mov = multiprocessing.Event()          # Event for learning mode

    cmd_queue = multiprocessing.Queue()             # Creates a queue for receiving commands
    ms_queue = multiprocessing.Queue()              # Creates a queue for receiving movement status

    # Creates processes that will be executed in parallel
    app_Read_Gamepad = multiprocessing.Process(target=Read_Gamepad,args=(ev_Enable,ev_Learn_Mov,serialUSB,cmd_queue,ms_queue))
    app_Read_SerialUSB = multiprocessing.Process(target=Read_SerialUSB,args=(ev_rec_OK,serialUSB,ms_queue,ev_Learn_Mov))
    app_Send_Seq = multiprocessing.Process(target=send_seq,args=(ev_rec_OK,ev_Enable,ev_Learn_Mov,serialUSB,cmd_queue))
    app_VideoCapture = multiprocessing.Process(target=VideoCapture)

    # Start the processes  
    app_VideoCapture.start()
    app_Read_Gamepad.start()
    app_Read_SerialUSB.start()
    app_Send_Seq.start()


# Let CTRL+C actually exit
