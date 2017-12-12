#!/usr/bin/python

from __future__ import print_function

import pyaudio
import pygame
import struct
import math
import time
import sys
import socket
import rospy

import Tkinter as Tk
from std_msgs.msg import String
from external_controller_msgs.msg import ValidCommands
from external_controller_msgs.msg import RawExecuteCommand
from external_controller_msgs.srv import ValidCommandsService


"""
Obtained most source from http://stackoverflow.com/questions/4160175/detect-tap-with-pyaudio-from-live-mic
or http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
open a microphone in pyAudio and listen for taps
"""


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Tap:
    PRESS = 1
    RELEASE = 2
    COOLINGDOWN = 3
    NOTHING = 4

    def __init__(self):
        pass


class Msg:
    NEXT_MSG = "2\n"
    SELECT_MSG = "3\n"
    WAITING_FOR_MSG = "4\n"

    def __init__(self):
        pass


class TapDetector(object):
    TAP_THRESHOLD = 0.5
    FORMAT = pyaudio.paInt16 
    SHORT_NORMALIZE = (1.0/32768.0)
    RATE = 16000  
    INPUT_BLOCK_TIME = 0.01
    INPUT_FRAMES_PER_BLOCK = int(RATE*INPUT_BLOCK_TIME)             
    CHANNELS = 2
    MAX_TAP_BLOCKS = 0.15/INPUT_BLOCK_TIME

    def __init__(self):
        self.pa = pyaudio.PyAudio()
        self.stream = self.open_mic_stream()
        self.has_cooled_down = True

    def stop(self):
        self.stream.close()

    def open_mic_stream( self ):
        stream = self.pa.open(format=self.FORMAT,
                              channels=self.CHANNELS,
                              rate=self.RATE,
                              input=True,
                              frames_per_buffer=self.INPUT_FRAMES_PER_BLOCK)

        return stream

    def listen(self):
        try:
            block = self.stream.read(self.INPUT_FRAMES_PER_BLOCK)
        except KeyboardInterrupt:
            exit(0)
        except IOError:
            return Tap.NOTHING

        amplitude = self.get_rms( block )

        result = Tap.RELEASE if amplitude > self.TAP_THRESHOLD \
            else Tap.PRESS if amplitude < -self.TAP_THRESHOLD \
            else Tap.NOTHING

        if result is Tap.NOTHING:
            self.has_cooled_down = True
            return result, amplitude

        elif self.has_cooled_down:
            self.has_cooled_down = False
            return result, amplitude
        else:
            return Tap.COOLINGDOWN, amplitude

        # return result, amplitude

    def get_rms( self, block ):
        # we will get one short out for each 
        # two chars in the string.
        count = len(block)/2
        rms_format = "%dh" % count
        shorts = struct.unpack(rms_format, block)

        # iterate over the block.
        sum_squares = 0.0
        num_pos = 0
        for sample in shorts:
            # sample is a signed short in +/- 32768. 
            # normalize it to 1.0
            n = sample * self.SHORT_NORMALIZE
            if n > 0:
                num_pos += 1
            sum_squares += n * n

        sign = -1
        if num_pos > count/2:
            sign = 1 
        return sign * math.sqrt( sum_squares / count )
        # return sum_squares/count


class AudioOutput(object):
    def __init__(self):
        pygame.init()
        self.waiting_file = 'waiting.wav'
        self.next_file = 'next.wav'
        self.select_file = 'select.wav'
        self.reset_file = 'reset.wav'
        self.sent_next_file = 'sent_next.wav'
        self.sent_select_file = 'sent_select.wav'

        self.previous_file = ''
        self.has_played_reset = False

    def play(self, filename):
        return
        # if not pygame.mixer.music.get_busy() and self.previous_file is not filename:
        #     pygame.mixer.music.load(filename)
        #     pygame.mixer.music.play(0)
        #     self.previous_file = filename

    def play_next(self):
        self.has_played_reset = False
        self.play(self.next_file)

    def playSelect(self):
        self.has_played_reset = False
        self.play(self.select_file)

    def playReset(self):
        self.has_played_reset = True

    def playWaiting(self):
        pass

    def playSentNext(self):
        pass

    def playSentSelect(self):
        pass

###Beginning of rospy code

class Communicator(object):
    WAIT_THRESHOLD = 0.1
    NEXT_THRESHOLD = 3
    SELECT_THRESHOLD = 7
    currentCommands = []
    currentlySelected = 0

    def __init__(self, audio):
        self.last_tap = time.time() - 100
        self.current_time = time.time()
        self.audio = audio

        self.client_socket = None
        self.init_client_socket()



        self.publisher_execute_command = rospy.Publisher('/raw_execute_command', RawExecuteCommand, queue_size=10)
        self.publisher_currently_selected_command = rospy.Publisher('/currently_selected_command', String, queue_size=10)

        rospy.wait_for_service('valid_commands_service')
        valid_commands_service = rospy.ServiceProxy('valid_commands_service', ValidCommandsService)
        resp = valid_commands_service('');
        print(resp)
        currentCommands = resp.commands;

        rospy.init_node('switch_driver', anonymous=True)

        # self.publisher_currently_selected_option = self.createPublisher('/CurrentlySelectedOption', 'String')
        # self.publisher_execute_option = self.createPublisher('/ExecuteOption', 'ExecuteOption')

        self.subscriber_current_options = rospy.Subscriber("/valid_commands", ValidCommands, self.updateCurrentCommands)


    #rospy stuff
    def updateCurrentCommands(self, CurrentCommandsMsg):
        self.currentCommands = CurrentCommandsMsg.commands
        print("updating phrases")
        print(self.currentCommands)
        self.publishCurrentlySelectedCommand()

    def IterateCurrentOption(self):
        self.currentlySelected += 1
        if (self.currentlySelected == len(self.currentCommands)):
            self.currentlySelected = 0

    def publishCurrentlySelectedCommand(self):
        self.publisher_currently_selected_command.publish(self.currentCommands[self.currentlySelected])

    def publishExecuteCommand(self):
        self.msg = RawExecuteCommand()
        self.msg.input_source = "switch"
        self.msg.command = self.currentCommands[self.currentlySelected]

        self.publisher_execute_command.publish(self.msg)

        # self.publisher_execute_option.publish(self.currentOptions[self.currentlySelected])
    #
    # def createPublisher(self, topic, message_type):
    #     pub = rospy.Publisher(topic, message_type, queue_size=10) # broken currently with message_type
    #     rospy.init_node('switch_driver', anonymous=True)
    #     # rate = rospy.Rate(10) # 10hz
    #     return pub

    #END OF ROSPY CODE

    def init_client_socket(self, ip='localhost', port=4775):
        return True
        # try:
        #     self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #     self.client_socket.connect((ip, port))
        #     return True
        # except(socket.timeout, IOError):
        #     print("Could not connect to server")
        #     return False

    def handleInput(self, tap):
        if tap is Tap.PRESS:
            self.initiateCommand()
        elif tap is Tap.RELEASE:
            self.executeCommand()
        elif tap is Tap.COOLINGDOWN or tap is Tap.NOTHING:
            self.updateTime()

    def updateTime(self):
        self.current_time = time.time()

    def initiateCommand(self):
        self.last_tap = time.time()
        self.updateTime()

    def executeCommand(self):
        state, x = self.readState()
        if state is not Msg.WAITING_FOR_MSG:
            self.submitMessage(state)
        return state

    def readState(self):
        self.updateTime()
        time_since_last_tap = self.current_time - self.last_tap

        if 0 <= time_since_last_tap <= self.NEXT_THRESHOLD:
            msg = Msg.NEXT_MSG
            # self.IterateCurrentOption()
            # self.publishCurrentlySelectedOption()
        elif self.NEXT_THRESHOLD < time_since_last_tap <= self.SELECT_THRESHOLD:
            msg = Msg.SELECT_MSG
            # self.publishExecuteOption()
        else: # Took too long on tap
            msg = Msg.WAITING_FOR_MSG
        return msg, time_since_last_tap

    def sent_success(self, msg):
        if msg == Msg.NEXT_MSG:
            self.audio.playSentNext()
        elif msg == Msg.SELECT_MSG:
            self.audio.playSentSelect()

    def submitMessage(self, msg):
        self.resetLastTap()
        if msg == Msg.NEXT_MSG:
            self.IterateCurrentOption()
            self.publishCurrentlySelectedCommand()
        elif msg == Msg.SELECT_MSG:
            self.publishExecuteCommand()
        # try:
        #     self.client_socket.send(msg.encode())
        #     self.sent_success(msg)
        # except:
        #     connected = self.init_client_socket()
        #     if connected:
        #         self.client_socket.send(msg)
        #         self.sent_success(msg)
        #     else:
        #         print("Failed to send msg: %s" % msg)

    def resetLastTap(self):
        self.last_tap = time.time() - self.SELECT_THRESHOLD

class UserInterfaceFrame(Tk.Frame):
    BUFFER_MSG = "Buffering input (%fs passed)"
    WAITING_FOR_INPUT_MSG = (bcolors.HEADER + "Waiting for input (%fs passed, %f amplitude)" + bcolors.ENDC)
    REGISTERING_SELECT_MSG = (bcolors.WARNING + "Registering select (%fs passed, %f amplitude)" + bcolors.ENDC)
    REGISTERING_NEXT_MSG = (bcolors.OKBLUE + "Registering next (%fs passed, %f amplitude)" + bcolors.ENDC)
    COOLINGDOWN_MSG = (bcolors.OKBLUE + "Cooling down (%fs passed, %f amplitude)" + bcolors.ENDC)
    SUBMITTING_SELECT_MSG = bcolors.OKGREEN + "\nSubmitting select" + bcolors.ENDC
    SUBMITTING_NEXT_MSG = bcolors.OKGREEN + "\nSubmitting next" + bcolors.ENDC
    INITIAL_TAP_MSG = bcolors.OKGREEN + "\nInitial tap" + bcolors.ENDC

    def __init__(self, parent, communicator, listener, audio):
        Tk.Frame.__init__(self, parent)

        self.communicator = communicator
        self.listener = listener
        self.audio = audio
        self.size_str = 0

        buttonFrame = Tk.Frame(self)

        pressButton = Tk.Button(buttonFrame, text="Press Switch", command=self.pressSwitch, font=36)
        releaseButton = Tk.Button(buttonFrame, text="Release Switch", command=self.releaseSwitch, font=36)

        pressButton.pack(side="top", fill="x")
        releaseButton.pack(side="top", fill="x")

        buttonFrame.pack(side="left", fill="y")

        self.current_status_label = Tk.Label(self, text='Waiting for User Input', font=36)
        self.current_status_label.pack(side="left", fill="both")

        self.info()

        self.manage_queue()

    def info(self):
        print("This program controls Graspit! using an assistive controller with a 3.5mm jack input")
        print(bcolors.OKBLUE + "Blue color designates that the output is going to send a NEXT signal" + bcolors.ENDC)
        print(bcolors.WARNING + "Yellow color designates that the output is going to send a SELECT signal" + bcolors.ENDC)
        print(bcolors.HEADER + "Purple color designates that the output is waiting for user input" + bcolors.ENDC)
        print(bcolors.OKGREEN + "Green color designates that a signal was sent" + bcolors.ENDC)
        print("Hold the switch for %0.2f to %0.2f second(s) if you want to send a NEXT signal" % (Communicator.WAIT_THRESHOLD, Communicator.NEXT_THRESHOLD))
        print("Hold the switch for %0.2f to %0.2f second(s) if you want to send a SELECT signal" % (Communicator.NEXT_THRESHOLD, Communicator.SELECT_THRESHOLD))
        print()

    def print_output(self, msg):
        output = "%s%s" % (msg, " " * (self.size_str - len(msg)))
        self.size_str = len(output)
        print(output, end='\r')
        sys.stdout.flush()

    def pressSwitch(self):
        communicator.handleInput(Tap.PRESS)

    def releaseSwitch(self):
        val = communicator.handleInput(Tap.RELEASE)
        if val is Msg.NEXT_MSG:
            self.print_output(self.SUBMITTING_NEXT_MSG)
        elif val is Msg.SELECT_MSG:
            self.print_output(self.SUBMITTING_SELECT_MSG)

    def manage_queue(self):
        result, amplitude = self.listener.listen()
        self.communicator.handleInput(result)
        state, t = self.communicator.readState()

        if result is Tap.COOLINGDOWN:
            self.current_status_label.config(text="Cooling down")
            self.print_output(self.COOLINGDOWN_MSG % (t, amplitude))
        elif state is Msg.NEXT_MSG:
            self.audio.play_next()
            self.current_status_label.config(text="Going to send NEXT")
            self.print_output(self.REGISTERING_NEXT_MSG % (t, amplitude))
        elif state is Msg.SELECT_MSG:
            self.audio.playSelect()
            self.current_status_label.config(text="Going to send SELECT")
            self.print_output(self.REGISTERING_SELECT_MSG % (t, amplitude))
        else:
            self.audio.playWaiting()
            self.current_status_label.config(text="Waiting for user input")
            self.print_output(self.WAITING_FOR_INPUT_MSG % (t, amplitude))

        # repeat again in 1 millisecond
        self.after(1, self.manage_queue)

if __name__ == "__main__":
    audio = AudioOutput()
    communicator = Communicator(audio)
    listener = TapDetector()
    root = Tk.Tk()
    root.title("Switch Controller")
    UserInterfaceFrame(root, communicator, listener, audio).pack(fill="both", expand=True)
    root.mainloop()
