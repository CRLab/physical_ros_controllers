#!/usr/bin/env python

from __future__ import print_function

import pyaudio
import pygame
import struct
import math
import time
import sys
import rospy
import os
import rospkg

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


class BColors:
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


class Msg:
    NEXT_MSG = "2\n"
    SELECT_MSG = "3\n"
    WAITING_FOR_MSG = "4\n"


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

    def open_mic_stream(self):
        stream = self.pa.open(format=self.FORMAT,
                              channels=self.CHANNELS,
                              rate=self.RATE,
                              input=True,
                              frames_per_buffer=self.INPUT_FRAMES_PER_BLOCK)

        return stream

    def listen(self):
        block = None
        try:
            block = self.stream.read(self.INPUT_FRAMES_PER_BLOCK)
        except KeyboardInterrupt:
            exit(0)
        except IOError:
            return Tap.NOTHING

        amplitude = self.get_rms(block)

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

    def get_rms(self, block):
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
        return sign * math.sqrt(sum_squares / count)
        # return sum_squares/count


class AudioOutput(object):
    def __init__(self):
        pygame.init()
        package_path = rospkg.get_ros_package_path()
        self.waiting_file = os.path.join(package_path, 'resources', 'waiting.wav')
        self.next_file = os.path.join(package_path, 'resources', 'next.wav')
        self.select_file = os.path.join(package_path, 'resources', 'select.wav')
        self.reset_file = os.path.join(package_path, 'resources', 'reset.wav')
        self.sent_next_file = os.path.join(package_path, 'resources', 'sent_next.wav')
        self.sent_select_file = os.path.join(package_path, 'resources', 'sent_select.wav')

        self.previous_file = ''
        self.has_played_reset = False

    def play(self, filename):
        if not pygame.mixer.music.get_busy() and self.previous_file is not filename:
            pygame.mixer.music.load(filename)
            pygame.mixer.music.play(0)
            self.previous_file = filename

    def play_next(self):
        self.has_played_reset = False
        self.play(self.next_file)

    def play_select(self):
        self.has_played_reset = False
        self.play(self.select_file)

    def play_reset(self):
        self.has_played_reset = True
        self.previous_file = ""

    def play_waiting(self):
        self.previous_file = ""

    def play_sent_next(self):
        self.previous_file = ""

    def play_sent_select(self):
        self.previous_file = ""


class Communicator(object):
    WAIT_THRESHOLD = 0.1
    NEXT_THRESHOLD = 3
    SELECT_THRESHOLD = 7
    currentCommands = []
    currentlySelected = 0

    def __init__(self, audio_client):
        self.last_tap = time.time() - 100
        self.current_time = time.time()
        self.audio_client = audio_client

        self.client_socket = None

        self.publisher_execute_command = rospy.Publisher('/raw_execute_command', RawExecuteCommand, queue_size=10)
        self.publisher_currently_selected_command = rospy.Publisher('/currently_selected_command', String, queue_size=10)

        rospy.loginfo('waiting for service')
        rospy.wait_for_service('/valid_commands_service')
        valid_commands_service = rospy.ServiceProxy('/valid_commands_service', ValidCommandsService)
        resp = valid_commands_service()
        rospy.loginfo(resp)
        self.currentCommands = resp.commands

        self.subscriber_current_options = rospy.Subscriber("/valid_commands", ValidCommands, self.update_current_commands)

    def update_current_commands(self, current_commands_msg):
        self.currentCommands = current_commands_msg.commands
        rospy.loginfo("updating phrases")
        rospy.loginfo(self.currentCommands)
        self.publish_currently_selected_command()

    def iterate_current_option(self):
        self.currentlySelected = (self.currentlySelected + 1) % len(self.currentCommands)

    def publish_currently_selected_command(self):
        self.publisher_currently_selected_command.publish(self.currentCommands[self.currentlySelected])

    def publish_execute_command(self):
        msg = RawExecuteCommand()
        msg.input_source = "switch"
        msg.command = self.currentCommands[self.currentlySelected]

        self.publisher_execute_command.publish(msg)

    def handle_input(self, tap):
        if tap is Tap.PRESS:
            self.initiate_command()
        elif tap is Tap.RELEASE:
            self.execute_command()
        elif tap is Tap.COOLINGDOWN or tap is Tap.NOTHING:
            self.update_time()

    def update_time(self):
        self.current_time = time.time()

    def initiate_command(self):
        self.last_tap = time.time()
        self.update_time()

    def execute_command(self):
        state, x = self.read_state()
        if state is not Msg.WAITING_FOR_MSG:
            self.submit_message(state)
        return state

    def read_state(self):
        self.update_time()
        time_since_last_tap = self.current_time - self.last_tap

        if 0 <= time_since_last_tap <= self.NEXT_THRESHOLD:
            msg = Msg.NEXT_MSG
            # self.IterateCurrentOption()
            # self.publishCurrentlySelectedOption()
        elif self.NEXT_THRESHOLD < time_since_last_tap <= self.SELECT_THRESHOLD:
            msg = Msg.SELECT_MSG
            # self.publishExecuteOption()
        else:  # Took too long on tap
            msg = Msg.WAITING_FOR_MSG
        return msg, time_since_last_tap

    def sent_success(self, msg):
        if msg == Msg.NEXT_MSG:
            self.audio_client.play_sent_next()
        elif msg == Msg.SELECT_MSG:
            self.audio_client.play_sent_select()

    def submit_message(self, msg):
        self.reset_last_tap()
        if msg == Msg.NEXT_MSG:
            self.iterate_current_option()
            self.publish_currently_selected_command()
        elif msg == Msg.SELECT_MSG:
            self.publish_execute_command()

    def reset_last_tap(self):
        self.last_tap = time.time() - self.SELECT_THRESHOLD


class UserInterfaceFrame(Tk.Frame):
    BUFFER_MSG = "Buffering input (%fs passed)"
    WAITING_FOR_INPUT_MSG = (BColors.HEADER + "Waiting for input (%fs passed, %f amplitude)" + BColors.ENDC)
    REGISTERING_SELECT_MSG = (BColors.WARNING + "Registering select (%fs passed, %f amplitude)" + BColors.ENDC)
    REGISTERING_NEXT_MSG = (BColors.OKBLUE + "Registering next (%fs passed, %f amplitude)" + BColors.ENDC)
    COOLINGDOWN_MSG = (BColors.OKBLUE + "Cooling down (%fs passed, %f amplitude)" + BColors.ENDC)
    SUBMITTING_SELECT_MSG = BColors.OKGREEN + "\nSubmitting select" + BColors.ENDC
    SUBMITTING_NEXT_MSG = BColors.OKGREEN + "\nSubmitting next" + BColors.ENDC
    INITIAL_TAP_MSG = BColors.OKGREEN + "\nInitial tap" + BColors.ENDC

    def __init__(self, parent, communicator_client, listener_client, audio_client):
        Tk.Frame.__init__(self, parent)

        self.communicator = communicator_client
        self.listener = listener_client
        self.audio = audio_client
        self.size_str = 0

        button_frame = Tk.Frame(self)

        press_button = Tk.Button(button_frame, text="Press Switch", command=self.press_switch, font=36)
        release_button = Tk.Button(button_frame, text="Release Switch", command=self.release_switch, font=36)

        press_button.pack(side="top", fill="x")
        release_button.pack(side="top", fill="x")

        button_frame.pack(side="left", fill="y")

        self.current_status_label = Tk.Label(self, text='Waiting for User Input', font=36)
        self.current_status_label.pack(side="left", fill="both")

        self.info()

        self.manage_queue()

    def info(self):
        rospy.loginfo("This program controls Graspit! using an assistive controller with a 3.5mm jack input")
        rospy.loginfo(BColors.OKBLUE + "Blue color designates that the output is going to send a NEXT signal" + BColors.ENDC)
        rospy.loginfo(BColors.WARNING + "Yellow color designates that the output is going to send a SELECT signal" + BColors.ENDC)
        rospy.loginfo(BColors.HEADER + "Purple color designates that the output is waiting for user input" + BColors.ENDC)
        rospy.loginfo(BColors.OKGREEN + "Green color designates that a signal was sent" + BColors.ENDC)
        rospy.loginfo("Hold the switch for %0.2f to %0.2f second(s) if you want to send a NEXT signal" % (Communicator.WAIT_THRESHOLD, Communicator.NEXT_THRESHOLD))
        rospy.loginfo("Hold the switch for %0.2f to %0.2f second(s) if you want to send a SELECT signal" % (Communicator.NEXT_THRESHOLD, Communicator.SELECT_THRESHOLD))
        rospy.loginfo()

    def print_output(self, msg):
        output = "%s%s" % (msg, " " * (self.size_str - len(msg)))
        self.size_str = len(output)
        rospy.loginfo(output, end='\r')
        sys.stdout.flush()

    def press_switch(self):
        self.communicator.handle_input(Tap.PRESS)

    def release_switch(self):
        self.communicator.handle_input(Tap.RELEASE)

    def manage_queue(self):
        result, amplitude = self.listener.listen()
        self.communicator.handle_input(result)
        state, t = self.communicator.read_state()

        if result is Tap.COOLINGDOWN:
            self.current_status_label.config(text="Cooling down")
            self.print_output(self.COOLINGDOWN_MSG % (t, amplitude))
        elif state is Msg.NEXT_MSG:
            self.audio.play_next()
            self.current_status_label.config(text="Going to send NEXT")
            self.print_output(self.REGISTERING_NEXT_MSG % (t, amplitude))
        elif state is Msg.SELECT_MSG:
            self.audio.play_select()
            self.current_status_label.config(text="Going to send SELECT")
            self.print_output(self.REGISTERING_SELECT_MSG % (t, amplitude))
        else:
            self.audio.play_waiting()
            self.current_status_label.config(text="Waiting for user input")
            self.print_output(self.WAITING_FOR_INPUT_MSG % (t, amplitude))

        # repeat again in 1 millisecond
        self.after(1, self.manage_queue)


def main():
    rospy.init_node('switch_driver')

    audio = AudioOutput()
    communicator = Communicator(audio)
    listener = TapDetector()
    root = Tk.Tk()
    root.title("Switch Controller")
    UserInterfaceFrame(root, communicator, listener, audio).pack(fill="both", expand=True)
    root.mainloop()

if __name__ == "__main__":
    main()
