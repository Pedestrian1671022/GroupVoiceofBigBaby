#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
import Group_Voice_of_BigBaby.msg

import os
import wave
import pyaudio

param_interrupt = '/param/interrupt'
param_play = '/param/play'

path_prefix = os.path.abspath('..')
synthesizedVoice = path_prefix + '/catkin_ws/src/Group_Voice_of_BigBaby/res/wav/synthesizedVoice.wav'


class BTAction(object):
    _feedback = Group_Voice_of_BigBaby.msg.BTFeedback()
    _result = Group_Voice_of_BigBaby.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Group_Voice_of_BigBaby.msg.BTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        try:
            rospy.set_param(param_play, True)
            wave_file = wave.open(synthesizedVoice, 'rb')
            dev_to_play = pyaudio.PyAudio()
            stream = dev_to_play.open(format=dev_to_play.get_format_from_width(wave_file.getsampwidth()),
                                      channels=wave_file.getnchannels(), rate=wave_file.getframerate(), output=True)
            while not rospy.get_param(param_interrupt):
                data = wave_file.readframes(500)
                if data == '':
                    break
                stream.write(data)
            rospy.set_param(param_play, False)
            rospy.set_param(param_interrupt, False)
            wave_file.close()
            stream.stop_stream()
            stream.close()
            dev_to_play.terminate()
            self.set_status('SUCCESS')
        except Exception:
            self.set_status('FAILURE')
            raise rospy.ROSInterruptException

    def set_status(self, status):
        if status == 'SUCCESS':
            self._feedback.status = 1
            self._result.status = self._feedback.status
            self._as.set_succeeded(self._result)
        else:
            self._feedback.status = 2
            self._result.status = self._feedback.status
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    try:
        rospy.init_node('VoicePlay')
        BTAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print('VoicePlay is over!')
