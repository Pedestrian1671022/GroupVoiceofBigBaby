#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
import Group_Voice_of_BigBaby.msg

import os
import wave
import numpy as np
from pyaudio import PyAudio, paInt16

path_prefix = os.path.abspath('..')
capturedVoice = path_prefix + '/catkin_ws/src/Group_Voice_of_BigBaby/res/wav/capturedVoice.wav'


class BTAction(object):
    _feedback = Group_Voice_of_BigBaby.msg.BTFeedback()
    _result = Group_Voice_of_BigBaby.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Group_Voice_of_BigBaby.msg.BTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        num_samples = 2000
        sampling_rate = 16000
        level = 2000
        count_num = 15
        save_length = 4
        min_length = 7
        save_count = 0
        save_buffer = []
        length = 0
        dev_to_capture = PyAudio()
        stream = dev_to_capture.open(format=paInt16, channels=1, rate=sampling_rate,
                                     input=True, frames_per_buffer=num_samples)

        while not self._as.is_preempt_requested():
            string_audio_data = stream.read(num_samples)
            audio_data = np.fromstring(string_audio_data, dtype=np.short)
            large_sample_count = np.sum(audio_data > level)

            if large_sample_count > count_num:
                save_count = save_length
            else:
                save_count -= 1

            if save_count > 0:
                save_buffer.append(string_audio_data)
                length += 1

            else:
                if length > min_length:
                    voice_string = save_buffer
                    wf = wave.open(capturedVoice, 'wb')
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(sampling_rate)
                    wf.writeframes(np.array(voice_string).tostring())
                    wf.close()
                    stream.stop_stream()
                    stream.close()
                    dev_to_capture.terminate()
                    self.set_status('SUCCESS')
                    break
                    print('Recorded a piece of voice successfully!')
                else:
                    save_count = 0
                    length = 0
                save_buffer = []

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
        rospy.init_node('VoiceCapture')
        BTAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print('VoiceCapture is over!')
