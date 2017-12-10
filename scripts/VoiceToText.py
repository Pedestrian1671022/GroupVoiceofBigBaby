#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
import Group_Voice_of_BigBaby.msg

import os
import json
import uuid
import base64
import urllib2
import requests
param_answer = '/param/answer'
param_request = '/param/request'
param_interrupt = '/param/interrupt'
param_play = '/param/play'

path_prefix = os.path.abspath('..')
capturedVoice = path_prefix + '/catkin_ws/src/Group_Voice_of_BigBaby/res/wav/capturedVoice.wav'


class BTAction(object):
    _feedback = Group_Voice_of_BigBaby.msg.BTFeedback()
    _result = Group_Voice_of_BigBaby.msg.BTResult()

    def __init__(self, name):
        self.recognized_result = ''
        self.voice_to_text_url = 'http://vop.baidu.com/server_api'
        self.apiKey = '1GQyi2TtlQc1xmkAkiaHzNtL'
        self.secretKey = 'mcdf5t6QZWNHzGbwFkhjm5nT3KuKrMyf'
        self.auth_url = 'https://openapi.baidu.com/oauth/2.0/token?grant_type=client_credentials&client_id=' \
                        + self.apiKey + '&client_secret=' + self.secretKey
        self.access_token = self.get_token()
        self.uuid = uuid.UUID(int=uuid.getnode()).hex[-12:]

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Group_Voice_of_BigBaby.msg.BTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        wav_fp = open(capturedVoice, 'rb')
        voice_data = wav_fp.read()
        data = {'format': 'wav', 'rate': 16000, 'channel': 1, 'cuid': self.uuid, 'token': self.access_token,
                'lan': 'zh', 'len': len(voice_data), 'speech': base64.b64encode(voice_data).decode('utf-8')}

        result = requests.post(self.voice_to_text_url, data=json.dumps(data),
                               headers={'Content-Type': 'application/json'}, stream=False)
        data_result = result.json()
        if data_result['err_no'] == 0:
            request = data_result['result'][0]
            request = request[:-1]
            if request == '':
                self.set_status('FAILURE')
            rospy.set_param(param_request, request)
            print request
            if rospy.get_param(param_play):
                rospy.set_param(param_interrupt, True)
            rospy.set_param(param_answer, True)
            self.set_status('SUCCESS')
        else:
            self.set_status('FAILURE')

    def get_token(self):
        res = urllib2.urlopen(self.auth_url)
        json_data = res.read()
        return json.loads(json_data)['access_token']

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
        rospy.init_node('VoiceToText')
        BTAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print('VoiceToText is over!')
