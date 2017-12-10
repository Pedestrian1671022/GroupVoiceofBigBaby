#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
import Group_Voice_of_BigBaby.msg

import json
import urllib

param_request = '/param/request'
param_response = '/param/response'


class BTAction(object):
    _feedback = Group_Voice_of_BigBaby.msg.BTFeedback()
    _result = Group_Voice_of_BigBaby.msg.BTResult()

    def __init__(self, name):
        self.key = 'a9c913f6e8f146a19ee8b40eca9cee03'
        self.user_id = '102043'
        self.loc = '辽宁省沈阳市'
        self.request = 'http://www.tuling123.com/openapi/api?key=' + self.key \
                       + '&loc=' + self.loc + '&userid=' + self.user_id + '&info='

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Group_Voice_of_BigBaby.msg.BTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        if rospy.has_param(param_request):
            request = rospy.get_param(param_request)
            rospy.delete_param(param_request)
            url = self.request + request.encode('utf8')
            response = urllib.urlopen(url)
            text_str = response.read()
            text_json = json.loads(text_str)
            response_text = text_json["text"]
            print response_text
            if response_text != '':
                rospy.set_param(param_response, response_text)
                self.set_status('SUCCESS')
            else:
                self.set_status('FAILURE')
        else:
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
        rospy.init_node('TextToText')
        BTAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print('TextToText is over!')
