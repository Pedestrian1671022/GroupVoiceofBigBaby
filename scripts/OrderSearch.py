#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
import Group_Voice_of_BigBaby.msg
from geometry_msgs.msg import PoseStamped

param_request = '/param/request'
param_response = '/param/response'


class BTAction(object):
    _feedback = Group_Voice_of_BigBaby.msg.BTFeedback()
    _result = Group_Voice_of_BigBaby.msg.BTResult()

    def __init__(self, name):
        self.location = ''
        self.location_list = ['厨房', '卧室', '客厅']
        self.order_dict = {
            'fetch': ['给我拿瓶水', '给我拿杯水', '递我一瓶水', '递我一杯水', '拿瓶水', '拿杯水'],
            'move': ['去厨房', '去卧室', '去客厅', '来厨房', '来卧室', '来客厅'],
            'deliver': ['递我', '放手', '给我', '递我吧', '放手吧', '给我吧'],
            'place': ['放桌子上', '放桌上', '放那', '放那吧']
        }
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Group_Voice_of_BigBaby.msg.BTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        text_to_voice = ''
        if rospy.has_param(param_request):
            request = rospy.get_param(param_request)
            request = request.encode('utf8')
            order = self.order_search(request, self.order_dict)
            if order == 'fetch':
                text_to_voice = "好的，我这就去拿！"
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = 5.0
                pose_stamped.pose.orientation.w = 1.0
                # self.pub_location.publish(pose_stamped)

            elif order == "move":
                text_to_voice = "命令已收到，我这就去" + self.location + "！"
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.header.frame_id = "map"
                if self.location == '厨房':
                    pose_stamped.pose.position.x = 0.0
                    pose_stamped.pose.orientation.w = 1.0
                elif self.location == '卧室':
                    pose_stamped.pose.position.x = 0.0
                    pose_stamped.pose.orientation.w = 1.0
                else:
                    pose_stamped.pose.position.x = 0.0
                    pose_stamped.pose.orientation.w = 1.0
                    # self.pub_location.publish(pose_stamped)

            elif order == 'deliver':
                text_to_voice = "您的水，请拿好！"
                # self.pub_left_arm.publish(2)
                # self.pub_left_arm.publish(1)

            elif order == 'place':
                text_to_voice = '我这就把水放下！'

            else:
                pass

            if text_to_voice != "":
                rospy.delete_param(param_request)
                rospy.set_param(param_response, text_to_voice)
                print(text_to_voice)
                # self.pub_android.publish(text_to_voice + "!")
                self.set_status('SUCCESS')
            else:
                self.set_status('FAILURE')
        else:
            raise rospy.ROSInterruptException

    def order_search(self, data, dictionary):
        for (key, sentences) in dictionary.iteritems():
            for sentence in sentences:
                if data == sentence:
                    if key == 'fetch':
                        pass

                    if key == 'move':
                        for location in self.location_list:
                            if sentence.find(location) != -1:
                                self.location = location

                    if key == 'deliver':
                        pass

                    if key == 'place':
                        pass

                    return key

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
        rospy.init_node('OrderSearch')
        BTAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print('OrderSearch is over!')
