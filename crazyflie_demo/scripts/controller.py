#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from crazyflie.srv import UpdateParams
from std_srvs.srv import Empty

class Controller():
    def __init__(self):
        self._buttons = None
        rospy.Subscriber("joy", Joy, self._joyChanged)

        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1:
                    value = bool(int(rospy.get_param("ring/headlightEnable")))
                    print(value)
                    rospy.set_param("ring/headlightEnable", not value)
                    self._update_params(["ring/headlightEnable"])
                    print(not value)
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    controller = Controller()
    rospy.spin()
