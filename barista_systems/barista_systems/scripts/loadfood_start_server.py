#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool,SetBoolRequest, SetBoolResponse


class LoadFoodServer(object):

    def __init__(self):
        s = rospy.Service('/loadfood_server', SetBool, self._handle_foodload)

        calibrate_server_name = 'load_sensor_calibrate_server'
        rospy.logwarn("Waiting for Service="+str(calibrate_server_name))
        rospy.wait_for_service(calibrate_server_name)
        rospy.logwarn("Ready Service=" + str(calibrate_server_name))
        self._calibrate_load_sensor_client = rospy.ServiceProxy(calibrate_server_name, SetBool)
        rospy.loginfo("Server READY for Food Loading")

    def _handle_foodload(self, req):
        rospy.logdebug("Request of LoadFood Recieved..Processing")
        calibration_req = SetBoolRequest()
        calibration_req.data = req.data
        cal_response = self._calibrate_load_sensor_client(calibration_req)

        rospy.logdebug("Calibration complete!")
        response = SetBoolResponse()
        response.success = cal_response.success
        response.message = cal_response.message
        return response





def loadfood_server():
    rospy.init_node('loadfood_server_node', log_level=rospy.DEBUG)
    LoadFoodServer()
    rospy.spin()

if __name__ == "__main__":
    loadfood_server()