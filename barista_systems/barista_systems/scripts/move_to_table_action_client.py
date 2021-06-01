#! /usr/bin/env python

import rospy
import time
import actionlib
from actionlib.msg import TestFeedback, TestResult, TestGoal, TestAction
"""
Test.action
int32 goal
---
int32 result
---
int32 feedback
"""

class MoveToTableActionClient(object):

    def __init__(self):
        # We create some constants with the corresponing vaules from the SimpleGoalState class
        self.PENDING = 0
        self.ACTIVE = 1
        self.DONE = 2
        self.WARN = 3
        self.ERROR = 4

        action_server_name = '/move_to_table_as'
        self.client = actionlib.SimpleActionClient(action_server_name, TestAction)
        # waits until the action server is up and running
        rospy.loginfo('Waiting for action Server ' + action_server_name)
        self.client.wait_for_server()
        rospy.loginfo('Action Server Found...' + action_server_name)

        # creates a goal to send to the action server
        self.goal = TestGoal()

    def feedback_callback(self, feedback):
        percentage_path_completed = feedback.feedback
        rospy.loginfo("[Feedback] Percentage path completed ==>"+str(percentage_path_completed))


    def go_to_table(self, table_number):
        self.goal.goal = table_number

        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        # You can access the SimpleAction Variable "simple_state", that will be 1 if active, and 2 when finished.
        # Its a variable, better use a function like get_state.
        # state = client.simple_state
        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = self.client.get_state()

        rate = rospy.Rate(1)

        rospy.loginfo("state_result: " + str(state_result))

        while state_result < self.DONE and not rospy.is_shutdown():
            rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
            rate.sleep()
            state_result = self.client.get_state()
            rospy.loginfo("state_result: " + str(state_result))

        rospy.loginfo("[Result] State: " + str(state_result))
        if state_result == self.ERROR:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == self.WARN:
            rospy.logwarn("There is a warning in the Server Side")

        rospy.loginfo("[Result] State: "+str(self.client.get_result()))

if __name__ == "__main__":

    # initializes the action client node
    rospy.init_node('example_no_waitforresult_action_client_node')
    object_client = MoveToTableActionClient()
    object_client.go_to_table(table_number=1)
    object_client.go_to_table(table_number=2)
    object_client.go_to_table(table_number=42)








