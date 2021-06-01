#! /usr/bin/env python
import rospy
import time
import actionlib
from actionlib.msg import TestFeedback, TestResult, TestAction

class MoveToTableActionServer(object):


    def __init__(self):
        # creates the action server
        # create messages that are used to publish feedback/result
        self._feedback = TestFeedback()
        self._result = TestResult()
        self._as = actionlib.SimpleActionServer("/move_to_table_as", TestAction, self.goal_callback, False)
        self._as.start()

    def goal_callback(self, goal):
        # this callback is called when the action server is called.

        # helper variables
        r = rospy.Rate(1)
        success = True

        # TODO: We simulate the perfentage of the path done
        percentage_path_done = 0
        for i in range(101):
            rospy.loginfo("Moving To Table=>>>"+str(goal.goal))
            # check that preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('The goal has been cancelled/preempted')
                # the following line, sets the client in preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the calculation of the Fibonacci sequence
                break

            # TODO: Dummy Wait to simulate doing something
            time.sleep(0.1)

            # build and publish the feedback message
            self._feedback.feedback = i
            self._as.publish_feedback(self._feedback)

            percentage_path_done = i

        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If success, then we publish the final result
        # If not success, we do not publish anything in the result
        if success:
            self._result.result = percentage_path_done
            rospy.loginfo('Percentage Path Done %i' % self._result.result)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('move_to_table_action_server_node')
    MoveToTableActionServer()
    rospy.spin()