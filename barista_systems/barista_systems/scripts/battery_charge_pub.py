#!/usr/bin/env python

from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rospy


class BatteryCharge(object):

    def __init__(self):

        self.diagnostics_topic = "/diagnostics"
        self.batterycharge_topic = "/battery_charge"

        self.check_diagnostics_ready()
        rospy.Subscriber(self.diagnostics_topic, DiagnosticArray, self.callback)

        self._pub = rospy.Publisher(self.batterycharge_topic, Float32, queue_size=1)
        self._check_battery_charge_publisher_connection()

    def _check_battery_charge_publisher_connection(self):
        """
        Checks that battery charge pub is working
        :return:
        """
        rate = rospy.Rate(10)
        while self._pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No subscribers to "+self.batterycharge_topic+" yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug( self.batterycharge_topic+"Publisher Connected")

    def get_battery_percentage(self, diagnostics_info):
        """
        We retrieve form the diagnostics the percent of the battery
        :param diagnostics_info:
        :return:
        """
        # Fake value
        percent = 0.0

        for info_msg in diagnostics_info.status:
            if info_msg.name == "mobile_base_nodelet_manager: Battery":
                for value in info_msg.values:
                    if value.key == "Percent":
                        percent = float(value.value)

        return percent

    def check_diagnostics_ready(self):
        diagnostics_data = None
        while diagnostics_data is None and not rospy.is_shutdown():
            try:
                diagnostics_data = rospy.wait_for_message(self.diagnostics_topic, DiagnosticArray, timeout=1.0)
                rospy.loginfo("Current "+self.diagnostics_topic+" READY=>" + str(diagnostics_data))
            except:
                rospy.logdebug("Current "+self.diagnostics_topic+" not ready yet, retrying...")

        rospy.logdebug("Current " + self.diagnostics_topic + " READY!")


    def callback(self, msg):

        battery_percent = self.get_battery_percentage(msg)
        rospy.logdebug("current_battery_percent="+str(battery_percent))
        self.publish_load_info(battery_percent)


    def publish_load_info(self, battery_percent):
        """
        Publishes if we picked an object and its weight.
        It also evaluates if the object has been lost or
        something extra was added.
        Its very important that it averages reading because by movements and
        so on the weight may vary a lot.
        :return:
        """

        battery_data = Float32()
        battery_data.data = battery_percent
        self._pub.publish(battery_data)

def main():
    rospy.init_node('battery_charge_node', log_level=rospy.WARN)
    load_sensor_obj = BatteryCharge()
    rospy.spin()

if __name__ == "__main__":
    main()