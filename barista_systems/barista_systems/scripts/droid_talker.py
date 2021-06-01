#!/usr/bin/env python

import rospy
from droidspeak.msg import DroidChat
from std_msgs.msg import UInt8, String
from droidspeak.srv import DroidEmotionRequest

letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z']


class DroidTalker(object):

    def __init__(self):

        self._emotions_speed_dict = {"NORMAL": 0.325,
                                     "ANGRY": 0.01,
                                     "SORRY": 0.8}
        self.current_emotion = self._emotions_speed_dict["NORMAL"]

        self._speak_pub = rospy.Publisher('speak', DroidChat, queue_size=10)
        self._droid_talker_emotion_sub = rospy.Subscriber('droid_talker_emotion',
                                                          UInt8,
                                                          self._droid_talker_emotion_clk)
        self._droid_talker_msg_sub = rospy.Subscriber('droid_talker_msg',
                                                      String,
                                                      self._droid_talker_msg_clk)

        self.init_droid_chat_message()

    def init_droid_chat_message(self):

        self._droid_chat_message = DroidChat()
        # Frase to say
        self._droid_chat_message.data = "ready"
        for i in self._droid_chat_message.data:
            self._droid_chat_message.spaces.append(abs(self.current_emotion))

        self.talk()

    def talk(self):
        """
        Says what has been saved in self._droid_chat_message
        :return:
        """
        self._check_speak_publisher_connection()
        self._speak_pub.publish(self._droid_chat_message)

    def _check_speak_publisher_connection(self):
        """
        Checks that speak the publisher is working
        :return:
        """
        rate = rospy.Rate(10)
        while self._speak_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No subscribers to /speak yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("speak_pub Publisher Connected")




    def _droid_talker_emotion_clk(self, data):
        """
        Based loosely in emotions defined in DroidEmotion.srv from droidspeak package:
        uint8 NORMAL=0, this will triguer normal speed speaking.
        uint8 SORRY=2, this will trigger slower speaking.
        uint8 ANGRY=3, this will trigger faster speaking.
        :param data:
        :return:
        """
        if data.data == DroidEmotionRequest.NORMAL:
            self.current_emotion = self._emotions_speed_dict["NORMAL"]
        elif data.data == DroidEmotionRequest.SORRY:
            self.current_emotion = self._emotions_speed_dict["SORRY"]
        elif data.data == DroidEmotionRequest.ANGRY:
            self.current_emotion = self._emotions_speed_dict["ANGRY"]
        else:
            self.current_emotion = self._emotions_speed_dict["NORMAL"]

    def _droid_talker_msg_clk(self, data):
        """
        We publish the message based on the current emotion and the message recieved
        :param data:
        :return:
        """
        sample = []
        spaces = []
        for letter in data.data:
            # We check that the letters in the text are supported
            if letter in letters:
                sample.append(letter)
            else:
                # For the moment if not supported we don't put anything
                pass

        for _ in sample:
            spaces.append(abs(self.current_emotion))

        self._droid_chat_message.data = ''.join(sample)
        self._droid_chat_message.spaces = spaces
        self._speak_pub.publish(self._droid_chat_message)

def main():
    rospy.init_node('droid_talker_node', log_level=rospy.DEBUG)
    DroidTalker()
    rospy.spin()

if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException as e:
        rospy.logerr(e.message)
