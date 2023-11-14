"""
Test publisher node that receives a string (in the form of a srv), returns the response as the
translated string, and constantly publishes the translated string as a msg
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from polyglotbot_interfaces.srv import TranslateString
import googletrans


class Translator(Node):
    """WIP"""

    def __init__(self):
        # Initialize the node
        super().__init__("translator")
        # Initialize variables
        self.init_var()

        #
        # PARAMETERS
        #
        self.declare_parameter(
            "target_language",
            "en",
            ParameterDescriptor(
                description="The height between the turtle robot's platform and the ground"
            ),
        )
        self.target_language = (
            self.get_parameter("target_language").get_parameter_value().string_value
        )

        #
        # PUBLISHERS
        #
        self.pub_translated_string = self.create_publisher(String, "translated_msg", 10)

        #
        # SERVICES
        #
        self.srv_target_language = self.create_service(
            TranslateString, "target_language", self.target_language_callback
        )
        self.srv_input_string = self.create_service(
            TranslateString, "input_msg", self.input_string_callback
        )

        #
        # TIMER
        #
        period = 1 / self.frequency
        self.timer = self.create_timer(period, self.timer_callback)

    # Initialize variables
    def init_var(self):
        """WIP"""
        self.frequency = 1
        self.translator = googletrans.Translator()
        self.input_string = None
        self.source_lang = None
        self.translated_string = None

    #
    # TIMER CALLBACK
    #
    def timer_callback(self):
        """WIP"""
        # If no initial message is received, standby
        if self.input_string is None:
            self.get_logger().info("Standing by to receive input string...", once=True)
        # Once received, begin publishing the translated string
        else:
            self.get_logger().info(
                "Target language: %s" % self.target_language
            )
            self.get_logger().info(
                "Translated string: %s" % self.translated_string
            )

            msg = String()
            msg.data = self.translated_string
            self.pub_translated_string.publish(msg)

    def target_language_callback(self, request, response):
        self.target_language = request.input
        self.translated_string = self.translator.translate(
            text=self.input_string, src=self.source_lang, dest=self.target_language
        ).text

        response.output = "Target language switched to: " + self.target_language
        return response

    def input_string_callback(self, request, response):
        # Takes note of the source language and translated message
        self.source_lang = self.translator.detect(request.input).lang
        self.input_string = request.input

        self.get_logger().info("Source language: %s" % self.source_lang)
        self.get_logger().info("Received string: %s" % self.input_string)

        self.translated_string = self.translator.translate(
            text=self.input_string, src=self.source_lang, dest=self.target_language
        ).text

        response.output = self.translated_string
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Translator()
    rclpy.spin(node)
    rclpy.shutdown()
