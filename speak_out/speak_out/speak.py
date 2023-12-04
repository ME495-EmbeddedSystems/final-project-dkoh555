import google_speech

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from polyglotbot_interfaces.srv import SpeakText
from std_msgs.msg import String


class Speak(Node):
    def __init__(self):
        super().__init__("speaker")

        self.lang = "en"
        self.sub_target_lang = self.create_subscription(
            String,
            "target_language",
            callback=self.sub_target_lang_callback,
            qos_profile=10,
        )

        self.cb_group = ReentrantCallbackGroup()

        self.srv_speak = self.create_service(
            SpeakText,
            "speak",
            callback=self.srv_speak_callback,
            callback_group=self.cb_group,
        )

    def sub_target_lang_callback(self, msg: String):
        self.lang = msg.data

    def srv_speak_callback(self, request, response):
        text = request.text

        google_speech.Speech(text="Translated text", lang="en").play()

        speech = google_speech.Speech(text=text, lang=self.lang)
        speech.play()

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node_speaker = Speak()
    rclpy.spin(node=node_speaker)

    node_speaker.destroy_node()
    rclpy.shutdown()
