import rclpy
from rclpy.node import Node
from enum import Enum, auto



class State(Enum):
    pass


class ListenSpeech(Node):
    """Listens to speech and publishes string"""

    def __init__(self):
        super().__init__("listen_speech")

        pass


def listen_speech_entry(args=None):
    rclpy.init(args=args)
    node = ListenSpeech()
    rclpy.spin(node)
    rclpy.shutdown()