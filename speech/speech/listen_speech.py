import rclpy
from rclpy.node import Node
from enum import Enum, auto
import speech_recognition as sr
from polyglotbot_interfaces.srv import TranslateString
from std_msgs.msg import String
# import sounddevice # This needs to be imported to run this code (even when unused)


class State(Enum):
    WAITING = auto()
    LISTENING = auto()
    RECOGNIZING = auto()


class ListenSpeech(Node):
    """Listens to speech. Triggered by service call."""

    def __init__(self):
        super().__init__("listen_speech")
        self.state = State.WAITING

        self.timer = self.create_timer(1.0 / 100.0, self.timer_callback)

        self.srv_listen = self.create_service(
            TranslateString, "record", self.record_callback
        )

        self.listen_pub = self.create_publisher(String, "listen", 10)

        # Create a recognizer instance...
        self.recognizer = sr.Recognizer()
        self.audio = None
        self.spoken_language = None
        self.text = None

        self.index = 0

        for index, name in enumerate(sr.Microphone().list_microphone_names()):
            if "USB Audio" in name:
                self.index = index
                self.get_logger().info(f"index: {self.index}")

    def record_callback(self, request, response):
        """
        Start recording for speech.

        Args
            request (Request): input
            response (Response): spoken language

        Returns
        -------
            response: Contains the spoken language

        """
        self.spoken_language = request.input
        response.output = self.spoken_language
        self.state = State.LISTENING

        return response

    def timer_callback(self):
        """Execute main timer callback."""
        self.get_logger().info("Running node...", once=True)

        if self.state == State.WAITING:
            self.get_logger().info("Waiting for record source language...", once=True)

            self.audio = None
            self.spoken_language = None
            self.text = None
            # pass

        if self.state == State.LISTENING:
            # Currently using default microphone (from computer) as audio source
            # with sr.Microphone() as source:
            with sr.Microphone(device_index=self.index) as source:
                self.get_logger().info("Say something...")
                # Adjust for ambient noise (if necessary)
                self.recognizer.adjust_for_ambient_noise(source)
                # Listen for speech (by default, it listens until it detects a pause)
                self.audio = self.recognizer.listen(source)

            self.state = State.RECOGNIZING

        elif self.state == State.RECOGNIZING:
            # Turn the recorded language into a string
            try:
                self.get_logger().info("Recognizing...", once=True)
                self.text = self.recognizer.recognize_google(
                    self.audio, language=self.spoken_language
                )
                self.get_logger().info(f"You said: {self.text}")
                self.listen_pub.publish(String(data=self.text))
            except sr.UnknownValueError:
                self.get_logger().info("Sorry, could not understand audio")
            except sr.RequestError as e:
                self.get_logger().info("Error:", str(e))

            self.state = State.WAITING
            self.get_logger().info("Waiting for record source language...")


def listen_speech_entry(args=None):
    rclpy.init(args=args)
    node = ListenSpeech()
    rclpy.spin(node)
    rclpy.shutdown()
