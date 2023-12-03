import rclpy
from rclpy.node import Node
from enum import Enum, auto
import speech_recognition as sr
import sounddevice

"""
Github: https://github.com/Uberi/speech_recognition/tree/master
Documentation: https://pypi.org/project/SpeechRecognition/
"""


class State(Enum):
    WAITING = auto()
    LISTENING = auto()
    RECOGNIZING = auto()

class ListenSpeech(Node):
    """Listens to speech and publishes string"""

    def __init__(self):
        super().__init__("listen_speech")
        self.state = State.LISTENING

        self.timer = self.create_timer(1.0/100.0, self.timer_callback)

        # Create a recognizer instance...
        self.get_logger().info("HERE1", once=True)
        self.recognizer = sr.Recognizer()
        self.get_logger().info("HERE2", once=True)
        self.audio = None
        self.spoken_language = None
        self.text = None

    
    def timer_callback(self):

        self.get_logger().info("Running speech...", once=True)
        # self.state = State.LISTENING

        if self.state == State.LISTENING:

            self.get_logger().info("HERE3", once=True)
            # Currently using default microphone (from computer) as audio source
            with sr.Microphone() as source:
                self.get_logger().info("Say something...", once=True)
                # Adjust for ambient noise (if necessary)
                self.recognizer.adjust_for_ambient_noise(source)
                # Listen for speech (by default, it listens until it detects a pause)
                self.audio = self.recognizer.listen(source)

            self.state = State.RECOGNIZING

        elif self.state == State.RECOGNIZING:

            # Turn the recorded language into a string
            try:
                self.get_logger().info("Recognizing...", once=True)
                self.spoken_language = 'en'
                self.text = self.recognizer.recognize_google(self.audio, language=self.spoken_language)
                print("You said:", self.text)
            except sr.UnknownValueError:
                print("Sorry, could not understand audio")
            except sr.RequestError as e:
                print("Error:", str(e))
            
            self.state = State.WAITING

# I think next I need to publish the speech onto a topic
# In addition, need to figure out how to get name of the language that is being spoken in

def listen_speech_entry(args=None):
    rclpy.init(args=args)
    node = ListenSpeech()
    rclpy.spin(node)
    rclpy.shutdown()