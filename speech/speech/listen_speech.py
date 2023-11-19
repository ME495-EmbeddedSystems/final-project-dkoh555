import rclpy
from rclpy.node import Node
from enum import Enum, auto
import speech_recognition as sr

"""
Github: https://github.com/Uberi/speech_recognition/tree/master
Documentation: https://pypi.org/project/SpeechRecognition/
"""

class State(Enum):
    WAITING = auto()
    LISTENING = auto()


class ListenSpeech(Node):
    """Listens to speech and publishes string"""

    def __init__(self):
        super().__init__("listen_speech")
        self.state = State.WAITING

        self.timer = self.create_timer(1.0 / 100.0, self.timer_callback)

        # Create a recognizer instance...
        self.recognizer = sr.Recognizer()

    
    def timer_callback(self):

        if self.state == State.LISTENING:
            # Currently using default microphone (from computer) as audio source
            with sr.Microphone() as source:
                print("Say something...")
                # Adjust for ambient noise (if necessary)
                self.recognizer.adjust_for_ambient_noise(source)
                # Listen for speech (by default, it listens until it detects a pause)
                audio = self.recognizer.listen(source)

            try:
                lang = 'en'
                text = self.recognizer.recognize_google(audio, language=lang)
                print("You said:", text)
            except sr.UnknownValueError:
                print("Sorry, could not understand audio")
            except sr.RequestError as e:
                print("Error:", str(e))
            
            self.state = State.WAITING

def listen_speech_entry(args=None):
    rclpy.init(args=args)
    node = ListenSpeech()
    rclpy.spin(node)
    rclpy.shutdown()