import rclpy
from rclpy.node import Node
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath
from polyglotbot_interfaces.srv import StringToWaypoint
import numpy as np
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory


class CreateWaypoint(Node):
    """
    A ROS2 Node for creating waypoints from a given string.

    Attributtes:
    ------------
        - waypoint_srv: A ROS2 service for converting a string to a list of waypoints.
    """

    def __init__(self):
        """
        Construct the CreateWaypoint class.

        Initializes the 'create_waypoint' node and creates a service for
        converting strings to waypoints.
        """
        super().__init__("create_waypoint")

        # Service
        self.waypoint_srv = self.create_service(StringToWaypoint, 'string2waypoint',
                                                self.string2waypoint_callback)

    def string2waypoint_callback(self, request, response):
        """
        Call back method for the string to waypoint service.

        Arguements
        ----------colcon test --packages-select string2waypoints --event-handlers console_cohesion+

            - request: The service request containing the string and the language of the string.

        Returns
        -------
            - response: The service response containing the waypoints.

        """
        self.string = request.text
        self.language = request.language
        self.waypoint = self.str2waypoint(self.string, self.language)
        self.message = self.create_waypoints_message(self.waypoint)
        response.waypoints = self.message
        return response

    def create_waypoints_message(self, waypoints):
        """
        Create a list of waypoints into the proper ROS2 message format.

        Arguments
        ---------
            - waypoints: List of waypoints.

        Returns
        -------
            - point_messages: List of point messages.

        """
        point_messages = []
        for i in waypoints:
            point_messages.append(Point(x=i[0], y=i[1], z=i[2]))

        return point_messages

    def str2waypoint(self, string, language):
        """
        Convert a string to a list of waypoints based on the specified language.

        Arguments:
        ----------
            - string: Input string
            - language: Language code

        Returns
        -------
            - test: A list of waypoints in the form [x, y, z]

        """
        # Prepare for different languages
        # Need to upload to main package and create package share paths
        package_name = "string2waypoints"
        package_share_directory = get_package_share_directory(package_name)

        if language == 'zh-cn':
            fontPath = package_share_directory + "/NotoSansSC-VariableFont_wght.ttf"
        elif language == 'zh-tw':
            fontPath = package_share_directory + "/NotoSansTC-VariableFont_wght.ttf"
        elif language == 'ja':
            fontPath = package_share_directory + "/NotoSansJP-VariableFont_wght.ttf"
        elif language == 'ko':
            fontPath = package_share_directory + "/NotoSansKR-VariableFont_wght.ttf"
        elif language == 'iw' or language == 'yi':
            fontPath = package_share_directory + "/NotoSansHebrew-VariableFont_wdth,wght.ttf"
        else:
            fontPath = package_share_directory + "/Roboto-Thin.ttf"

        # Create FontProperties object with "custom" google font
        customFont = FontProperties(fname=fontPath)

        order_of_char = []

        startX, startY = 0, 0

        for char in string:
            if char.isspace():
                continue
            # Path of current character
            charPath = TextPath((startX, startY), char, size=1., prop=customFont)
            # Get vertices of path
            charVerts = charPath.vertices
            # Remove unnecessary points
            charVerts = charVerts[~(charVerts[:, 0] % 1 < 0.01)]
            # print(charVerts.shape)

            duplicate_entry = []

            for i in range(0, len(charVerts)):
                for j in range(i + 1, len(charVerts)):
                    if np.array_equal(charVerts[i], charVerts[j]):
                        duplicate_entry.append(charVerts[j])

            # Find indices of duplicate entries
            indices_of_duplicates = []
            for duplicate in duplicate_entry:
                indices = [index for index, cv in enumerate(charVerts)
                           if np.array_equal(cv, duplicate)]
                indices_of_duplicates.append(indices)

            ind = []
            for i, indices in enumerate(indices_of_duplicates):
                ind += [indices[1]+1]
                print(ind)

            order_of_char.append((char, ind))
        # print("order of chars before sorting:", order_of_char)

        order_of_char = [(char, sorted(ind)) for char, ind in order_of_char]
        # print("order of char after sorting:", order_of_char)

        for char, ind in order_of_char:
            a = np.split(charVerts, ind)
            a = [arr for arr in a if arr.size > 0]
            a = [np.column_stack((arr, np.full(arr.shape[0], 0))) for arr in a]

            test = []
            for i in range(len(a)):
                for j in range(len(a[i])):
                    if i != 0 and j == 0:
                        x = a[i][j][0]
                        y = a[i][j][1]
                        z = 1.0
                        xyz = [x, y, z]
                        test.append(xyz)
                        x = a[i][j][0]
                        y = a[i][j][1]
                        z = -1.0
                        xyz = [x, y, z]
                        test.append(xyz)

                    x = a[i][j][0]
                    y = a[i][j][1]
                    z = a[i][j][2]
                    xyz = [x, y, z]
                    test.append(xyz)

                    if i != (len(a) - 1) and j == (len(a[i]) - 1):
                        y = a[i][j][1]
                        z = 1.0
                        xyz = [x, y, z]
                        test.append(xyz)
            return test


def main(args=None):
    rclpy.init(args=args)
    node = CreateWaypoint()
    rclpy.spin(node)
    rclpy.shutdown()

    # 'af': 'afrikaans'
    # 'sq': 'albanian'
    # 'be': 'belarusian'
    # 'bg': 'bulgarian'
    # 'ca': 'catalan'
    # 'hr': 'croatian'
    # 'cs': 'czech'
    # 'da': 'danish'
    # 'nl': 'dutch'
    # 'en': 'english'
    # 'eo': 'esperanto'
    # 'et': 'estonian'
    # 'tl': 'filipino'
    # 'fi': 'finnish'
    # 'fr': 'french'
    # 'gl': 'galician'
    # 'de': 'german'
    # 'el': 'greek'
    # 'hu': 'hungarian'
    # 'is': 'icelandic'
    # 'id': 'indonesian'
    # 'ga': 'irish'
    # 'it': 'italian'
    # 'la': 'latin'
    # 'lv': 'latvian'
    # 'lt': 'lithuanian'
    # 'mk': 'macedonian'
    # 'ms': 'malay'
    # 'mt': 'maltese'
    # 'no': 'norwegian'
    # 'pl': 'polish'
    # 'pt': 'portuguese'
    # 'ro': 'romanian'
    # 'ru': 'russian'
    # 'sr': 'serbian'
    # 'sk': 'slovak'
    # 'sl': 'slovenian'
    # 'es': 'spanish'
    # 'sw': 'swahili'
    # 'sv': 'swedish'
    # 'tr': 'turkish'
    # 'uk': 'ukrainian'
    # 'vi': 'vietnamese'
    # 'cy': 'welsh'

    # 'iw': 'hebrew'
    # 'yi': 'yiddish'

    # 'zh-CN': 'chinese_simplified'
    # 'zh-TW': 'chinese_traditional'

    # 'ja': 'japanese'

    # 'ko': 'korean'

    # Needs work
    # 'ar': 'arabic'
    # 'hi': 'hindi'
    # 'fa': 'persian'
    # 'th': 'thai'
