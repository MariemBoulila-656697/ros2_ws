import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData, AudioInfo
import sounddevice as sd
import numpy as np

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')

        # Parametri
        self.declare_parameter('device_index', 1)
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_duration', 0.1)
        self.declare_parameter('device_name', "Scarlett Solo")
        self.declare_parameter('publish_audio_topic', "audio_stream")
        self.declare_parameter('publish_info_topic', "audio_info")

        self.device_index = self.get_parameter('device_index').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.chunk_duration = self.get_parameter('chunk_duration').value
        self.device_name = self.get_parameter('device_name').value
        self.audio_topic = self.get_parameter('publish_audio_topic').value
        self.info_topic = self.get_parameter('publish_info_topic').value

        # Publisher
        self.audio_pub = self.create_publisher(AudioData, self.audio_topic, 10)
        self.info_pub = self.create_publisher(AudioInfo, self.info_topic, 10)

        # Stream audio
        self.chunk_size = int(self.sample_rate * self.chunk_duration)
        self.stream = sd.InputStream(samplerate=self.sample_rate,
                                     channels=self.channels,
                                     dtype='float32',
                                     device=self.device_index)
        self.stream.start()

        # Timer per pubblicazione
        self.timer = self.create_timer(self.chunk_duration, self.timer_callback)
        self.get_logger().info(f"🎙️ Publisher avviato sul dispositivo '{self.device_name}'")

    def timer_callback(self):
        frames, _ = self.stream.read(self.chunk_size)

        # Pubblica audio
        audio_msg = AudioData()
        #audio_msg.device_name = self.device_name
        audio_msg.data = frames.flatten().tobytes()
        #audio_msg.timestamp = self.get_clock().now().to_msg()
        self.audio_pub.publish(audio_msg)

        # Pubblica info
        info_msg = AudioInfo()
        info_msg.sample_rate = self.sample_rate
        info_msg.channels = self.channels
        info_msg.sample_format = "float32"
        info_msg.coding_format = "PCM"
        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

