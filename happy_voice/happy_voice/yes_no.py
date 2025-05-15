import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from happy_voice_msgs.srv import SpeechToText, YesNo, TextToSpeech
import time

class YesNoNode(Node):
    def __init__(self):
        super().__init__('yes_no_node')
        self.cb_group = ReentrantCallbackGroup()

        self.stt_client = self.create_client(SpeechToText, 'speech_to_text', callback_group=self.cb_group)
        self.tts_client = self.create_client(TextToSpeech, 'text_to_speech', callback_group=self.cb_group)
        self.srv = self.create_service(YesNo, 'yes_no', self.handle_yes_no, callback_group=self.cb_group)

        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('speech_to_text サービスを待っています...')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('text_to_speech サービスを待っています...')

    def handle_yes_no(self, request, response):
        yes_words = {"yes", "yeah", "yep"}
        no_words = {"no", "nope", "nah"}

        while True:
            # ✅ TTSで「Please say yes or no.」
            if not self.speak("Please say yes or no."):
                response.result = False
                return response
            time.sleep(0.5)

            # ✅ STT呼び出し
            stt_req = SpeechToText.Request()
            stt_future = self.stt_client.call_async(stt_req)
            rclpy.spin_until_future_complete(self, stt_future)

            if stt_future.result() is None:
                self.get_logger().error("❌ STTの応答がありませんでした")
                response.result = False
                return response

            text = stt_future.result().text.strip().lower().strip(".,!?")
            self.get_logger().info(f"✅ 認識結果: {text}")

            if text in yes_words:
                response.result = True
                return response
            elif text in no_words:
                response.result = False
                return response
            else:
                self.get_logger().warn(f"⚠️ 不明な応答「{text}」、再試行します")
                self.speak("Sorry, one more time please.")

    def speak(self, text):
        """TTSで発話し、成功すればTrueを返す"""
        tts_req = TextToSpeech.Request()
        tts_req.text = text
        tts_req.wait_until_done = True
        self.get_logger().info(f"🗣️ TTS発話: {text}")
        future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            return True
        self.get_logger().error("❌ TTSが失敗しました")
        return False


def main():
    rclpy.init()
    node = YesNoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
