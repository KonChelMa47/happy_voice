#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from happy_voice_msgs.srv import TextToSpeech
import requests
import os
import threading

class PiperTTSService(Node):
    def __init__(self):
        super().__init__('piper_tts_service')
        self.srv = self.create_service(TextToSpeech, '/text_to_speech', self.speak_callback)
        self.get_logger().info("🗣️ Piper TTS Service ready on /text_to_speech")

    def speak_callback(self, request, response):
        text = request.text.strip()
        wait = request.wait_until_done
        self.get_logger().info(f"Received: '{text}' (wait={wait})")

        def play():
            try:
                res = requests.get('http://localhost:5000', params={'text': text})
                if res.status_code != 200:
                    raise RuntimeError(f"Piper error {res.status_code}")
                with open('/tmp/tts.wav', 'wb') as f:
                    f.write(res.content)
                os.system('aplay /tmp/tts.wav')
            except Exception as e:
                self.get_logger().error(f"TTS failed: {e}")

        if wait:
            play()
        else:
            thread = threading.Thread(target=play)
            thread.start()

        response.success = True
        return response

def main():
    rclpy.init()
    rclpy.spin(PiperTTSService())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
