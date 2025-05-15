import rclpy
from rclpy.node import Node
from happy_voice_msgs.srv import SpeechToText

import torch
from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline
import sounddevice as sd
import numpy as np
import time
from collections import deque
import sys

sys.path.append("/workspace/models/silero-vad")
from silero_vad import SileroVAD

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.get_logger().info('🎙️ STTノードを起動中...')

        # 使用デバイスとデータ型
        device = "cuda:0" if torch.cuda.is_available() else "cpu"
        dtype = torch.float16 if torch.cuda.is_available() else torch.float32

        # Whisper モデル（ローカル）
        model_dir = "/workspace/models/distil-whisper"

        self.model = AutoModelForSpeechSeq2Seq.from_pretrained(
            model_dir,
            torch_dtype=dtype,
            low_cpu_mem_usage=True,
            use_safetensors=True
        ).to(device)

        self.processor = AutoProcessor.from_pretrained(model_dir)
        self.pipe = pipeline(
            "automatic-speech-recognition",
            model=self.model,
            tokenizer=self.processor.tokenizer,
            feature_extractor=self.processor.feature_extractor,
            max_new_tokens=128,
            torch_dtype=dtype,
            device=device,
        )

        # 🔁 オンラインではなくローカル構造から読み込む
        self.vad_model = SileroVAD()
        self.vad_model.load_state_dict(torch.load("/workspace/models/silero_vad.pt"))
        self.vad_model.eval()

        self.get_logger().info("✅ Silero VAD モデルをローカルから読み込みました")

        # ROS2 サービスの作成
        self.srv = self.create_service(SpeechToText, 'speech_to_text', self.speech_callback)

    def speech_callback(self, request, response):
        self.get_logger().info('🛎️ サービスが呼び出されました')
        self.get_logger().info('🎧 録音開始します（ピッ）')
        self.beep()
        audio = self.record_until_silence()
        self.beep()
        result = self.pipe(audio)
        response.text = result['text']
        self.get_logger().info(f"✅ 認識結果: {response.text}")
        return response

    def beep(self, duration=0.15, freq=880, samplerate=44100):
        t = np.linspace(0, duration, int(samplerate * duration), False)
        tone = 0.5 * np.sin(freq * 2 * np.pi * t)
        sd.play(tone, samplerate=samplerate)
        sd.wait()

    def record_until_silence(self, max_duration=10, silence_duration=1.5, device_index=None):
        samplerate = 16000
        frame_size = 512  # Silero VAD が要求する固定長
        frame_duration = frame_size / samplerate  # 約32ms

        buffer = []
        silence_buffer = deque(maxlen=int(silence_duration / frame_duration))
        start_time = time.time()

        with sd.InputStream(samplerate=samplerate, channels=1, dtype='float32', blocksize=frame_size, device=device_index) as stream:
            self.get_logger().info("🗣️ 発話検知を待機中...")
            while True:
                chunk, _ = stream.read(frame_size)
                chunk = np.squeeze(chunk)

                audio_tensor = torch.from_numpy(chunk).unsqueeze(0)

                with torch.no_grad():
                    prob = self.vad_model(audio_tensor, sr=samplerate).item()

                if prob > 0.5:
                    buffer.append(chunk)
                    silence_buffer.clear()
                elif buffer:
                    silence_buffer.append(chunk)
                    if len(silence_buffer) == silence_buffer.maxlen:
                        buffer.extend(silence_buffer)
                        break

                if time.time() - start_time > max_duration:
                    self.get_logger().warn("⚠️ 最大録音時間に達しました")
                    break

        if buffer:
            return np.concatenate(buffer)
        else:
            self.get_logger().warn("⚠️ 音声が検出されず、録音が空でした")
            return np.zeros(1, dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
