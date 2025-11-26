import os
import numpy as np
from openwakeword.model import Model
from scipy.signal import resample

MODEL_NAME = "alexa"
MODEL_PATH = "/home/up/test_ws/src/pick_and_place_voice_new/resource"


class WakeupWord:
    def __init__(self, buffer_size):
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None
        self.buffer_size = buffer_size

    def is_wakeup(self):
        if self.model is None:
            # 모델이 없으면 항상 True 반환 (테스트용)
            print("Wakeword detection skipped (model not loaded)")
            return True
            
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
        outputs = self.model.predict(audio_chunk, threshold=0.1)
        confidence = outputs[self.model_name]
        print("confidence: ", confidence)
        # Wakeword 탐지
        if confidence > 0.05:
            print("Wakeword detected!")
            return True
        return False

    def set_stream(self, stream):
        # openwakeword 0.6.0 버전의 올바른 API 사용
        try:
            self.model = Model()  # 기본 모델 로드
        except Exception as e:
            print(f"Warning: Could not load openwakeword model: {e}")
            self.model = None
        self.stream = stream
