import os
import numpy as np
from openwakeword.model import Model
from scipy.signal import resample
from ament_index_python.packages import get_package_share_directory

######### Wakeup Word model 설정 ############
# MODEL 추가시: setup.py의 data_files에 model 추가 필
MODEL_NAME = "alexa.onnx"
# MODEL_NAME = "hey_yong_yihan.onnx"
package_path = get_package_share_directory("llm_for_pick_place_voice")
MODEL_PATH = os.path.join(f"{package_path}/resource/{MODEL_NAME}")


class WakeupWord:
    def __init__(self, buffer_size):
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None
        self.buffer_size = buffer_size

    ### Wakeup 단어 감지
    def is_wakeup(self):
        if self.model is None:
            # 모델이 없으면 항상 True 반환 (감지된 것으로 간주)
            print("Wakeupword detection skipped (model not loaded)")
            return True
            
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))   # Whisper는 16kHz를 선호
        # audio_chunk 확인 및 분석
        outputs = self.model.predict(audio_chunk, threshold=0.1)
        confidence = outputs[self.model_name]
        # audo_chunk 디버깅 출력
        # print("outputs:", outputs)
        # print("model name:", self.model_name)
        print(f"Tell me wakeupword: {self.model_name} \n confidence: ", confidence)
        # Wakeword 탐지
        if confidence > 0.05:    # 임계값 조정 가능(0~1)
            print("Wakeupword detected!")
            return True
        return False

    ### audio stream 전달
    def set_stream(self, stream):
        # wakeupword 모델 경로 및 유무 확인
        # print(f"Loading model from: {MODEL_PATH}")
        # print(f"Model file exists: {os.path.exists(MODEL_PATH)}")

        try:
            # 기본 모델 로드
            # self.model()
            self.model = Model(
            wakeword_model_paths=[f"{MODEL_PATH}"],
        )  
            print(f"Wakeupword model {MODEL_NAME} loaded")
        except Exception as e:
            print(f"Warning: Could not load openwakeword model: {e}")
            self.model = None

        self.stream = stream
