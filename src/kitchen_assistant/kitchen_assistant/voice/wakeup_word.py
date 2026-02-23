from __future__ import annotations
import os
import numpy as np
from openwakeword.model import Model
from scipy.signal import resample
from ament_index_python.packages import get_package_share_directory

MODEL_NAME = "hello_rokey_8332_32.tflite"

def _model_path() -> str:
    package_path = get_package_share_directory("kitchen_assistant")
    return os.path.join(package_path, "resource", MODEL_NAME)

class WakeupWord:
    """
    Wakeword detector using openwakeword with a custom TFLite model.
    Audio input: 48kHz int16 (pyaudio). Internally resampled to 16kHz.
    """
    def __init__(self, buffer_size: int, threshold: float = 0.3, print_conf: bool = False):
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None
        self.buffer_size = buffer_size
        self.threshold = threshold
        self.print_conf = print_conf

    def set_stream(self, stream):
        self.model = Model(wakeword_models=[_model_path()])
        self.stream = stream

    def is_wakeup(self) -> bool:
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
        outputs = self.model.predict(audio_chunk, threshold=0.1)
        confidence = float(outputs.get(self.model_name, 0.0))
        if self.print_conf:
            print("[wakeword] confidence:", confidence)
        if confidence > self.threshold:
            print("[wakeword] detected!")
            return True
        return False
