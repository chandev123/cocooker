from __future__ import annotations
import os
import tempfile
import numpy as np
import sounddevice as sd
import scipy.io.wavfile as wav
from dotenv import load_dotenv, find_dotenv
from openai import OpenAI

def load_openai_key() -> str:
    key = os.getenv("OPENAI_API_KEY", "")
    if key:
        return key
    env_path = find_dotenv(usecwd=True)
    if env_path:
        load_dotenv(env_path)
    return os.getenv("OPENAI_API_KEY", "")

class STT:
    def __init__(self, duration: float = 5.0, samplerate: int = 16000):
        api_key = load_openai_key()
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY not found. Put it in .env or export OPENAI_API_KEY.")
        self.client = OpenAI(api_key=api_key)
        self.duration = duration
        self.samplerate = samplerate

    def speech2text(self) -> str:
        print("[stt] recording... (%.1fs)" % self.duration)
        audio = sd.rec(
            int(self.duration * self.samplerate),
            samplerate=self.samplerate,
            channels=1,
            dtype="int16",
        )
        sd.wait()
        print("[stt] sending to whisper-1...")
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio)
            with open(temp_wav.name, "rb") as f:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=f,
                )
        text = (transcript.text or "").strip()
        print("[stt] result:", text)
        return text
