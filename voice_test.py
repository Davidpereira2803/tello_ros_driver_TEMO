import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer

model = Model("vosk-model-small-en-us-0.15")
recognizer = KaldiRecognizer(model, 16000)

q = queue.Queue()

def callback(indata, frames, time, status):
    if status:
        print(status)
    q.put(bytes(indata))

with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                       channels=1, callback=callback):
    print("Listening... (say 'shoot' or 'reload')")

    while True:
        data = q.get()
        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            text = result.get("text", "")
            print("You said:", text)

            if "shoot" in text:
                print("SHOOT detected!")
            elif "reload" in text:
                print("RELOAD detected!")
