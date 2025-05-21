import speech_recognition as sr
import time

def main():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)
        print("Voice command test ready. Say 'shoot' or 'reload'...")

    while True:
        with mic as source:
            try:
                print("Listening...")
                audio = recognizer.listen(source, timeout=5)
                command = recognizer.recognize_google(audio).lower()
                print(f"You said: {command}")

                if "shoot" in command:
                    print("SHOOT command recognized!")
                elif "reload" in command:
                    print("RELOAD command recognized!")
                else:
                    print("Unknown command")

                time.sleep(1)

            except sr.WaitTimeoutError:
                print("Timeout: No speech detected.")
            except sr.UnknownValueError:
                print("Could not understand audio.")
            except sr.RequestError as e:
                print(f"Speech recognition error: {e}")

if __name__ == "__main__":
    main()
