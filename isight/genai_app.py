import os
import time
from signal import pause
from gpiozero import Button
import wave
import pyaudio
import sys
from threading import Lock
from PIL import Image
from picamera2 import Picamera2
from gemini import ai_model
from stt import speech_to_text
#from tts import speak  # uses piper which is slow
from tts_flite import speak
from audio import record_audio

# iniitialize camera
picam2 = Picamera2()
camera_config = picam2.create_still_configuration(main={
    "size": (160, 120)})
picam2.configure(camera_config)


# GPIO pin setup
button = Button(16, bounce_time=0.1)

# Thread lock for recording
recording_lock = Lock()


    
def button_pressed():
    if not recording_lock.acquire(blocking=False):
        print('recording in progress, ignore button press')
        return
    try:
        speak("Please speak while the button is pressed.")
        # record your voice while button is pressed.
        wav_file = record_audio(button=button)
        
        # Take a photo
        picam2.start()
        time.sleep(0.1)
        file_path = os.path.expanduser("~/Pictures/test.jpg")
        picam2.capture_file(file_path)
        picam2.stop()
        print(f'file saved as {file_path}')

        # convert speech to text
        prompt = speech_to_text(wav_file)
        print(f'prompt:{prompt}')

        if prompt is None or prompt.strip() == '':
            print('No speech detected, exiting')
            speak("No speech detected, Please try again")
            return
        else: 
            speak(f'Image captured. Generating response. please wait.')
            # Get image from stored file
            image = Image.open(file_path)
            start_time = time.monotonic()
            response = ai_model.generate_content([prompt, image])
            end_time = time.monotonic()
            duration = end_time - start_time
            print(f'Response generated in {duration:.2f} seconds')
            print(f'Response: {response.text}')
            speak(response.text)
    except Exception as e:
        print(f'Error occurred: {e}')
    finally:
        recording_lock.release()


if __name__ == '__main__':
    

    button.when_pressed = button_pressed    
    print('Ready! Press button to start. Press Ctrl+c to exit')
    try:
        pause()
    except KeyboardInterrupt:
        print('Exiting program')
    finally:
        button.close()
        picam2.close()
        print('GPIO cleaned up')

