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
from speech_processing import speech_to_text, speak

# iniitialize camera
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)


# GPIO pin setup
button = Button(16, bounce_time=0.1)



# Thread lock for recording
recording_lock = Lock()


        

def record_audio(input_device_index=1):
    """
    Record audio from a specified input device
    save into a .wav file.
    """
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    OUTPUT_FILENAME= "output.wav"
    audio = None
    stream = None
    try:
        # initialize PyAudio
        audio = pyaudio.PyAudio()

        # Validate input device index
        if input_device_index is not None:
            device_info = audio.get_device_info_by_index(input_device_index)
            if device_info['maxInputChannels'] <= 0:
                raise ValueError(f"Device {input_device_index} ({device_info['name']}) does not support input")
            print(f"Using device: {device_info['name']}")
        

        # open a stream
        stream = audio.open(format=FORMAT,
                            channels=CHANNELS,
                            rate=RATE,
                            input=True,
                            frames_per_buffer=CHUNK,
                            input_device_index=input_device_index)
        frames = []
        print("Recording ...")
        while button.is_pressed:
            data = stream.read(CHUNK)
            frames.append(data)
        print("Recording stopped")

        print(f"saving to {OUTPUT_FILENAME}....")
        with wave.open(OUTPUT_FILENAME, 'wb') as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(audio.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
        print(f'Audio saved successfully as {OUTPUT_FILENAME}')
        return OUTPUT_FILENAME
    
    except Exception as e:
        print(f"An error occurred:{str(e)}")
        print("Available input devices:")
        try:
            for i in range(audio.get_device_count()):
                dev = audio.get_device_info_by_index(i)
                if dev['maxInputChannels'] > 0:
                    print(f"Device {i}: {dev['name']}, Inputs: {dev['maxInputChannels']}")
        except:
            print("Unable to list devices.")
        return None
    finally:
        if stream is not None:
            stream.stop_stream()
            stream.close()
        if audio is not None:
            audio.terminate()
        time.sleep(0.2)
        

def button_pressed():
    if not recording_lock.acquire(blocking=False):
        print('recording in progress, ignore button press')
        return
    try:
        # record your voice while button is pressed.
        wav_file = record_audio()

        # Take a photo
        picam2.start()
        time.sleep(0.2)
        file_path = os.path.expanduser("~/Pictures/test.jpg")
        picam2.capture_file(file_path)
        picam2.stop()
        print(f'file saved as {file_path}')

        # convert speech to text
        prompt = speech_to_text(wav_file)
        print(f'prompt:{prompt}')


        # Get image from stored file
        image = Image.open(file_path)
        response = ai_model.generate_content([prompt, image])
        print(f'Response: {response.text}')
        speak(response.text)
    except Exception as e:
        print('fError occurred: {str(e)}')
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

