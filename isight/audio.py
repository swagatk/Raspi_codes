import os
import wave
import pyaudio
import sys
import time

def find_input_device():
    """
    Finds the first available audio input device.
    """
    audio = pyaudio.PyAudio()
    device_index = None
    for i in range(audio.get_device_count()):
        dev = audio.get_device_info_by_index(i)
        # Check for devices that have input channels
        if dev['maxInputChannels'] > 0:
            print(f"Found input device {i}: {dev['name']}")
            device_index = i
            break # Use the first one found

    audio.terminate()
    return device_index


def record_audio(button=None):
    """
    Record audio from a specified input device
    save into a .wav file.
    """
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    OUTPUT_FILENAME= "input.wav"
    RECORD_SECONDS = 5 # if button is not defined.
    audio = None
    stream = None
    input_device_index = find_input_device()
    if device_index is None:
        raise ValueError("❌ No audio input device found. Please connect a microphone.")
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
        if button is not None:
            print('Button detected. record while pressed')
            while button.is_pressed:
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)
        else: # record for five seconds
            print('Button not defined, recording for 5 seconds')
            for _ in range(int(RATE / CHUNK * RECORD_SECONDS)):
                data = stream.read(CHUNK, exception_on_overflow=False)
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
        
if __name__ == '__main__':
    record_audio()
