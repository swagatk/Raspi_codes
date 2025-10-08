import os
import subprocess
import time

def speak(text, model_path=None):
    """
    Speak the given text using piper TTS.
    """
    if not text or text == "No speech detected":
        print("No valid text to speak.")
        return
    if model_path is None:
        model_path = os.getenv('PIPER_MODEL_PATH',
            '/home/pi/en-us-lessac-medium/en-us-lessac-medium.onnx')

    # This command generates the speech and pipes it directly to the audio player
    piper_command = (
        f'echo "{text}" | '
        f'/home/pi/genai_app/bin/piper --model {model_path} --output_raw | '
        f'paplay --raw --rate=22050 --channels=1'
    )
    try:
        subprocess.run(piper_command, shell=True, check=True)
        print("Speech playback complete")
    except subprocess.CalledProcessError as e:
        print(f"Error during piper TTS: {e.stderr.decode()}")
    except Exception as e:
        print(f"Error during piper TTS: {str(e)}")



if __name__ == '__main__':
    text = "Hello! How are you today?"
    speak(text)
    
