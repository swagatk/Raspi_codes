import os
import subprocess

def speak(text, model_path=None, output_file="output.wav"):
    """
    Speak the given text using piper TTS.
    """
    if not text or text == "No speech detected":
        print("No valid text to speak.")
        return
    if model_path is None:
        model_path = os.getenv('PIPER_MODEL_PATH',
            '/home/pi/en-us-lessac-medium/en-us-lessac-medium.onnx')
    try:
        with open("/tmp/speech.txt", "w") as f:
            f.write(text)
        subprocess.run([
            "piper",
            "--model", model_path,
            "--input_file", "/tmp/speech.txt",
            "--output_file", output_file
        ], check=True)
        subprocess.run(["paplay", output_file], check=True)
        print(f"Speech saved to {output_file}")
    except subprocess.CalledProcessError as e:
        print(f"Error during piper TTS: {e.stderr.decode()}")
    except Exception as e:
        print(f"Error during piper TTS: {str(e)}")


if __name__ == '__main__':
    text = "Hello! How are you today?"
    speak(text)
    
