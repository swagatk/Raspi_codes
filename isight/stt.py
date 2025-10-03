from vosk import Model, KaldiRecognizer, SetLogLevel
import json
import subprocess
import os
import wave


def speech_to_text(wav_file, model_path='/home/pi/vosk-model-small-en-us-0.15'):
    """
    Converts a WAV file into text using VOSK speech recognition library

    Args:
        wav_file: path to input wave file
        model_path: path to vosk model
    Returns:
        Str: Transcribed text from the audio file
    """
    # check if wave file exists
    if not os.path.exists(wav_file):
        raise FileNotFoundError(f"WAV file not found:{wav_file}")

    # check if vosk model is available
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Vosk model not found:{model_path}")

    # set vosk log level: -1 for minimal logging
    SetLogLevel(-1)

    try:
        # open the file
        with wave.open(wav_file, 'rb') as wf:
            # Verify WAV file format
            if wf.getnchannels() != 1:
                raise ValueError("WAV file must be mono")
            if wf.getsampwidth() != 2:
                raise ValueError("WAV file must be 16-bit PCM")
            if wf.getframerate() not in [8000, 16000, 32000, 441000, 48000]:
                raise ValueError("Unsupport sample rate. Use 16000 or other supported rate")
                

            # load the vosk model
            vosk_model = Model(model_path)
            recognizer = KaldiRecognizer(vosk_model, wf.getframerate())
            recognizer.SetWords(True) # enable word level results
            recognizer.SetPartialWords(True) # enable partial words

            # read and process audio data
            transcribed_text = []
            while True:
                data = wf.readframes(4000) # read in chunks
                if len(data) == 0:
                    break
                if recognizer.AcceptWaveform(data):
                    # Get partial result
                    result = json.loads(recognizer.Result())
                    if "text" in result and result["text"]:
                        transcribed_text.append(result["text"])
                else:
                    part_result = json.loads(recognizer.PartialResult())
                    if "text" in part_result and part_result["text"]:
                        transcribed_text.append(part_result["text"])

            # Get final result
            final_result = json.loads(recognizer.FinalResult())
            if "text" in final_result and final_result["text"]:
                transcribed_text.append(final_result["text"])
    except Exception as e:
        print(f"Error during speech-to-text recognition: {str(e)}")
        return ""
    finally:
        wf.close()
        pass
    # Combine all transcribed text
    return " ".join(transcribed_text).strip()


if __name__ == '__main__':
    filename = './output.wav'
    text = speech_to_text(wav_file=filename)
    print(text)

    text2 = "what do you see?"
    speak(text2)

