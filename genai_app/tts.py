from espeak import Espeak
import os

def text_to_speech(text, output_file="output_speech.wav", voice="en-us", speed=150, pitch=50):
    """
    Convert text to speech using the eSpeak library and save as a WAV file.
    
    Args:
        text (str): The text to convert to speech.
        output_file (str): Path to save the output WAV file.
        voice (str): eSpeak voice (e.g., 'en-us', 'en-uk'). Default: 'en-us'.
        speed (int): Speech speed (words per minute, 80-450). Default: 150.
        pitch (int): Pitch adjustment (0-99). Default: 50.
    
    Returns:
        str: Path to the generated WAV file or None if an error occurs.
    """
    if not text or text.strip() == "No speech detected":
        print("No valid text to convert to speech.")
        return None
    
    try:
        # Initialize eSpeak
        espeak = Espeak()
        
        # Set parameters
        espeak.speed = speed  # Words per minute
        espeak.pitch = pitch  # Pitch adjustment
        espeak.voice = voice  # Voice selection
        
        # Convert text to speech and save to WAV file
        espeak.say(text, output=output_file)
        print(f"Text-to-speech saved to {output_file}")
        return output_file
    
    except Exception as e:
        print(f"Error during text-to-speech conversion: {str(e)}")
        return None

if __name__ == "__main__":
    # Example usage
    sample_text = "Hello, this is a test of text to speech."
    text_to_speech(sample_text)
