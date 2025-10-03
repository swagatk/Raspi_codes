import requests
import json
import base64
import os
import time  # Import the time module

# --- Configuration ---
OLLAMA_ENDPOINT = "http://localhost:11434/api/generate"
MODEL_NAME = "moondream"
IMAGE_PATH = "/home/pi/Pictures/test.jpg"  # Change this to the path of your image
PROMPT = "Describe this image in one sentence."

# --- Main Logic ---

def encode_image(filepath):
    """Encodes an image file into base64."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Image file not found at {filepath}")
    with open(filepath, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

def describe_image_with_moondream(image_path, prompt):
    """Sends a request to the local Ollama server with an image and prompt."""
    print(f"Sending request to local Moondream model for '{image_path}'...")
    
    try:
        # Encode the image to base64
        base64_image = encode_image(image_path)
        
        # Prepare the payload for the API request
        payload = {
            "model": MODEL_NAME,
            "prompt": prompt,
            "images": [base64_image],
            "stream": False
        }
        
        # --- Start Timer ---
        start_time = time.monotonic()
        
        # Send the POST request to the Ollama server
        response = requests.post(OLLAMA_ENDPOINT, json=payload)
        
        # --- Stop Timer ---
        end_time = time.monotonic()
        
        response.raise_for_status()
        
        # Calculate the duration
        duration = end_time - start_time
        
        # Parse the JSON response
        response_data = response.json()
        
        print("\n--- ✨ Moondream Description ---")
        print(response_data['response'].strip())
        print("-------------------------------")
        print(f"✅ Response generated in {duration:.2f} seconds.") # Print the duration
        
    except FileNotFoundError as e:
        print(f"Error: {e}")
    except requests.exceptions.RequestException as e:
        print(f"Error connecting to Ollama server: {e}")
        print("Please ensure the Ollama server is running and accessible.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    describe_image_with_moondream(IMAGE_PATH, PROMPT)
