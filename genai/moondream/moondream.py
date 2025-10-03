import moondream as md
from PIL import Image
import os
import time

# ---Core Identity & Response Protocols for the iSight Application---
# Since the moondream library doesn't have a separate system prompt, 
# we combine these instructions with the user's question.
SYSTEM_INSTRUCTIONS = """
You are a wearable assistive device for visually impaired users. Your primary function is environmental interpretation and hazard navigation.
- Prioritize safety: Announce immediate hazards like "Stop! [object] directly ahead."
- Be concise and accurate.
- If the prompt is blank, briefly describe the scene or warn of hazards.
- Never describe human faces.
- If asked about safety, answer with a clear "Yes" or "No" and explain why, for example, "Yes, you can take 3 steps forward before the chair."
"""

def get_ai_response(prompt: str, image: Image.Image) -> str:
    """
    Generates a context-aware response for the iSight application.
    
    Args:
        prompt: The user's question.
        image: The PIL Image object.
        model: The initialized moondream model.
        
    Returns:
        The text response from the model.
    """


    API_KEY = os.environ["MOONDREAM_API_KEY"]
    if not API_KEY:
        raise EnvironmentError("Error: MOONDREAM_API_KEY environment variable not set")

    # Initialize the model
    try:
        model = md.VisionLanguageModel(api_key=os.environ["MOONDREAM_API_KEY"])
    except Exception as e:
        print(f"Error initializing model: {e}")
        exit()
    
    # Combine the system instructions with the user's prompt.
    # This guides the model to behave according to the iSight rules.
    full_prompt = f"{SYSTEM_INSTRUCTIONS}\n\nUser's question: {prompt if prompt else 'Describe the scene for safety.'}"
    
    # Use the answer_question method for better contextual responses
    answer = model.answer_question(image, full_prompt)
    return answer

if __name__ == '__main__':
    
    # Define user input and image path
    user_prompt = "Is it safe to move forwards?"
    file_path = "/home/pi/Pictures/test.jpg"

    # Load the image
    if not os.path.exists(file_path):
        print(f"Error: Image file not found at {file_path}")
    else:
        image = Image.open(file_path)
        
        # 4. Get the response and time it
        print("Asking Moondream for an iSight response...")
        start_time = time.monotonic()
        response = get_ai_response(user_prompt, image)
        end_time = time.monotonic()
        
        duration = end_time - start_time
        
        # 5. Print the result
        print(f'\nResponse: {response}')
        print(f'--> Generated in {duration:.2f} seconds')
