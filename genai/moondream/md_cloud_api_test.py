import moondream as md
from PIL import Image
import os
import time


# acess the model
model = md.vl(api_key=os.environ["MOONDREAM_API_KEY"])


# load an image
image = Image.open("/home/pi/Pictures/test.jpg")

start_time = time.monotonic()
caption_response = model.caption(image, length="short")
end_time = time.monotonic()
duration = end_time - start_time
print(caption_response["caption"])
print(f'Response generated in {duration:.2f} seconds')


              
