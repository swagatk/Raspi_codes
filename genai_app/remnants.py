
##        # Take a photo
##        picam2.start()
##        time.sleep(0.2)
##        file_path = os.path.expanduser("~/Pictures/test.jpg")
##        picam2.capture_file(filepath)
##        picam2.stop()
##        print(f'file saved as {file_path}')
##
##        # convert speech to text
##        prompt = speech_to_text(wav_file)
##        print(f'prompt:{prompt}')
##
##        # Get image from stored file
##        image = Image.open(file_path)
##        response = ai_model.generate_content([prompt, image])
##        print(f'Response: {response.text}')
##        speak(response.text)
