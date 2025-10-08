import subprocess
import sys

def say_text(text_to_speak, voice="slt"):
    """
    Uses the Flite command-line tool to speak the provided text.
    """
    if not text_to_speak:
        print("No text provided to speak.")
        return

    # The command to be executed.
    # We use a list of arguments to avoid issues with shell quoting.
    command = ["flite", "-voice", voice, "-t", text_to_speak]

    try:
        # Run the command.
        # stdout=subprocess.PIPE and stderr=subprocess.PIPE will capture the output
        # if you need it. For just speaking, it's not strictly necessary.
        print(f"Speaking: {text_to_speak}")
        subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        print("Error: 'flite' command not found.")
        print("Please ensure Flite is installed (`sudo apt install flite`).")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        # This error occurs if flite returns a non-zero exit code
        print(f"Error executing Flite: {e}")
        print(f"Stderr: {e.stderr.decode()}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

# --- Main part of the script ---
if __name__ == "__main__":
    try:
        user_text = input("What would you like me to say? ")
        say_text(user_text)
    except KeyboardInterrupt:
        print("\nExiting.")