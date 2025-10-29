import streamlit as st
import os
import ollama
import base64
import time
from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage

# --- Configuration ---
OLLAMA_BASE_URL = os.environ.get("OLLAMA_BASE_URL", "https://ollama.com")
OLLAMA_API_KEY = os.environ.get("OLLAMA_API_KEY")

# --- Page Setup ---
st.set_page_config(page_title="Ollama Cloud Chat", page_icon="☁️")
st.title("☁️ Ollama Cloud Chat")

# --- Sidebar for Settings ---
with st.sidebar:
    st.header("Settings")
    # Allow user to select the model
    model_name = st.text_input(
        "Enter Ollama Cloud Model",
        value="qwen3-vl:235b-cloud",
        help="Specify a multi-modal model from ollama.com (e.g., qwen3-vl, llava)",
    )
    if st.button("Apply and Clear Cache"):
        st.cache_resource.clear()
        st.rerun()

    st.info("After changing the model name, click 'Apply and Clear Cache' to load the new model.")
    
# --- API Key Check ---
if not OLLAMA_API_KEY:
    st.error("OLLAMA_API_KEY environment variable is not set. Please set it before running.")
    st.stop()

# --- LLM Initialization ---
@st.cache_resource
def get_llm(model):
    """Cached function to get the LLM instance."""
    client_kwargs = {"headers": {"Authorization": f"Bearer {OLLAMA_API_KEY}"}}
    return ChatOllama(
        model=model,
        base_url=OLLAMA_BASE_URL,
        client_kwargs=client_kwargs,
        temperature=0.3,
    )

try:
    llm = get_llm(model_name)
except Exception as e:
    st.error(f"Failed to initialize the model: {e}")
    st.stop()

# --- File Uploader for Images ---
with st.container():
    uploaded_files = st.file_uploader(
        "Upload images to chat with",
        type=["png", "jpg", "jpeg"],
        accept_multiple_files=True,
    )
    # Store uploaded files in session state
    if uploaded_files:
        image_bytes_list = []
        st.session_state.uploaded_images = []
        for uploaded_file in uploaded_files:
            bytes_data = uploaded_file.read()
            image_bytes_list.append(bytes_data)
            base64_encoded = base64.b64encode(bytes_data).decode("utf-8")
            st.session_state.uploaded_images.append(
                {"name": uploaded_file.name, "base64": base64_encoded}
            )
        # Display thumbnails
        st.image(image_bytes_list, width=100)

# --- Chat History ---
if "messages" not in st.session_state:
    st.session_state.messages = []

# Display chat messages from history
for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        # Handle complex content (text + images) for user messages
        if isinstance(message["content"], list):
            for item in message["content"]:
                if item["type"] == "text":
                    st.markdown(item["text"])
                elif item["type"] == "image_url":
                    # We don't re-display the image here, just acknowledge it was sent
                    pass
        else: # Assistant messages are just text
            st.markdown(message["content"])

# --- Chat Input and Response ---
if prompt := st.chat_input(f"Ask {model_name}..."):
    # Construct the message content, including images if they exist
    message_content = [{"type": "text", "text": prompt}]
    if "uploaded_images" in st.session_state and st.session_state.uploaded_images:
        for img_data in st.session_state.uploaded_images:
            message_content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_data['base64']}"}})

    # Add user message to history and display it
    st.session_state.messages.append({"role": "user", "content": message_content})
    st.chat_message("user").markdown(prompt)

    # Generate and display assistant response
    with st.chat_message("assistant"):
        with st.spinner("Thinking..."):
            try:
                start_time = time.monotonic()
                response = llm.invoke([HumanMessage(content=message_content)])
                end_time = time.monotonic()
                response_content = response.content
                st.markdown(response_content)
                st.caption(f"Response generated in {end_time - start_time:.2f} seconds")
                # Add assistant response to history
                st.session_state.messages.append({"role": "assistant", "content": response_content})
            except ollama.ResponseError as e:
                error_message = f"An error occurred: {e}\n\nThis usually means the model '{model_name}' was not found on the server. Please check the model name on ollama.com and try again."
                st.error(error_message)
            except Exception as e:
                st.error(f"An unexpected error occurred: {e}")

    # Clear uploaded images from session state after sending
    if "uploaded_images" in st.session_state:
        del st.session_state.uploaded_images
