import streamlit as st
import os
import time
import tempfile

from langchain_ollama import ChatOllama, OllamaEmbeddings
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import PyPDFLoader
from langchain_community.vectorstores import Chroma
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import RunnableLambda, RunnablePassthrough
from langchain_core.output_parsers import StrOutputParser

# --- Configuration ---
OLLAMA_BASE_URL = os.environ.get("OLLAMA_BASE_URL", "https://ollama.com")
LOCAL_OLLAMA_BASE_URL = os.environ.get("LOCAL_OLLAMA_BASE_URL", "http://localhost:11434")
OLLAMA_API_KEY = os.environ.get("OLLAMA_API_KEY")

if not OLLAMA_API_KEY:
    st.error("OLLAMA_API_KEY environment variable is not set. Please set it before running.")
    st.stop()

@st.cache_resource
def get_rag_chain(pdf_bytes: bytes, pdf_name: str):
    """
    Loads the PDF, creates a vector store, and returns a RAG chain.
    This function is cached to avoid expensive re-computation on each interaction.
    """
    with st.spinner(f"Loading and processing {pdf_name}..."):
        # Write bytes to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix=".pdf") as tmp:
            tmp.write(pdf_bytes)
            tmp_path = tmp.name

        client_kwargs = {"headers": {"Authorization": f"Bearer {OLLAMA_API_KEY}"}}

        # The rest of the function remains the same, but loads from the temp path
        # and cleans up afterwards.
        llm = ChatOllama(
            model="deepseek-v3.1:671b-cloud",
            base_url=OLLAMA_BASE_URL,
            client_kwargs=client_kwargs,
            temperature=0.2,
        )

        embeddings = OllamaEmbeddings(
            model="twine/mxbai-embed-xsmall-v1:latest",
            base_url=LOCAL_OLLAMA_BASE_URL,
        )

        try:
            loader = PyPDFLoader(tmp_path)
            docs = loader.load()
        finally:
            os.remove(tmp_path) # Clean up the temporary file

        text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200)
        splits = text_splitter.split_documents(docs)

        vectorstore = Chroma.from_documents(documents=splits, embedding=embeddings)

        prompt_template = ChatPromptTemplate.from_template(
            """Answer the user's question based on this context:
<context>{context}</context>
Question: {input}"""
        )

        retriever = vectorstore.as_retriever()

        def format_docs(docs):
            return "\n\n".join(doc.page_content for doc in docs)

        rag_chain = (
            {
                "context": retriever | RunnableLambda(format_docs),
                "input": RunnablePassthrough(),
            }
            | prompt_template
            | llm
            | StrOutputParser()
        )
        return rag_chain

# --- Streamlit App ---

st.set_page_config(page_title="Document Chat (Cloud LLM)", layout="wide")
st.title("📄 Document Chat with DeepSeek Cloud")

uploaded_file = st.file_uploader("Upload a PDF to chat with", type="pdf")

if uploaded_file:
    # Initialize chat history
    if "messages" not in st.session_state:
        st.session_state.messages = []

    # Display chat messages from history on app rerun
    for message in st.session_state.messages:
        with st.chat_message(message["role"]):
            st.markdown(message["content"])

    # Get the RAG chain
    pdf_bytes = uploaded_file.getvalue()
    rag_chain = get_rag_chain(pdf_bytes, uploaded_file.name)

    # React to user input
    if prompt := st.chat_input(f"Ask a question about {uploaded_file.name}"):
        # Display user message in chat message container
        st.chat_message("user").markdown(prompt)
        # Add user message to chat history
        st.session_state.messages.append({"role": "user", "content": prompt})

        with st.chat_message("assistant"):
            with st.spinner("Thinking..."):
                start_time = time.monotonic()
                response = rag_chain.invoke(prompt)
                end_time = time.monotonic()
                st.markdown(response)
                st.caption(f"Response generated in {end_time - start_time:.2f} seconds")
        # Add assistant response to chat history
        st.session_state.messages.append({"role": "assistant", "content": response})
else:
    st.info("Please upload a PDF file to begin the conversation.")
