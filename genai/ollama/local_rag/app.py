import os
import tempfile
import time

import streamlit as st

from langchain_community.vectorstores import Chroma
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import PyPDFLoader
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import RunnableLambda, RunnablePassthrough
from langchain_core.output_parsers import StrOutputParser
from langchain_ollama import OllamaLLM as Ollama
from langchain_ollama import OllamaEmbeddings

st.set_page_config(page_title="PDF Chat", page_icon="📄")
st.title("📄 Local PDF Chat")
st.write("Upload a PDF and ask questions using a local Ollama model.")

@st.cache_resource
def load_models():
    #llm = Ollama(model="llama3:8b")
    llm = Ollama(model="smollm2:latest")
    #embeddings = OllamaEmbeddings(model="mxbai-embed-large")
    embeddings = OllamaEmbeddings(model="twine/mxbai-embed-xsmall-v1:latest")
    return llm, embeddings

@st.cache_resource
def build_chain(pdf_bytes: bytes):
    with tempfile.NamedTemporaryFile(delete=False, suffix=".pdf") as tmp:
        tmp.write(pdf_bytes)
        tmp_path = tmp.name
    try:
        loader = PyPDFLoader(tmp_path)
        docs = loader.load()
    finally:
        os.remove(tmp_path)

    splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200)
    splits = splitter.split_documents(docs)

    llm, embeddings = load_models()
    vectorstore = Chroma.from_documents(splits, embeddings)
    retriever = vectorstore.as_retriever()

    prompt = ChatPromptTemplate.from_template(
        "Answer the user's question based on this context:\n<context>{context}</context>\nQuestion: {input}"
    )

    def format_docs(docs):
        return "\n\n".join(doc.page_content for doc in docs)

    return (
        {
            "context": retriever | RunnableLambda(format_docs),
            "input": RunnablePassthrough(),
        }
        | prompt
        | llm
        | StrOutputParser()
    )

uploaded_file = st.file_uploader("Upload PDF", type="pdf")
if uploaded_file:
    pdf_bytes = uploaded_file.getvalue()
    rag_chain = build_chain(pdf_bytes)

    query = st.text_input("Your question", placeholder="What is the paper about?")
    if st.button("Ask") and query.strip():
        with st.spinner("Thinking..."):
            start_time = time.monotonic()
            answer = rag_chain.invoke(query)
            end_time = time.monotonic()
        st.markdown("**Answer:**")
        st.write(answer)
        duration = end_time - start_time
        st.caption(f"Response generated in {duration:.2f} seconds")
else:
    st.info("Please upload a PDF to start chatting.")