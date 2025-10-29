import os
import time

from langchain_ollama import ChatOllama, OllamaEmbeddings
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import PyPDFLoader
from langchain_community.vectorstores import Chroma
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import RunnableLambda, RunnablePassthrough
from langchain_core.output_parsers import StrOutputParser

OLLAMA_BASE_URL = os.environ.get("OLLAMA_BASE_URL", "https://ollama.com")
LOCAL_OLLAMA_BASE_URL = os.environ.get("LOCAL_OLLAMA_BASE_URL", "http://localhost:11434")
OLLAMA_API_KEY = os.environ.get("OLLAMA_API_KEY")

print(OLLAMA_API_KEY)

if not OLLAMA_API_KEY:
    raise EnvironmentError("Set OLLAMA_API_KEY before running this script.")

client_kwargs = {"headers": {"Authorization": f"Bearer {OLLAMA_API_KEY}"}}

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
##
##embeddings = OllamaEmbeddings(
##    model="mxbai-embed-large",
##    base_url=LOCAL_OLLAMA_BASE_URL,
##)

print("Loading and splitting document...")
loader = PyPDFLoader("/home/pi/Raspi_codes/genai/ollama/local_rag/paper.pdf")
docs = loader.load()

text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200)
splits = text_splitter.split_documents(docs)

print("Creating vector store...")
vectorstore = Chroma.from_documents(documents=splits, embedding=embeddings)

prompt = ChatPromptTemplate.from_template(
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
    | prompt
    | llm
    | StrOutputParser()
)

print("Ready to answer questions with Ollama Cloud. (Ctrl+C to quit)")

try:
    while True:
        query = input("\nAsk a question about your PDF: ")
        if query.lower() in ["exit", "quit"]:
            break

        print("Thinking...")
        start_time = time.monotonic()
        response = rag_chain.invoke(query)
        end_time = time.monotonic()
        print("\nAnswer:")
        print(response)
        print(f"\n(Response generated in {end_time - start_time:.2f} seconds)")

except KeyboardInterrupt:
    print("\nDemo finished.")
