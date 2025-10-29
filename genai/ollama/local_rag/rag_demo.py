from langchain_community.llms import Ollama
from langchain_community.embeddings import OllamaEmbeddings
from langchain_community.vectorstores import Chroma
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import PyPDFLoader
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import RunnableLambda, RunnablePassthrough
from langchain_core.output_parsers import StrOutputParser

# 1. Init local models via Ollama
llm = Ollama(model="smollm2:latest")
#embeddings = OllamaEmbeddings(model="mxbai-embed-large")
embeddings = OllamaEmbeddings(model="twine/mxbai-embed-xsmall-v1:latest")

print("Loading and splitting document...")
# 2. Load and split document
loader = PyPDFLoader("/home/pi/ollama/local_rag/paper.pdf")
docs = loader.load()
text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200)
splits = text_splitter.split_documents(docs)

print("Creating vector store...")
# 3. Create local vector store
vectorstore = Chroma.from_documents(documents=splits, embedding=embeddings)

# 4. Create retrieval chain
prompt = ChatPromptTemplate.from_template("""Answer the user's question based on this context:
<context>{context}</context>
Question: {input}""")

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

print("Ready to answer questions. (Ctrl+C to quit)")

# 5. Run an interactive loop
try:
    while True:
        query = input("\nAsk a question about your PDF: ")
        if query.lower() in ['exit', 'quit']:
            break

        print("Thinking...")
        response = rag_chain.invoke(query)
        print("\nAnswer:")
        print(response)

except KeyboardInterrupt:
    print("\nDemo finished.")
