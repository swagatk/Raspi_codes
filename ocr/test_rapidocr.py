import cv2
from rapidocr_onnxruntime import RapidOCR

# Initialize the ONNX inference engine
engine = RapidOCR()

# Run inference on the generated test image
image_path = "./test_sample.png"
result, elapse_list = engine(image_path)

print("\n--- OCR Test Results ---")
if result:
    for idx, line in enumerate(result, 1):
        box, text, score = line
        print(f"Line {idx}: {text} (Confidence: {score:.2f})")
    print(f"Elapsed Time: {sum(elapse_list):.3f}s")
else:
    print("No text detected.")