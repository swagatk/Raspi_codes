import time

import cv2
import numpy as np
from picamera2 import Picamera2
from rapidocr_onnxruntime import RapidOCR


OCR_INTERVAL_SECONDS = 0.75


def main():
	engine = RapidOCR()
	camera = Picamera2()
	config = camera.create_preview_configuration(
		main={"format": "RGB888", "size": (640, 480)}
	)
	camera.configure(config)
	camera.start()

	print("Live printed-text OCR started at 640x480.")
	print("Press Space to scan immediately, or Q to quit.")

	last_ocr_time = 0.0
	last_text = None
	detections = []

	try:
		while True:
			frame = camera.capture_array()
			current_time = time.monotonic()
			key = cv2.waitKey(1) & 0xFF

			if key == ord("q"):
				break

			if key == ord(" ") or current_time - last_ocr_time >= OCR_INTERVAL_SECONDS:
				detections, _ = engine(frame)
				detections = detections or []
				last_ocr_time = current_time

				text_lines = [line[1].strip() for line in detections if line[1].strip()]
				recognized_text = "\n".join(text_lines)
				if recognized_text != last_text:
					print("\nRecognized text:" if recognized_text else "\nNo text detected.")
					if recognized_text:
						print(recognized_text)
					last_text = recognized_text

			display_frame = frame.copy()
			for box, text, score in detections:
				points = np.asarray(box, dtype=np.int32).reshape((-1, 1, 2))
				cv2.polylines(display_frame, [points], True, (0, 200, 0), 2)
				x, y = points[0, 0]
				cv2.putText(
					display_frame,
					f"{score:.2f}",
					(int(x), max(20, int(y) - 6)),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.5,
					(0, 200, 0),
					1,
					cv2.LINE_AA,
				)

			cv2.imshow("Live OCR (640x480)", display_frame)

	finally:
		camera.stop()
		camera.close()
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
