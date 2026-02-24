from typing import Tuple, Union
import math
import cv2
import numpy as np
import mediapipe as mp

MARGIN = 10  # pixels
ROW_SIZE = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
TEXT_COLOR = (255, 0, 0)  # red


def _normalized_to_pixel_coordinates(
    normalized_x: float, normalized_y: float, image_width: int,
    image_height: int) -> Union[None, Tuple[int, int]]:
  """Converts normalized value pair to pixel coordinates."""

  # Checks if the float value is between 0 and 1.
  def is_valid_normalized_value(value: float) -> bool:
    return (value > 0 or math.isclose(0, value)) and (value < 1 or
                                                      math.isclose(1, value))

  if not (is_valid_normalized_value(normalized_x) and
          is_valid_normalized_value(normalized_y)):
    # TODO: Draw coordinates even if it's outside of the image bounds.
    return None
  x_px = min(math.floor(normalized_x * image_width), image_width - 1)
  y_px = min(math.floor(normalized_y * image_height), image_height - 1)
  return x_px, y_px


def visualize(
    image,
    detection_result
) -> np.ndarray:
  """Draws bounding boxes and keypoints on the input image and return it.
  Args:
    image: The input RGB image.
    detection_result: The list of all "Detection" entities to be visualize.
  Returns:
    Image with bounding boxes.
  """
  annotated_image = image.copy()
  height, width, _ = image.shape

  for detection in detection_result.detections:
    # Draw bounding_box
    bbox = detection.bounding_box
    start_point = bbox.origin_x, bbox.origin_y
    end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
    cv2.rectangle(annotated_image, start_point, end_point, TEXT_COLOR, 3)

    # Draw keypoints
    for keypoint in detection.keypoints:
      keypoint_px = _normalized_to_pixel_coordinates(keypoint.x, keypoint.y,
                                                     width, height)
      color, thickness, radius = (0, 255, 0), 2, 2
      cv2.circle(annotated_image, keypoint_px, thickness, color, radius)

    # Draw label and score
    category = detection.categories[0]
    category_name = category.category_name
    category_name = '' if category_name is None else category_name
    probability = round(category.score, 2)
    result_text = category_name + ' (' + str(probability) + ')'
    text_location = (MARGIN + bbox.origin_x,
                     MARGIN + ROW_SIZE + bbox.origin_y)
    cv2.putText(annotated_image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)

  return annotated_image


## visualizing gesture

def visualize_gestures(image, recognition_result):
  image_copy = np.copy(image)

  # Get the hand landmarks and connections
  hand_landmarks_list = recognition_result.hand_landmarks
  handedness_list = recognition_result.handedness

  for idx in range(len(hand_landmarks_list)):
    hand_landmarks = hand_landmarks_list[idx]
    handedness = handedness_list[idx]

    # convert the normalized landmarks to pixel coordinates
    image_h, image_w, _ = image_copy.shape
    landmark_px = [(lm.x * image_w, lm.y * image_h) for lm in hand_landmarks]

    # Draw hand landmarks
    for x, y in landmark_px:
      cv2.circle(image_copy, (int(x), int(y)), 5, (255, 0, 0), -1)

    # Draw hand connections
    for connection in mp.solutions.hands.HAND_CONNECTIONS:
      start_node = connection[0]
      end_node = connection[1]
      cv2.line(image_copy,
               (int(landmark_px[start_node][0]), int(landmark_px[start_node][1])),
               (int(landmark_px[end_node][0]), int(landmark_px[end_node][1])),
               (0, 255, 0), 2)

    # Get Gesture name
    if recognition_result.gestures and recognition_result.gestures[idx]:
      gesture_name = recognition_result.gestures[idx][0].category_name

      # Get the top of the wrist to place the text
      wrist_landmark = landmark_px[mp.solutions.hands.HandLandmark.WRIST.value]
      text_location = (int(wrist_landmark[0]), int(wrist_landmark[1] - 20))

      # draw the gesture name on the image
      cv2.putText(image_copy,
                  f"Gesture: {gesture_name}",
                  text_location,
                  cv2.FONT_HERSHEY_SIMPLEX,
                  0.7,
                  (0, 0, 255), 2)

  return image_copy
        
