import argparse
import sys
import time
import RPi.GPIO as GPIO
import cv2
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from utils import visualize

GPIO.setmode(GPIO.BOARD)

control_pins1 = [29, 31, 33, 35] 
control_pins2 = [36, 38, 40, 32]

for pin in control_pins1:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

for pin in control_pins2:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)
    

halfstep_seq = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

# Reverse sequence
halfstep_seq_reverse = halfstep_seq[::-1]



steps = 5


def run(model: str, width: int, height: int) -> None:
  """Continuously run inference on images acquired from the camera.

  Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
  """

  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(0)



cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  detection_result_list = []

  def visualize_callback(result: vision.ObjectDetectorResult,
                         output_image: mp.Image, timestamp_ms: int):
      result.timestamp_ms = timestamp_ms
      detection_result_list.append(result)



  # Initialize the object detection model
  base_options = python.BaseOptions(model_asset_path=model)
  options = vision.ObjectDetectorOptions(base_options=base_options,
                                         running_mode=vision.RunningMode.LIVE_SSTREAM,
                                         score_threshold=0.35,
                                         result_callback=visualize_callback,
                                         category_allowlist=['potted plant'])   #['potted plant', 'vase'])
  detector = vision.ObjectDetector.create_from_options(options)
  
 
 # Continuously capture images from the camera and run inference
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      sys.exit(
          'ERROR: Unable to read from webcam. Please verify your webcam setting>
      )

    counter += 1
    image = cv2.flip(image, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

    # Run object detection using the model.
    detector.detect_async(mp_image, counter)
    current_frame = mp_image.numpy_view()
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
        end_time = time.time()
        fps = fps_avg_frame_count / (end_time - start_time)
        start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(current_frame, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    if detection_result_list:
        highest_scoring_detection = None
        highest_score = 0
        #print(detection_result_list)
        vis_image = visualize(current_frame, detection_result_list[0])
        for detection_result in detection_result_list:
          for detection in detection_result.detections:
              for category in detection.categories:
                  # Check if the category name is 'potted plant' and the score >
                  if category.score > highest_score:
                      highest_scoring_detection = detection
                      highest_score = category.score
        if highest_scoring_detection:
          bounding_box = highest_scoring_detection.bounding_box
          print(f"Highest Scoring Potted Plant Detection:")
          center_x = bounding_box.origin_x + (bounding_box.width / 2)
          center_y = bounding_box.origin_y + (bounding_box.height / 2)
          print(f"Origin X: {bounding_box.origin_x}, Origin Y: {bounding_box.origin_y}, Width: {bounding_box.width}, Height: {bounding_box.height},Center: {center_x};{center_y} ")
          print(f"Score: {highest_score}")
          cv2.circle(vis_image, (int(center_x), int(center_y)), radius=10,color=(0, 255, 0), thickness=3) 

          if center_y<250:
            for i in range(steps):
               for halfstep in range(8):
                 for pin in range(4):
                   GPIO.output(control_pins2[pin], halfstep_seq[halfstep][pin])
                 time.sleep(0.001)
          if center_y>470:
            for i in range(steps):
               for halfstep in range(8):
                 for pin in range(4):
                   GPIO.output(control_pins2[pin], halfstep_seq_reverse[halfstep][pin])
                 time.sleep(0.001)
          if center_x<420:
            for i in range(steps):
               for halfstep in range(8):
                 for pin in range(4):
                    GPIO.output(control_pins1[pin], halfstep_seq_reverse[halfstep][pin])
                 time.sleep(0.001)
          if center_x>860:
            for i in range(steps):
               for halfstep in range(8):
                 for pin in range(4):
                   GPIO.output(control_pins1[pin], halfstep_seq[halfstep][pin])
                 time.sleep(0.001)
        cv2.line(vis_image, (0,250), (1280,250), (0,255,0), 9)
        cv2.line(vis_image, (0,470), (1280,470), (0,255,0), 9)
        cv2.line(vis_image, (420,0), (420,720), (0,255,0), 9)
        cv2.line(vis_image, (860,0), (860,720), (0,255,0), 9)
          
        cv2.imshow('object_detector', vis_image)
        detection_result_list.clear()

        
    else:
        cv2.line(current_frame, (0,250), (1280,250), (0,255,0), 9)
        cv2.line(current_frame, (0,470), (1280,470), (0,255,0), 9)
        cv2.line(current_frame, (420,0), (420,720), (0,255,0), 9)
        cv2.line(current_frame, (860,0), (860,720), (0,255,0), 9)
         
        cv2.imshow('object_detector', current_frame)
   # Stop the program if the ESC key is pressed.
    if cv2.waitKey(1) == 27:
      break

  detector.close()
  cap.release()
  cv2.destroyAllWindows()


def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default='efficientdet.tflite')
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=0)
  parser.add_argument(
    '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=1280)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=720)
  args = parser.parse_args()

  run(args.model, args.frameWidth, args.frameHeight)


if __name__ == '__main__':
  main()



    

