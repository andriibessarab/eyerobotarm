import math
from datetime import datetime

import cv2
from pyapriltags import Detector

STREAM_URL = "tcp://10.12.194.1:5000"

STARE_TIME = 1 # Seconds
DISTANCE_THRESHOLD = 50 # Pixels

def main():
    # Use CAP_DSHOW on Windows to make webcam opening more reliable.
    cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=2.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    print("AprilTag detector running.")

    detected_tag = None
    tag_detected = None

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        frame = cv2.flip(frame, -1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        detections = detector.detect(gray)

        if detected_tag and detected_tag[0] not in [d.tag_id for d in detections]:
            print(f"Tag ID {detected_tag[0]} lost. No longer detected")
            detected_tag = None

        for detection in detections:
            if detection.decision_margin < 10:
                continue
            
            tag_id = detection.tag_id
            center = detection.center
            corners = detection.corners

            frame_x_center = gray.shape[1] / 2

            x_distance = math.fabs(detection.center[0] - frame_x_center)

            if x_distance > DISTANCE_THRESHOLD:
                continue

            if detected_tag is not None and detected_tag[0] == tag_id:
                # If the same tag is detected, check if it's within the stare time
                if (datetime.now() - detected_tag[2]).total_seconds() < STARE_TIME:
                    continue
                else:
                    detected_tag = None
                    tag_detected = detection
                    print(f"Tag ID {tag_id} detected at distance {x_distance:.2f} pixels from center.")

            elif detected_tag is not None:
                # Take the closest tag to the center if multiple tags are detected
                if detected_tag[1] < x_distance:
                    continue

            detected_tag = [tag_id, x_distance, datetime.now()]

            print(f"Tag ID {tag_id} detected at distance {x_distance:.2f} pixels from center.")

        if tag_detected is not None:
            # TODO: Connect this with the rest of the system


            # Convert corner points to integers for drawing
            corners = tag_detected.corners.astype(int)
            center = tag_detected.center.astype(int)

            # Draw box around tag
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            # Draw center point
            cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)

            # Draw tag ID
            cv2.putText(
                frame,
                f"ID: {tag_detected.tag_id}",
                tuple(center),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 0),
                2
            )

        cv2.imshow("AprilTag Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
