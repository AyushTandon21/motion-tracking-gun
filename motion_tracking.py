import os
import cv2
from imutils.video import VideoStream
import time


def object_detection():
    try:
        print("Starting Object Detection Mode")
        # start the video stream thread
        print("[INFO] starting video stream thread...")
        vs = VideoStream(src=0).start()
        # vs = VideoStream(usePiCamera=True).start()
        time.sleep(1.0)
        while True:

                frame = vs.read()
                #frame = imutils.resize(frame, width=450)
                face = detect_face(frame)
                if (len(face) != 0):
                    draw_rectangle(frame,face)
                    out=str(len(face))+" face detected successfully."
                    print(out)

                time.sleep(0.1)
                # show the frame
                cv2.imshow("Frame", frame)

                key = cv2.waitKey(1) & 0xFF

                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                    break

        vs.stop()
    except Exception as e:
        print(e.message)


def detect_face(img):
    # convert the test image to gray image as opencv face detector expects gray images
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # load OpenCV face detector, I am using LBP which is fast
    # there is also a more accurate but slow Haar classifier
    face_cascade = cv2.CascadeClassifier('cascade_classifier/haarcascade_fullbody.xml')

    # let's detect multiscale (some images may be closer to camera than others) images
    # result is a list of faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5);
    # return only the face part of the image
    return faces

def draw_rectangle(img, rects):
    for (x, y, w, h) in rects:
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)


if __name__ == '__main__':
    object_detection()
