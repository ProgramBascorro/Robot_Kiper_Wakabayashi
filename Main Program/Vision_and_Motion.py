from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import serial
import math
from cobaajagyro import initialize_mpu, calibrate_gyro, get_gyro_data
import threading

# Initialize variables for ball detection and gyroscope reading
ball_detection_active = True
gyroscope_reading_active = False

# pyserial section
global TxData
TxData = 0 
TxD_packet = bytearray(6)
TxD_packet[0] = 0xff
TxD_packet[1] = 0x55
ser = serial.Serial("/dev/ttyUSB0", 57600, timeout=0.1, write_timeout=0.1)
ser.reset_output_buffer()
# initialize the MPU
initialize_mpu()
gx_offset, gy_offset, gz_offset = calibrate_gyro()

def send_data(TxData):
    lowbyte = TxData & 0xff
    highbyte = (TxData >> 8) & 0xff
    TxD_packet[2] = lowbyte
    TxD_packet[3] = ~lowbyte & 0xff
    TxD_packet[4] = highbyte
    TxD_packet[5] = ~highbyte & 0xff
    if ser.write(TxD_packet) != 6:
        print("Transmit error!!!")
    time.sleep(0.1)

def wrap_angle(angle):
    while angle > 90:
        angle -= 210
    while angle < -90:
        angle += 210
    return angle

# Define threshold values for falling forward and backward
gy_threshold_forward = 125 #100  # untuk bangun belakang (nilai kebalikan)
gy_threshold_backward = -80  # untuk bangun depan (nilai kebalikan)

def mapObjectPosition(x, y):
    print("[INFO] Object Center coordinates at X0 = {0} and Y0 = {1}".format(x, y))

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green" ball in the HSV color space
greenLower = (5, 150, 150)
greenUpper = (15, 255, 255)
pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference to the webcam
if not args.get("video", False):
    vs = VideoStream(src=0).start()
else:
    vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

# Function to update values from the trackbars
def update_values(*args):
    pass

# Create a window and trackbars for HSV range calibration
cv2.namedWindow('Calibration')

# Trackbars for HSV range
cv2.createTrackbar('Hue Min', 'Calibration', 0, 179, update_values)
cv2.createTrackbar('Hue Max', 'Calibration', 26, 179, update_values)
cv2.createTrackbar('Saturation Min', 'Calibration', 86, 255, update_values)
cv2.createTrackbar('Saturation Max', 'Calibration', 255, 255, update_values)
cv2.createTrackbar('Value Min', 'Calibration', 100, 255, update_values)
cv2.createTrackbar('Value Max', 'Calibration', 255, 255, update_values)

# Trackbars for morphological operations
cv2.createTrackbar('Kernel Size', 'Calibration', 5, 30, update_values)
cv2.createTrackbar('Area Threshold', 'Calibration', 1500, 10000, update_values)

# Trackbars for brightness and contrast
cv2.createTrackbar('Brightness', 'Calibration', 78, 200, update_values)
cv2.createTrackbar('Contrast', 'Calibration', 59, 200, update_values)

# Function to draw two main Cartesian coordinate axes
def draw_cartesian_axes(frame, origin, color=(0, 255, 0), thickness=1):
    height, width = frame.shape[:2]
    # Vertical line (Y-axis)
    cv2.line(frame, (origin[0], 0), (origin[0], height), color, thickness)
    # Horizontal line (X-axis)
    cv2.line(frame, (0, origin[1]), (width, origin[1]), color, thickness)

# Function to draw and display the reference point
def draw_reference_point(frame, point, color=(255, 0, 0), radius=5):
    cv2.circle(frame, point, radius, color, -1)
    cv2.putText(frame, f"({int(point[0])}, {int(point[1])})", 
                (point[0] + 10, point[1] - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

# Initialize variables to calculate FPS
fps_start_time = time.time()
frame_count = 0

# Initialize the reference point
origin = np.array([0, 0], np.float32)

# Initialize moving average filters
gyro_data_buffer = {'gx': deque(maxlen=10), 'gy': deque(maxlen=10), 'gz': deque(maxlen=10)}

def moving_average(data_buffer, new_data):
    data_buffer.append(new_data)
    return sum(data_buffer) / len(data_buffer)

def calibrate_zero_angle():
    global gx_offset, gy_offset, gz_offset
    
    while True:
        gx, gy, gz = get_gyro_data(gx_offset, gy_offset, gz_offset)
        # print(f"Current Gyro Readings - GX: {gx:.2f}, GY: {gy:.2f}, GZ: {gz:.2f}")
        
        # Update offsets based on current gyro readings
        gx_offset = gx
        gy_offset = gy
        gz_offset = gz
        
        # Delay to avoid continuous updates (adjust as needed)
        time.sleep(0.1)

# Start the calibration thread
calibration_thread = threading.Thread(target=calibrate_zero_angle)
calibration_thread.start()

# Initialize variables to calculate FPS
fps_start_time = time.time()
frame_count = 0

# Initialize the reference point
origin = np.array([0, 0], np.float32)


# Initialize flags for falling detection
is_fallen_forward = False
is_fallen_backward = False
gyroscope_value_when_fallen = 0
fall_detected = False  # New flag to indicate fall detection

def check_fall(gy):
    global fall_detected, gyroscope_value_when_fallen, is_fallen_forward, is_fallen_backward, gx_offset, gy_offset, gz_offset
        # Define threshold values for falling forward and backward
    # gy_threshold_forward = 45  # Example threshold value for forward fall
    # gy_threshold_backward = -45  # Example threshold value for backward fall

    # Check condition for falling forward and send data for "Bangun Depan"
    if gy > gy_threshold_forward:
        fall_detected = True
        is_fallen_forward = True
        is_fallen_backward = False
        gyroscope_value_when_fallen = gy  # Save current gyroscope value when fallen
        TxData = 2
        send_data(TxData)
        time.sleep(2.5)
        print("Bangun Depan:", TxData)
        # for gy in range(10):
        #     gy = 0
        gx_offset, gy_offset, gz_offset = 0, 0, 0  # Resetting to zero (or neutral)
        # gyroscope_value_when_fallen = gy  # Save current gyroscope value when fallen
        # is_fallen_forward = False
        # is_fallen_backward = False
        # fall_detected = False  
        TxData = 256
        send_data(TxData)
        print('balance.')
        time.sleep(2.5)
        is_fallen_forward = False
        is_fallen_backward = False
        fall_detected = False  # Reset the fall detection flag
        # gx_offset, gy_offset, gz_offset = calibrate_gyro()
        # print("Gyro recalibrated after forward motion.")

        # gx_offset, gy_offset, gz_offset = calibrate_gyro()
        # print("Gyro recalibrated after backward motion.")

        
    # Check condition for falling backward and send data for "Bangun Belakang"
    elif gy < gy_threshold_backward:
        fall_detected = True
        is_fallen_backward = True
        is_fallen_forward = False
        gyroscope_value_when_fallen = gy  # Save current gyroscope value when fallen
        TxData = 1
        send_data(TxData)
        time.sleep(2.5)
        print("Bangun Belakang:", TxData)
        gx_offset, gy_offset, gz_offset = 0, 0, 0  # Resetting to zero (or neutral)
        TxData = 256
        send_data(TxData)
        print('balance.')
        time.sleep(2.5)
        is_fallen_forward = False
        is_fallen_backward = False
        fall_detected = False  # Reset the fall detection flag
        # gx_offset, gy_offset, gz_offset = calibrate_gyro()
        # print("Gyro recalibrated after backward motion.")

        # gx_offset, gy_offset, gz_offset = calibrate_gyro()
        # print("Gyro recalibrated after backward motion.")

    # Reset flags when the robot is balanced
    # elif gy_threshold_backward <= gy <= gy_threshold_forward:
    #     is_fallen_forward = False
    #     is_fallen_backward = False
time.sleep(1.5)
print("Resetting gyroscope readings to zero.")
gx_offset, gy_offset, gz_offset = 0, 0, 0  # Resetting to zero (or neutral)
print(f"Updated Gyroscope Readings after reset - GX: {gx_offset}, GY: {gy_offset}, GZ: {gz_offset}")

    # TxData = 256
    # send_data(TxData)
    # print('balance.')
        
    # is_fallen_forward = False
    # is_fallen_backward = False

while True:
    if ball_detection_active:
        # Logika deteksi bola (contoh dari kode Anda)
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame
        
        # Deteksi bola dilakukan di sini
        
        # if we are viewing a video and we did not grab a frame, then we have reached the end of the video
        if frame is None:
            break

        # Adjust brightness and contrast
        brightness = cv2.getTrackbarPos('Brightness', 'Calibration') / 100.0
        contrast = cv2.getTrackbarPos('Contrast', 'Calibration') / 100.0
        adjusted_frame = cv2.convertScaleAbs(frame, alpha=contrast, beta=brightness * 128)

        # resize the frame, blur it, and convert it to the HSV color space
        frame = imutils.resize(adjusted_frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Get HSV values from the trackbars
        hue_min = cv2.getTrackbarPos('Hue Min', 'Calibration')
        hue_max = cv2.getTrackbarPos('Hue Max', 'Calibration')
        saturation_min = cv2.getTrackbarPos('Saturation Min', 'Calibration')
        saturation_max = cv2.getTrackbarPos('Saturation Max', 'Calibration')
        value_min = cv2.getTrackbarPos('Value Min', 'Calibration')
        value_max = cv2.getTrackbarPos('Value Max', 'Calibration')

        # Get kernel size and area threshold from trackbars
        kernel_size = cv2.getTrackbarPos('Kernel Size', 'Calibration')
        if kernel_size % 2 == 0:  # Kernel size must be odd
            kernel_size += 1
        area_threshold = cv2.getTrackbarPos('Area Threshold', 'Calibration')

        mask = cv2.inRange(hsv, (hue_min, saturation_min, value_min),
                        (hue_max, saturation_max, value_max))

        # Morphological operations to remove noise
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame, then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                mapObjectPosition(center[0], center[1])

                # check the position of the ball and send the appropriate data
                if center[0] < 230:
                    TxData = 4
                    send_data(TxData)
                    print("Kiri :", TxData)
                # elif center[0] <= 295:
                #     TxData = 32
                #     send_data(TxData)
                #     print("Data :", TxData)
                # elif center[0] > 295 and center[0] < 305:
                #     TxData = 64
                #     send_data(TxData)
                #     print("Data :", TxData)
                elif center[0] >= 305:
                    TxData = 8
                    send_data(TxData)
                    print("Data :", TxData)

                # New center detection logic
                center_x_min = 270
                center_x_max = 330

                if center_x_min <= center[0] <= center_x_max:
                    TxData = 256  # Stop moving if the ball is centered
                    send_data(TxData)
                    print('Ball centered, stopping.')

                pts.appendleft(center)

        for i in range(1, len(pts)):
            if pts[i - 1] is None or pts[i] is None:
                continue
        # Draw main Cartesian coordinate axes
        draw_cartesian_axes(frame, (int(origin[0]), int(origin[1])))

            # Draw reference point and its value
        draw_reference_point(frame, (int(origin[0]), int(origin[1])))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        cv2.imshow("Ball Detection", frame)
        cv2.imshow('Mask', mask)

         # Set flag untuk berpindah ke pembacaan gyroscope
        ball_detection_active = False
        gyroscope_reading_active = True

    elif gyroscope_reading_active:
        # Logika pembacaan gyroscoget_gyro_datape (contoh dari kode Anda)
        gx, gy, gz = (gx_offset, gy_offset, gz_offset)
        print(f"Gyroscope readings - GX: {gx:.2f}, GY: {gy:.2f}, GZ: {gz:.2f}")

        # Check if the robot has fallen
        check_fall(gy)

        # Set flag untuk berpindah kembali ke deteksi bola
        gyroscope_reading_active = False
        ball_detection_active = True

    # Exit loop jika tombol 'q' ditekan
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# Release the camera and close all windows
if args.get("video", False):
    vs.release()
else:
    vs.stop()
cv2.destroyAllWindows()
