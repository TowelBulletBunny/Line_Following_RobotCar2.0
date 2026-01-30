import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import time

# --- MOTOR SETUP ---
GPIO.setmode(GPIO.BCM)
ENA, IN1, IN2 = 12, 23, 24
ENB, IN3, IN4 = 13, 17, 27

GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)
pwmA = GPIO.PWM(ENA, 1000); pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0); pwmB.start(0)

last_error = 0

# --- CAMERA SETUP ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (160, 120)})
picam2.configure(config)
picam2.start()

def get_line_error():
    """Captures a frame and returns horizontal error or 'FINISH'."""
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    roi = gray[75:115, 0:160]  
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    
    # This turns black tape into WHITE pixels (value 255)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # 1. Calculate Moments
    M = cv2.moments(thresh)
    
    # 2. FIX: Convert raw sum to actual Pixel Count
    # Raw Area was 425850.0. 425850 / 255 = 1670 pixels.
    pixel_count = M['m00'] / 255  

    print(f"Pixels Seen: {pixel_count}", flush=True)

    # 3. Check for Finish Line 
    # (The ROI is 40x160 = 6400 pixels total. 3500 is a good half-way mark)
    if pixel_count > 5000:
        return "FINISH"

    # 4. Check for normal line center
    if pixel_count > 400:
        cx = int(M['m10'] / M['m00']) # Use raw M['m00'] for the math here
        return cx - 80  
    else:
        return None

def move_robot(error):
    global last_error
    derivative = error - last_error
    last_error = error 

    if abs(error) > 30:
        spin_power = 85 
        if error > 0: # Spin Right
            GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)
            GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
        else:        # Spin Left
            GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)
        pwmA.ChangeDutyCycle(spin_power)
        pwmB.ChangeDutyCycle(spin_power)
    else:
        # Normal P+D Steering
        base_speed = 45 
        Kp, Kd = 1.4, 0.8
        steering = (error * Kp) + (derivative * Kd)
        GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)
        pwmA.ChangeDutyCycle(max(0, min(100, base_speed + steering)))
        pwmB.ChangeDutyCycle(max(0, min(100, base_speed - steering)))

def search_for_line():
    global last_error
    spin_speed = 90
    if last_error > 0:
        GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    else:
        GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)
    pwmA.ChangeDutyCycle(spin_speed)
    pwmB.ChangeDutyCycle(spin_speed)

def stop_motors():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwmA.ChangeDutyCycle(0); pwmB.ChangeDutyCycle(0)

# --- MAIN LOOP ---
try:
    while True:
        # ONE call to camera per loop (saves 50% CPU time)
        result = get_line_error()
        
        status_img = np.zeros((100, 400, 3), dtype="uint8")

        if result == "FINISH":
            print("Finish line detected!")
            stop_motors()
            break # Exit loop
        
        elif result is not None:
            # result is the numerical error
            error = result
            if abs(error) > 25: 
                last_error = error
            move_robot(error)
            txt, clr = f"Line Found! Error: {error}", (0, 255, 0)
        else:
            search_for_line()
            txt, clr = f"LOST! Last Error: {last_error}", (0, 0, 255)
        
        cv2.putText(status_img, txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, clr, 2)
        cv2.imshow("Robot Data", status_img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'): break

except KeyboardInterrupt:
    print("User stopped the robot.")
finally:
    stop_motors()
    pwmA.stop(); pwmB.stop()
    picam2.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
