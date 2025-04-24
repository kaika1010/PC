import time
import threading
import RPi.GPIO as GPIO
from RPLCD.gpio import CharLCD
import smbus2
import os
from utils import check_posture_angle
from lcd_menu import LCDMenu

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)

# Button Pins
PAGE_UP = 17
PAGE_DOWN = 27
SELECT = 22
GPIO.setup(PAGE_UP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PAGE_DOWN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SELECT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Vibration Motors controlled by transistors
MOTOR_1 = 23
MOTOR_2 = 24
GPIO.setup(MOTOR_1, GPIO.OUT)
GPIO.setup(MOTOR_2, GPIO.OUT)

# LCD Setup (4-bit mode)
lcd = CharLCD(
    numbering_mode=GPIO.BCM,
    cols=16,
    rows=2,
    pin_rs=26,
    pin_e=19,
    pins_data=[14, 18, 0, 11]  # D4=14, D5=18, D6=0, D7=11
)

# Backlight control
BACKLIGHT_PIN = 13  # Control via transistor base through 2kΩ resistor
GPIO.setup(BACKLIGHT_PIN, GPIO.OUT)
GPIO.output(BACKLIGHT_PIN, GPIO.HIGH)

# I2C Gyroscope Setup
I2C_ADDR = 0x68
bus = smbus2.SMBus(1)
bus.write_byte_data(I2C_ADDR, 0x6B, 0)  # Wake sensor

# Global variables
last_activity = time.time()
posture_corrections = 0
wifi_enabled = False
menu = LCDMenu(lcd)

# Functions
def read_gyro():
    high = bus.read_byte_data(I2C_ADDR, 0x43)
    low = bus.read_byte_data(I2C_ADDR, 0x44)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value / 131.0

def update_idle_screen():
    global posture_corrections
    angle = read_gyro()
    if abs(angle) > 10:
        lcd.clear()
        lcd.write_string("! Correct Posture")
        GPIO.output(MOTOR_1, GPIO.HIGH)
        GPIO.output(MOTOR_2, GPIO.HIGH)
        posture_corrections += 1
        time.sleep(1)
        GPIO.output(MOTOR_1, GPIO.LOW)
        GPIO.output(MOTOR_2, GPIO.LOW)
    else:
        lcd.clear()
        lcd.write_string("\u2714 Posture OK")

def check_buttons():
    global last_activity, wifi_enabled, posture_corrections
    if GPIO.input(PAGE_UP) == GPIO.LOW:
        last_activity = time.time()
        menu.page_up()
        time.sleep(0.3)
    elif GPIO.input(PAGE_DOWN) == GPIO.LOW:
        last_activity = time.time()
        menu.page_down()
        time.sleep(0.3)
    elif GPIO.input(SELECT) == GPIO.LOW:
        last_activity = time.time()
        if not menu.active:
            menu.enter(posture_corrections, wifi_enabled)
        else:
            selection = menu.select()
            if selection == 'Reset Counter':
                posture_corrections = 0
            elif selection == 'Toggle WiFi':
                wifi_enabled = not wifi_enabled
                if wifi_enabled:
                    os.system("sudo systemctl start hostapd dnsmasq")
                else:
                    os.system("sudo systemctl stop hostapd dnsmasq")

# Backlight auto-off

def backlight_handler():
    while True:
        if time.time() - last_activity > 120:
            GPIO.output(BACKLIGHT_PIN, GPIO.LOW)
        else:
            GPIO.output(BACKLIGHT_PIN, GPIO.HIGH)
        time.sleep(5)

# Start backlight manager thread
threading.Thread(target=backlight_handler, daemon=True).start()

# Main loop
try:
    while True:
        if not menu.active:
            update_idle_screen()
        check_buttons()
        time.sleep(1)
except KeyboardInterrupt:
    lcd.clear()
    GPIO.cleanup()
