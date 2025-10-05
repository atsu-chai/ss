import RPi.GPIO as GPIO
import time

# GPIOピン番号（BCMモード）
IN1 = 23
IN2 = 24

def setup():
    GPIO.setmode(GPIO.BCM)  # BCM番号で設定
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)

def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    print("モータが正転しています")

def backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    print("モータが逆転しています")

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    print("モータが停止しました")

def loop():
    while True:
        forward()
        time.sleep(2)

        backward()
        time.sleep(2)

        stop()
        time.sleep(2)

def cleanup():
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        print("\n終了処理中...")
        cleanup()
