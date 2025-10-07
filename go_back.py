import keyboard
from gpiozero import DigitalOutputDevice
from time import sleep

# GPIOの定義
in1 = DigitalOutputDevice(17)  # モーター制御1
in2 = DigitalOutputDevice(27)  # モーター制御2

print("W：前進｜S：後退｜Q：停止｜Ctrl+Cで終了")

try:
    while True:
        if keyboard.is_pressed('w'):
            print("前進")
            in1.on()
            in2.off()

        elif keyboard.is_pressed('s'):
            print("後退")
            in1.off()
            in2.on()

        elif keyboard.is_pressed('q'):
            print("停止")
            in1.off()
            in2.off()

        sleep(0.1)  # CPU負荷を軽減

except KeyboardInterrupt:
    print("終了します")
    in1.off()
    in2.off()
