from gpiozero import Button
# you can check pinout via $pinout in shell
pin = 14
button = Button(pin) 
print(f"Status pin {pin} is {not button.is_pressed}")