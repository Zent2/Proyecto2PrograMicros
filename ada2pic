# Import Adafruit IO REST client.
from Adafruit_IO import Client, Feed, Data, RequestError
import datetime, serial, time

# Set to your Adafruit IO key.
# Remember, your key is a secret,
# so make sure not to publish it when you publish this code!
ADAFRUIT_IO_KEY = "CONTRASEÑA"

# Set to your Adafruit IO username.
# (go to https://accounts.adafruit.com to find your username)
ADAFRUIT_IO_USERNAME = "USUARIO"

# Create an instance of the REST client.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)
try:
    Servo1 = aio.feeds('servo1')
    Servo2 = aio.feeds('servo2')
    Servo3 = aio.feeds('servo3')
    Servo4 = aio.feeds('servo4')
except RequestError:
    feed = Feed(name="servo1")
    Servo1 = aio.create_feed(feed)
    feed2 = Feed(name="servo2")
    Servo2 = aio.create_feed(feed2)
    feed3 = Feed(name="servo3")
    Servo3 = aio.create_feed(feed3)
    feed4 = Feed(name="servo4")
    Servo4 = aio.create_feed(feed4)

import serial   

with serial.Serial() as ser:    # EUSART config
    ser.baudrate = 9600
    ser.port = 'COM9'

while True:                     # Ciclo principal
    time.sleep(0.5)               # Delay 500ms
    ser.open()                  
    
    base = int(aio.receive(Servo1.key).value)  # Recibir datos de adafruit
    if base < 100:                              # Centenas = 0
        ser.write(b'0')
    if base < 10:                               # Decenas = 0
        ser.write(b'0')
        
    ser.write(bytes(str(base), 'utf-8'))        # Escribir el valor al puerto serial   
    time.sleep(0.5)               # Delay 500ms
    
    Brazo = int(aio.receive(Servo2.key).value)  # Recibir datos de adafruit
    if Brazo < 100:                              # Centenas = 0
        ser.write(b'0')
    if Brazo < 10:                               # Decenas = 0
        ser.write(b'0')
        
    ser.write(bytes(str(Brazo), 'utf-8'))        # Escribir el valor al puerto serial  
    time.sleep(0.5)               # Delay 500ms    
    
    antebrazo = int(aio.receive(Servo3.key).value)  # Recibir datos de adafruit
    if antebrazo < 100:                              # Centenas = 0
        ser.write(b'0')
    if antebrazo < 10:                               # Decenas = 0
        ser.write(b'0')
        
    ser.write(bytes(str(antebrazo), 'utf-8'))        # Escribir el valor al puerto serial  
    time.sleep(0.5)               # Delay 500ms    
    
    Garra= int(aio.receive(Servo4.key).value)  # Recibir datos de adafruit
    if Garra < 100:                              # Centenas = 0
        ser.write(b'0')
    if Garra < 10:                               # Decenas = 0
        ser.write(b'0')
    
    ser.write(bytes(str(Garra), 'utf-8'))        # Escribir el valor al puerto serial  
    
    
    # Imprimir los valores recibidos en la consola
    print("Base:", base)
    print("Brazo:", Brazo)
    print("Antebrazo:", antebrazo)
    print("Garra:", Garra)
    #data2 = ser.read()                          # Leer datos del puerto serial
    #aio.send_data(counter2.key, int(ord(data2.decode('utf_8', 'strict'))))      # Enviar datos a adafruit
    
    ser.close()

