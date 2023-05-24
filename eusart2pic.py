from PySide2.QtWidgets import QApplication, QMainWindow, QPushButton, QSlider
from PySide2.QtUiTools import QUiLoader
import serial, datetime, time

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        # Cargar la interfaz gráfica desde el archivo .ui
        loader = QUiLoader()
        self.ui = loader.load('eusart2pic2.ui')
        self.setCentralWidget(self.ui)

        # Conectar el botón "Enviar" a la función sendValues
        self.send_button = self.ui.findChild(QPushButton, 'pushButton')
        self.send_button.clicked.connect(self.sendValues)

        # Configurar los sliders para que vayan de 0 a 255
        self.slider_base = self.ui.findChild(QSlider, 'horizontalSlider_4')
        self.slider_base.setMinimum(0)
        self.slider_base.setMaximum(255)

        self.slider_brazo = self.ui.findChild(QSlider, 'horizontalSlider_3')
        self.slider_brazo.setMinimum(0)
        self.slider_brazo.setMaximum(255)

        self.slider_antebrazo = self.ui.findChild(QSlider, 'horizontalSlider')
        self.slider_antebrazo.setMinimum(0)
        self.slider_antebrazo.setMaximum(255)

        self.slider_garra = self.ui.findChild(QSlider, 'horizontalSlider_2')
        self.slider_garra.setMinimum(0)
        self.slider_garra.setMaximum(255)

    def sendValues(self):
        # Obtener los valores de los sliders
        base_value = self.slider_base.value()
        brazo_value = self.slider_brazo.value()
        antebrazo_value = self.slider_antebrazo.value()
        garra_value = self.slider_garra.value()

        # Crear el mensaje a enviar
        message = f"{base_value},{brazo_value},{antebrazo_value},{garra_value}\n"

        # Imprimir los valores en la terminal
        print(f"Valores enviados: {message}")

        # Enviar el mensaje al COM9 con baud rate de 9600
        with serial.Serial('COM9', 9600) as ser:
            time.sleep(0.5)               # Delay 500ms
            #ser.open() 
            #Mandamos el valor de la base
            if base_value < 100:                              # Centenas = 0
                ser.write(b'0')
            if base_value < 10:                               # Decenas = 0
                ser.write(b'0')
           
            ser.write(bytes(str(base_value), 'utf-8'))        # Escribir el valor al puerto serial
            time.sleep(0.5)               # Delay 500ms
            #Mandamos el valor del brazo
            if brazo_value < 100:                              # Centenas = 0
                ser.write(b'0')
            if brazo_value < 10:                               # Decenas = 0
                ser.write(b'0')
           
            ser.write(bytes(str(brazo_value), 'utf-8'))        # Escribir el valor al puerto serial
            time.sleep(0.5)               # Delay 500ms
            #Mandamos el valor del antebrazo
            if antebrazo_value < 100:                              # Centenas = 0
                ser.write(b'0')
            if antebrazo_value < 10:                               # Decenas = 0
                ser.write(b'0')
            
            ser.write(bytes(str(antebrazo_value), 'utf-8'))        # Escribir el valor al puerto serial
            time.sleep(0.5)               # Delay 500ms
            #Mandamos el valor de la garra
            if garra_value < 100:                              # Centenas = 0
                ser.write(b'0')
            if garra_value < 10:                               # Decenas = 0
                ser.write(b'0')
            
            ser.write(bytes(str(garra_value), 'utf-8'))        # Escribir el valor al puerto serial
            
            ser.close()

if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
