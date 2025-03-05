import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QComboBox, QStackedWidget, QLineEdit, QPushButton, QLabel, QHBoxLayout
from PyQt5.QtCore import QRect
from PyQt5.QtGui import QIcon

import paho.mqtt.client as mqtt

def publish(topic, message):
    MQClient.publish(topic, message)

MQClient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
MQClient.connect("192.168.1.85", 1883, 60)
MQClient.loop_start()

class DynamicLayoutWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle('Contol Panel')
        self.setGeometry(100, 100, 300, 400)  
        
        

        
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        
        
        self.main_layout = QVBoxLayout(self.main_widget)
        
        
        self.combo_box = QComboBox()
        self.combo_box.addItem('Go forward')
        self.combo_box.addItem('Tune')
        self.combo_box.addItem('Robot locate at')
        self.combo_box.currentIndexChanged.connect(self.switch_layout)
        
        self.main_layout.addWidget(self.combo_box)
        
       
        self.stacked_widget = QStackedWidget()
        self.main_layout.addWidget(self.stacked_widget)
        
        # Layout 1:"move" and "rotate"
        self.layout1 = QWidget()
        self.layout1_vbox = QVBoxLayout(self.layout1)
        
        self.move_label = QLabel('move')
        self.line_edit1_1 = QLineEdit()
        self.line_edit1_1.setGeometry(QRect(0, 0, 100, 20))  
        self.rotate_label = QLabel('rotate')
        self.line_edit1_2 = QLineEdit()
        self.line_edit1_2.setGeometry(QRect(0, 0, 100, 20))  
        
        self.layout1_move_layout = QHBoxLayout()
        self.layout1_move_layout.addWidget(self.move_label)
        self.layout1_move_layout.addWidget(self.line_edit1_1)
        
        self.layout1_rotate_layout = QHBoxLayout()
        self.layout1_rotate_layout.addWidget(self.rotate_label)
        self.layout1_rotate_layout.addWidget(self.line_edit1_2)
        
        self.layout1_vbox.addLayout(self.layout1_move_layout)
        self.layout1_vbox.addLayout(self.layout1_rotate_layout)
        
        self.submit_button1 = QPushButton('Go and Plot')
        self.submit_button1.clicked.connect(self.submit_layout1)
        self.layout1_vbox.addWidget(self.submit_button1)
        
        # Layout 2: "kp position", "kd position", "kp velocity", "ki velocity", "kb"
        self.layout2 = QWidget()
        self.layout2_vbox = QVBoxLayout(self.layout2)
        
        self.kp_position_label = QLabel('kp position')
        self.line_edit2_1 = QLineEdit()
        self.line_edit2_1.setGeometry(QRect(0, 0, 100, 20))  
        self.kd_position_label = QLabel('kd position')
        self.line_edit2_2 = QLineEdit()
        self.line_edit2_2.setGeometry(QRect(0, 0, 100, 20))  
        self.kp_velocity_label = QLabel('kp velocity')
        self.line_edit2_3 = QLineEdit()
        self.line_edit2_3.setGeometry(QRect(0, 0, 100, 20))  
        self.ki_velocity_label = QLabel('ki velocity')
        self.line_edit2_4 = QLineEdit()
        self.line_edit2_4.setGeometry(QRect(0, 0, 100, 20))  
        self.kb_label = QLabel('kb')
        self.line_edit2_5 = QLineEdit()
        self.line_edit2_5.setGeometry(QRect(0, 0, 100, 20))  
        
        self.layout2_vbox.addWidget(self.kp_position_label)
        self.layout2_vbox.addWidget(self.line_edit2_1)
        self.layout2_vbox.addWidget(self.kd_position_label)
        self.layout2_vbox.addWidget(self.line_edit2_2)
        self.layout2_vbox.addWidget(self.kp_velocity_label)
        self.layout2_vbox.addWidget(self.line_edit2_3)
        self.layout2_vbox.addWidget(self.ki_velocity_label)
        self.layout2_vbox.addWidget(self.line_edit2_4)
        self.layout2_vbox.addWidget(self.kb_label)
        self.layout2_vbox.addWidget(self.line_edit2_5)
        
        self.submit_button2 = QPushButton('Tune')
        self.submit_button2.clicked.connect(self.submit_layout2)
        self.layout2_vbox.addWidget(self.submit_button2)
        
        # Layout 3: "x" and "y"
        self.layout3 = QWidget()
        self.layout3_vbox = QVBoxLayout(self.layout3)
        
        self.x_label = QLabel('x')
        self.line_edit3_1 = QLineEdit()
        self.line_edit3_1.setGeometry(QRect(0, 0, 100, 20))  
        self.y_label = QLabel('y')
        self.line_edit3_2 = QLineEdit()
        self.line_edit3_2.setGeometry(QRect(0, 0, 100, 20))  
        
        self.layout3_x_layout = QHBoxLayout()
        self.layout3_x_layout.addWidget(self.x_label)
        self.layout3_x_layout.addWidget(self.line_edit3_1)
        
        self.layout3_y_layout = QHBoxLayout()
        self.layout3_y_layout.addWidget(self.y_label)
        self.layout3_y_layout.addWidget(self.line_edit3_2)
        
        self.layout3_vbox.addLayout(self.layout3_x_layout)
        self.layout3_vbox.addLayout(self.layout3_y_layout)
        
        self.submit_button3 = QPushButton('Go and Plot')
        self.submit_button3.clicked.connect(self.submit_layout3)
        self.layout3_vbox.addWidget(self.submit_button3)
        
       
        self.stacked_widget.addWidget(self.layout1)
        self.stacked_widget.addWidget(self.layout2)
        self.stacked_widget.addWidget(self.layout3)
        
       
        self.stacked_widget.setCurrentIndex(0)
    
    def switch_layout(self, index):
        self.stacked_widget.setCurrentIndex(index)
        
    def submit_layout1(self):
        text1 = self.line_edit1_1.text()
        text2 = self.line_edit1_2.text()
        text1 = float(text1) if self.is_float(text1) else 0
        text2 = float(text2) if self.is_float(text2) else 0
        cmd=str(text1) + ',' + str(text2)
        publish("systemCommand", cmd)
        
        print("Layout 1 Content:")
        print("Move:", text1)
        print("Rotate:", text2)
        
    def submit_layout2(self):
        text1 = self.line_edit2_1.text()
        text2 = self.line_edit2_2.text()
        text3 = self.line_edit2_3.text()
        text4 = self.line_edit2_4.text()
        text5 = self.line_edit2_5.text()
        text1 = float(text1) if self.is_float(text1) else -1
        text2 = float(text2) if self.is_float(text2) else -1
        text3 = float(text3) if self.is_float(text3) else -1
        text4 = float(text4) if self.is_float(text4) else -1
        text5 = float(text5) if self.is_float(text5) else -1
        cmd=str(text1) + ',' + str(text2) + ',' + str(text3) + ',' + str(text4) + ',' + str(text5)
        publish("setCoeffs", cmd)
        print("Layout 2 Content:")
        print("kp position:", text1)
        print("kd position:", text2)
        print("kp velocity:", text3)
        print("ki velocity:", text4)
        print("kb:", text5)
        
    def submit_layout3(self):
        text1 = self.line_edit3_1.text()
        text2 = self.line_edit3_2.text()
        text1 = float(text1) if self.is_float(text1) else -1
        text2 = float(text2) if self.is_float(text2) else -1
        cmd=str(text1) + ',' + str(text2)
        publish("robotLocate", cmd)
        print("Layout 3 Content:")
        print("x:", text1)
        print("y:", text2)
        
    def is_float(self, value):
        try:
            float(value)
            return True
        except ValueError:
            return False

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DynamicLayoutWindow()
    window.show()
    sys.exit(app.exec_())
