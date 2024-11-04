import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
import pyqtgraph as pg
import numpy as np
from PyQt5.QtCore import QTimer
import serial
import struct

class FFTDisplay(QMainWindow):
    def __init__(self):
        super().__init__()
        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                timeout=1
            )
        except serial.SerialException as e:
            print(f"串口打开失败: {e}")
            sys.exit(1)
            
        # 初始化FFT数据存储列表
        self.fft_data = []
        self.bin_nums = []
            
        self.initUI()
        
    def initUI(self):
        # 创建绘图窗口
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
        
        # 设置图表标题和标签
        self.graphWidget.setTitle("实时FFT频谱", color="w", size="20pt")
        self.graphWidget.setLabel('left', '幅值')
        self.graphWidget.setLabel('bottom', '频率 (Hz)')
        
        # 设置画布背景
        self.graphWidget.setBackground('k')
        
        # 创建曲线对象
        self.curve = self.graphWidget.plot(pen='y')
        
        # 设置窗口大小和标题
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('FFT频谱显示器')
        
        # 创建定时器用于更新数据
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # 每100ms更新一次
        
    def update_plot(self):
        try:
            line = self.serial_port.readline().decode().strip()
            # print(line)
            if line.startswith("FFT bin"):
                # 解析FFT数据行
                parts = line.split(": ")
                # print(parts)
                if len(parts) == 2:
                    bin_num = int(parts[0].split()[2])
                    value = float(parts[1])
                    print(bin_num, value)
                    print("len:",len(self.fft_data))
                    
                    # 如果是新的一组FFT数据开始
                    if bin_num == 0:
                        self.fft_data = []
                        self.bin_nums = []
                    
                    self.fft_data.append(value)
                    self.bin_nums.append(bin_num)
                    
                    # 当收集到足够的数据点时更新图表 
                    if len(self.fft_data) >= 1024/2:  # 根据C代码修改为N_SAMPLES/2
                        print("Total:",self.fft_data,self.bin_nums)
                        freq = np.array(self.bin_nums) * (20000/1024)  # 采样率20kHz, FFT点数1024
                        self.curve.setData(freq, self.fft_data)
                        self.fft_data = []
                        self.bin_nums = []
                        
        except Exception as e:
            print(f"数据读取错误: {e}")

    def closeEvent(self, event):
        # 关闭窗口时清理串口
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
        event.accept()

def main():
    app = QApplication(sys.argv)
    ex = FFTDisplay()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 