"""
Battery Monitor - 배터리 모니터링
"""
from PyQt5.QtCore import QThread, pyqtSignal
import random
import time


class BatteryMonitor(QThread):
    """배터리 모니터 스레드"""
    
    battery_updated = pyqtSignal(float, float)
    
    def __init__(self):
        super().__init__()
        self.running = False
        self.percentage = 85.0
        self.voltage = 12.6
    
    def run(self):
        """스레드 실행"""
        self.running = True
        
        while self.running:
            # 시뮬레이션: 배터리 서서히 감소
            self.percentage = max(0, self.percentage - random.uniform(0, 0.1))
            self.voltage = 10.5 + (self.percentage / 100.0) * 2.1
            
            self.battery_updated.emit(self.percentage, self.voltage)
            time.sleep(1.0)
    
    def stop(self):
        """스레드 중지"""
        self.running = False
        self.wait()
