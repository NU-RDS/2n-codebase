
import time
import math

class MockComm:
    def __init__(self):
        self.angle = 0.0
        self.opened = False

    def open_serial(self):
        self.opened = True
        print("[MockComm] 打开(假的)串口")

    def close_serial(self):
        self.opened = False
        print("[MockComm] 关闭(假的)串口")

    def is_open(self):
        return self.opened

    def read_data(self):
        """
        返回类似 "45.0,30.0" 这样的字符串数据。
        假设每次调用就让角度做一点变化，模拟动态关节角。
        """
        if not self.opened:
            return ""

        self.angle += 3.0
        j1 = 45.0 * math.sin(math.radians(self.angle))
        j2 = 30.0 * math.cos(math.radians(self.angle))

        # 拼成字符串
        return f"{j1:.1f},{j2:.1f}"

    def write_data(self, msg: str):
        print(f"[MockComm] (假发送) -> {msg}")
        time.sleep(0.001)
