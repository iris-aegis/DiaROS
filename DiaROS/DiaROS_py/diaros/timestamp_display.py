import time
import sys

class TimestampDisplay:
    def __init__(self):
        self.timestamp = ""

    def update(self, new_timestamp):
        self.timestamp = new_timestamp

    def run(self):
        while True:
            # sys.stdout.write('OK.\n')
            if self.timestamp:
                print(f"[Timestamp] {self.timestamp}")
                # sys.stdout.write('OK.\n')
                # sys.stdout.flush()
                self.timestamp = ""  # 表示したらリセット
            time.sleep(0.5)
