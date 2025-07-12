### ros2_turn_taking.py ###
import rclpy
import threading
import sys
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from diaros.turnTaking import TurnTaking, push_audio_data, turn_taking_result_queue  # TurnTakingを実行するために読み込み
from interfaces.msg import Itt

class RosTurnTaking(Node):
    def __init__(self):
        super().__init__('turn_taking')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'mic_audio_float32',
            self.listener_callback,
            10
        )
        self.pub_tt = self.create_publisher(Itt, 'TTtoDM', 10)
        self.timer = self.create_timer(0.1, self.publish_turn_taking)
        self.recv_count = 0  # 受信回数カウンタ追加
        self.get_logger().info('[ros2_turn_taking] Listening to mic_audio_float32...')

    def listener_callback(self, msg):
        audio_np = np.array(msg.data, dtype=np.float32)
        push_audio_data(audio_np)
        self.recv_count += 1
        first_val = audio_np[0] if len(audio_np) > 0 else None
        # sys.stdout.write(f"[ros2_turn_taking] Received mic_audio_float32 #{self.recv_count} (len={len(audio_np)}) first={first_val}\n")
        # sys.stdout.flush()
        # print(f"[ros2_turn_taking] received buffer size: {len(audio_np)}")

    def publish_turn_taking(self):
        if not turn_taking_result_queue.empty():
            (result_value, confidence) = turn_taking_result_queue.get()
            msg = Itt()
            msg.result = result_value
            msg.confidence = confidence
            self.pub_tt.publish(msg)
            # self.get_logger().info(f'[RosTurnTaking] Published TT = {result_value}, conf={confidence}')


def runROS(node):
    rclpy.spin(node)

def runTurnTaking():
    TurnTaking()  # ここでモデル実行箇所

def shutdown():
    while True:
        key = input()
        if key == "kill":
            print("kill command received.")
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    node = RosTurnTaking()

    ros = threading.Thread(target=runROS, args=(node,))
    mod = threading.Thread(target=runTurnTaking)

    ros.setDaemon(True)
    mod.setDaemon(True)

    ros.start()
    mod.start()
    shutdown()

if __name__ == '__main__':
    main()
