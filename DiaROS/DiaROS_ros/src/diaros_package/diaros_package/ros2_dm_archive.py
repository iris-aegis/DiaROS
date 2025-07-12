# import rclpy
# import threading
# import sys
# from interfaces.msg import Iasr
# from interfaces.msg import Imm
# from rclpy.node import Node

# """
# 対話管理モジュールを組み込む場合利用する
# (現在は言語理解部から流れてきたデータを応答制御へ流しているだけ)
# """

# class RosDialogManagement(Node):
#     def __init__(self, dialogManagement):
#         super().__init__('dialog_management')
#         self.dialogManagement = dialogManagement
#         self.sub_lu = self.create_subscription(Iasr, 'LUtoDM', self.send, 1)
#         self.pub_dm = self.create_publisher(Iasr, 'DMtoRC', 1)
#         self.pub_mm = self.create_publisher(Imm, 'MM', 1)
#         self.timer = self.create_timer(1, self.ping)
#         sys.stdout.write('DialogManagement start up.\n')
#         sys.stdout.write('=====================================================\n')

#     def send(self, lu):
#         nlg = Iasr()
#         nlg.you = lu.you
#         nlg.is_final = lu.is_final
#         self.pub_dm.publish(nlg)
        
#     def ping(self):
#         mm = Imm()
#         mm.mod = "dm"
#         self.pub_mm.publish(mm)

# def runROS(pub):
#     rclpy.spin(pub)

# def shutdown():
#     while True:
#         key = input()
#         if key == "kill":
#             print("kill command received.")
#             sys.exit()

# def main(args=None):
#     dm = ""
#     rclpy.init(args=args)
#     rdm = RosDialogManagement(dm)

#     ros = threading.Thread(target=runROS, args=(rdm,))

#     ros.setDaemon(True)

#     ros.start()
#     shutdown()

# if __name__ == '__main__':
#     main()