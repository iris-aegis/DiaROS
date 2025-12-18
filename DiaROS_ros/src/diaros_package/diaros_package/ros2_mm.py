import rclpy
import threading
import sys
import requests
import json
from rclpy.node import Node
from interfaces.msg import Imm

class RosModuleManager(Node):
    def __init__(self):
        super().__init__('module_manager')
        self.status = { 
            "sm":  False,
            "lu":  False,
            "dm":  False,
            "rc":  False,
            "nlg": False,
            "ss":  False,
            "dr":  False
            }
        self.sub = self.create_subscription(Imm, 'MM', self.statusUpdate, 1)
        self.timer = self.create_timer(1, self.update)
        sys.stdout.write('ModuleManager start up.\n')
        sys.stdout.write('=====================================================\n')

    def statusUpdate(self, mod):
        modName = mod.mod
        self.status[modName] = True

    def update(self):
        sys.stdout.write(f"{self.status}\n")
        sys.stdout.flush()

        data = { 'data': json.dumps(self.status) }
        requests.post('http://localhost:3000/modstatus', data)

        for key in self.status.keys():
            self.status[key] = False
        
        
        
def runROS(pub):
    rclpy.spin(pub)

def shutdown():
    while True:
        key = sys.stdin.readline().strip()
        if key == "kill":
            sys.stdout.write("kill command received.\n")
            sys.stdout.flush()
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    rmm = RosModuleManager()

    ros = threading.Thread(target=runROS, args=(rmm,))

    ros.setDaemon(True)

    ros.start()
    shutdown()

if __name__ == '__main__':
    main()
    