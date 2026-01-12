# ==============================
# Author: sqplab
# Date: 2025-09-23
# Description: RC 스위치로 오프보드 모드 토글
# ==============================

from std_msgs.msg import Bool

class RCSwitchTrigger(Node):
    def __init__(self):
        super().__init__('rc_switch_trigger')
        self.subscription = self.create_subscription(
            RCIn, '/mavros/rc/in', self.rc_callback, 10)
        self.trigger_pub = self.create_publisher(Bool, '/offboard_start', 10)
        self.prev_state = False

    def rc_callback(self, msg):
        ch7 = msg.channels[5]
        active = ch7 > 1700

        if active != self.prev_state:
            self.prev_state = active
            msg_out = Bool()
            msg_out.data = active
            self.trigger_pub.publish(msg_out)
            self.get_logger().info(f"Offboard {'ON' if active else 'OFF'}")


"""
self.offboard_enabled = False
self.create_subscription(Bool, '/offboard_start', self.offboard_start_cb, 10)

def offboard_start_cb(self, msg):
    self.offboard_enabled = msg.data



and Timercallback 에서
if not self.offboard_enabled:
    # offboard 모드 진입 / setpoint publish를 잠시 멈춤
    return

"""