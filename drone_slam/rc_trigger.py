import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn
import subprocess

class RCSwitchTrigger(Node):
	def __init__(self):
		super().__init__('rc_switch_trigger')
		self.subscription = self.create_subscription(
			RCIn,
			'/mavros/rc/in',
			self.rc_callback,
			10)
		self.prev_triggered = False
		
	def rc_callback(self, msg):
		ch7 = msg.channels[5]
		
		if ch7 > 1700 and not self.prev_triggered:
			self.get_logger().info("offboard flight start...")
			subprocess.Popen(['python3', 'src/SQPLab-UAV/drone_slam/offboard_mission.py'])
			self.prev_triggered = True
		elif ch7 < 1500 and self.prev_triggered:
			self.prev_triggered = False
			
		
def main(args=None):
	rclpy.init(args=args)
	node = RCSwitchTrigger()
	rclpy.spin(node)
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
