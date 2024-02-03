import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class LeadVehicleControl(Node):

    def __init__(self):
        super().__init__('lead_vehicle_control')


				self.go_forward(

    def go_forward(self, speed):

def main(args=None):
    rclpy.init(args=args)

    lead_vehicle_control = LeadVehicleControl()

    while (find_actor_by_rolename(world, VEHICLE_NAME) == None):
				try:
						print("Waiting for ego vehicle to spawn....", end="\r")
				except KeyboardInterrupt:
						lead_vehicle.destroy()

    rclpy.spin(lead_vehicle_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lead_vehicle_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
