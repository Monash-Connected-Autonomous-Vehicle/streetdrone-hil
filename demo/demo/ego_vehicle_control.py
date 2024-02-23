import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import String
from carla_msgs.msg import CarlaEgoVehicleControl
import carla


class EgoVehicleControl(Node):
				def __init__(self):
								super().__init__('ego_vehicle_control')
								self.publisher_ = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
								timer_period = 0.1  # seconds
								self.timer = self.create_timer(timer_period, self.timer_callback)

				def timer_callback(self):
								msg = CarlaEgoVehicleControl()
								msg.throttle = 0.2
								msg.steer = 0.0
								self.publisher_.publish(msg)
								self.get_logger().info('Publishing: "%s"' % msg.throttle)

def find_actor_by_rolename(world, role_name_tofind):  # searches all actors in a carla world looking for a specific role name and returns object if found
    actors = world.get_actors()
    
    actors = actors.filter('vehicle.*')  # filter out just the vehicle actors

    if (actors):
        for actor in actors:
            role_name = "None"
            if 'role_name' in actor.attributes:
                if (actor.attributes['role_name'] == role_name_tofind):
                    return actor
        return None
    else:
        return None

def calc_dist(actor_a, actor_b):  # calculates distance in xy plane between two actors
    loc_a = actor_a.get_location()
    loc_b = actor_b.get_location()
    return math.sqrt((loc_a.x - loc_b.x) ** 2 + (loc_a.y - loc_b.y) ** 2 + (loc_a.z - loc_b.z) ** 2)


def main(args=None):
        rclpy.init(args=args)

        ego_vehicle_control = EgoVehicleControl()
        VEHICLE_MODEL = 'vehicle.toyota.prius'
        LEAD_VEHICLE_NAME = 'lead_vehicle'
        EGO_VEHICLE_NAME = 'ego_vehicle'
        SAFETY_DIST = 20

        client = carla.Client('localhost', 2000)  # create client to connect to simulator
        client.set_timeout(10.0)
        world = client.get_world()

        print('Sucessfully connected and retrieved carla world.')

        # set spectator camera to a birds view of testing area
        # get the spectator actor which is a spectator that controls camera view
        # in simulation window (window that appears when you start carla)
        #spectator = world.get_spectator()
        #spectator.set_transform(carla.Transform(carla.Location(SPEC_CAM_X, SPEC_CAM_Y, SPEC_CAM_Z),
                                                #carla.Rotation(SPEC_CAM_PITCH, SPEC_CAM_YAW, SPEC_CAM_ROLL)))

        #world.set_weather(carla.WeatherParameters())  # set default weather

        # get blueprint library which is used for creating actors
        blueprint_library = world.get_blueprint_library()

        # select a blueprint for our lead vehicle
        lead_vehicle_bp = next(bp for bp in blueprint_library if bp.id == VEHICLE_MODEL)

        # set lead vehicle role_name attribute to reflect lead_vehicle so that
        # it's easily distinguishable in debugging
        lead_vehicle_bp.set_attribute('role_name', LEAD_VEHICLE_NAME)

        spawn_loc = carla.Location(50, 0, 2)
        rotation = carla.Rotation(0, 90, 0)
        transform = carla.Transform(spawn_loc, rotation)


        # spawn the vehicle
        lead_vehicle = world.spawn_actor(lead_vehicle_bp, transform)
        # turn all vehicle lights on so its more visible in nighttime tests
        lead_vehicle.set_light_state(carla.VehicleLightState.All)

        # wait for ego vehicle to spawn

        
        while (find_actor_by_rolename(world, EGO_VEHICLE_NAME) == None):
            try:
                print("Waiting for ego vehicle to spawn....", end="\r")
            except KeyboardInterrupt:
                lead_vehicle.destroy()

        ego_vehicle = find_actor_by_rolename(world, EGO_VEHICLE_NAME)
        print("Found ego vehicle!")

        while (calc_dist(lead_vehicle, ego_vehicle) > SAFETY_DIST):
            try:
                print(
                    "Waiting for ego vehcile to enter within trigger distance. Current distance: %im | Safety Distance Distance %im" % (
                    calc_dist(lead_vehicle, ego_vehicle), SAFETY_DIST), end="\r")
            except KeyboardInterrupt:
                lead_vehicle.destroy()

        print("")
        print("Ego vehicle entered safety distance!")
        lead_vehicle.set_target_velocity(carla.Vector3D(0, 0.5, 0))  # set target velocity for lead vehicle


        rclpy.spin(ego_vehicle_control)

        ego_vehicle_control.destroy_node()
        rclpy.shutdown()

        time.sleep(5)
         # set target velocity for lead vehicle
        lead_vehicle.destroy()

if __name__ == '__main__':
				main()
