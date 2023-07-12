# STATE requirements:
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive

# COMMAND requirements:
from bosdyn.client.docking import blocking_undock, blocking_dock_robot
from bosdyn.client.time_sync import TimeSyncClient, TimeSyncThread
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient

# GENERAL requirements:
import time
import bosdyn.client
from bosdyn.client import create_standard_sdk, RpcError
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.api.graph_nav import graph_nav_pb2

'______________________________________________________________________________________________________________________________________________________'

class Robot:
    def __init__(self, username, password, robot_ip, robot_name, sdk_name):
        self.sdk = bosdyn.client.create_standard_sdk(sdk_name)
        self.robot = self.sdk.create_robot(robot_ip, robot_name)
        self.robot.authenticate(username, password)

        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self.acquire_lease()

        self.time_sync_client = self.robot.ensure_client(TimeSyncClient.default_service_name)
        self.establish_time_sync()

    def acquire_lease(self):
        try:
            if self.lease_client.lease_wallet.has_lease():
                print("Another client has the lease. Attempting to take it...")
                self.lease_client.take()
            else:
                print("No other client has the lease. Acquiring it...")
            
            self.lease = self.lease_client.acquire()
            self.lease_keep_alive = LeaseKeepAlive(self.lease_client, return_at_exit=True)
            print("Lease acquired.")
        except Exception as e:
            print("Failed to acquire lease: ", e)

    def establish_time_sync(self):
        try:
            self.time_sync_thread = TimeSyncThread(self.time_sync_client)
            self.time_sync_thread.start()

            if not self.time_sync_thread.wait_for_sync(timeout_sec=10.0):
                print("Failed to achieve time sync")
            else:
                print("Time sync established")
                self.timesync_endpoint = self.time_sync_thread.time_sync_endpoint
        except Exception as e:
            print("Failed to establish time sync: ", e)


class TrajectoryPoint:
    def __init__(self, timestamp, position, velocity):
        self.timestamp = timestamp
        self.position = position
        self.velocity = velocity

class Trajectory:
    def __init__(self):
        self.points = []

    def add_point(self, timestamp, position, velocity):
        point = TrajectoryPoint(timestamp, position, velocity)
        self.points.append(point)

    def get_velocity_at_time(self, timestamp):
        for point in self.points:
            if point.timestamp == timestamp:
                return point.velocity
        return None


class RobotTrajectoryRetriever:
    def __init__(self, robot):
        # set robot object
        self.robot = robot
        # set up state 
        self.state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self.trajectory = Trajectory()


    def record_trajectory(self, duration):
        start_time = time.time()
        prev_position = None
        prev_timestamp = None

        while time.time() - start_time < duration:
            state = self.state_client.get_robot_state()
            # check how long the code below takes to run because this could affect the timestamp that is recorded below and the accuracy of the trajectory
            position = state.kinematic_state.transforms_snapshot.child_to_parent_transform['odom_tform_body'].position
            timestamp = state.kinematic_state.transforms_snapshot.acquisition_time # this is the time at which the state was recorded

            if prev_position is not None:
               dx = position.x - prev_position.x
               dy = position.y - prev_position.y
               dz = position.z - prev_position.z
               dt = timestamp - prev_timestamp

               velocity  = (dx / dt, dy /dt, dz / dt)
            else:
                velocity = (0, 0, 0)

            prev_position = position
            prev_timestamp = timestamp
            self.trajectory.add_point(timestamp, position, velocity)
            time.sleep(0.1)

class Controller:
    def __init__(self, robot):
        self.lease_client = robot.robot.lease_client
        self.lease_keep_alive = robot.robot.lease_keep_alive
        self.estop_client = robot.robot.estop_client
        self.estop_keep_alive = robot.robot.estop_keep_alive
        self.time_sync_endpoint = robot.robot.time_sync_endpoint
        self.command_client = robot.robot.ensure_client(RobotCommandClient.default_service_name)
        self.trajectory = robot.trajectory


    def dock_robot(robot, dock_id):
        try:
            blocking_dock_robot(robot, dock_id)
            print("Docking successful.")
        except Exception as e:
            print("Docking failed with error:", e)

    def undock_robot(robot):
        try:
            blocking_undock(robot)
            print("Undocking successful.")
        except Exception as e:
            print("Undocking failed with error:", e)


class Mission:
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.commands = []

    def generate_commands(self):
        prev_timestamp = None
        for point in self.trajectory.points:
            if prev_timestamp is not None:
                dt = point.timestamp - prev_timestamp
                command = RobotCommandBuilder.synchro_velocity_command(point.velocity[0], point.velocity[1], point.velocity[2])
                self.commands.append(command)
            prev_timestamp = point.timestamp


# create spot object 
spot_test = Robot('admin', 'fovxbyqujxha', '192.168.8.104', 'spot-BD-30030001', 'Scheduling_App')
spot_test.acquire_lease()
spot_test.establish_time_sync()




'''
class GraphNav:
    def __init__(self, robot_ip, user, password):
        # Initialize SDK and Robot clients
        self.sdk = create_standard_sdk('GraphNavClient')
        self.robot = self.sdk.create_robot(robot_ip)
        self.robot.authenticate(user, password)

        # Time sync
        self.time_sync_endpoint = TimeSyncEndpoint(self.robot)
        self.time_sync_thread = TimeSyncThread(self.time_sync_endpoint)
        self.time_sync_thread.start()
        self.time_sync_thread.wait_for_sync()

        # Acquire lease
        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self.lease = self.lease_client.acquire()
        self.lease_keep_alive = LeaseKeepAlive(self.lease_client)

        # Initialize GraphNav and RobotCommand clients
        self.graph_nav_client = self.robot.ensure_client(GraphNavClient.default_service_name)
        self.robot_command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)

    def download_graph(self):
        # Download the graph from the robot
        self.graph = self.graph_nav_client.download_graph()

    def build_route(self):
        # Extract waypoints and edges from the graph
        waypoints = self.graph.waypoints
        edges = self.graph.edges

        # Create a route using the waypoints and edges
        self.route = GraphNavClient.build_route([wp.id for wp in waypoints], [edge.id for edge in edges])

    def navigate_route(self):
        # Command the robot to navigate the route
        command = RobotCommandBuilder.synchro_se2_trajectory_point_command(self.route)
        self.robot_command_client.robot_command(command)
'''


'''
graph_nav = GraphNav('ROBOT_IP', 'USER', 'PASSWORD')
graph_nav.download_graph()
graph_nav.build_route()
graph_nav.navigate_route()
graph_nav.shutdown()
'''