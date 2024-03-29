import logging
import time
import json
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Specify the uri of the drone to which we want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/34/2M/E7E7E7E7E7'

# Specify the variables we want to log (all at 100 Hz)
variables = [
    # State estimates (custom observer)
    'ae483log.o_x',
    'ae483log.o_y',
    'ae483log.o_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # State estimates (default observer)
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.yaw',
    'stateEstimate.pitch',
    'stateEstimate.roll',
    'stateEstimate.vx',
    'stateEstimate.vy',
    'stateEstimate.vz',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.n_x',
    'ae483log.n_y',
    'ae483log.r',
    'ae483log.a_z',
    # Setpoint (default controller)
    'ctrltarget.x',
    'ctrltarget.y',
    'ctrltarget.z',
    # Setpoint (custom controller)
    'ae483log.o_x_des',
    'ae483log.o_y_des',
    'ae483log.o_z_des',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
]


class SimpleClient:
    def __init__(self, uri, use_controller=True, use_observer=False):
        self.init_time = time.time()
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self.connected)
        self.cf.fully_connected.add_callback(self.fully_connected)
        self.cf.connection_failed.add_callback(self.connection_failed)
        self.cf.connection_lost.add_callback(self.connection_lost)
        self.cf.disconnected.add_callback(self.disconnected)
        print(f'Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_fully_connected = False
        self.data = {}

    def connected(self, uri):
        print(f'Connected to {uri}')
    
    def fully_connected(self, uri):
        print(f'Fully connected to {uri}')
        self.is_fully_connected = True

        # Reset the default observer
        self.cf.param.set_value('kalman.resetEstimation', 1)

        # Reset the ae483 observer
        self.cf.param.set_value('ae483par.reset_observer', 1)

        # Enable the controller (1 for default controller, 4 for ae483 controller)
        if self.use_controller:
            self.cf.param.set_value('stabilizer.controller', 4)
            self.cf.param.set_value('powerDist.motorSetEnable', 1)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)
            self.cf.param.set_value('powerDist.motorSetEnable', 0)

        # Enable the observer (0 for disable, 1 for enable)
        if self.use_observer:
            self.cf.param.set_value('ae483par.use_observer', 1)
        else:
            self.cf.param.set_value('ae483par.use_observer', 0)

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self.log_data)
                logconf.error_cb.add_callback(self.log_error)
                logconf.start()
            except KeyError as e:
                print(f'Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')

    def connection_failed(self, uri, msg):
        print(f'Connection to {uri} failed: {msg}')

    def connection_lost(self, uri, msg):
        print(f'Connection to {uri} lost: {msg}')

    def disconnected(self, uri):
        print(f'Disconnected from {uri}')
        self.is_fully_connected = False

    def log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp)
            self.data[v.name]['data'].append(data[v.name])

    def log_error(self, logconf, msg):
        print(f'Error when logging {logconf}: {msg}')

    def move(self, x, y, z, yaw, dt):
        print(f'Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)
    
    def move_smooth(self, p1, p2, yaw, speed):
        print(f'Move smoothly from {p1} to {p2} with yaw {yaw} degrees at {speed} meters / second')
        p1 = np.array(p1)
        p2 = np.array(p2)
        
        # Compute distance from p1 to p2
        distance_from_p1_to_p2 = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
        
        # Compute time it takes to move from p1 to p2 at desired speed
        time_from_p1_to_p2 = distance_from_p1_to_p2/speed
        
        start_time = time.time()
        while True:
            current_time = time.time()
            
            # Compute what fraction of the distance from p1 to p2 should have
            # been travelled by the current time
            s = (current_time-start_time)*speed/distance_from_p1_to_p2
            
            # Compute where the drone should be at the current time, in the
            # coordinates of the world frame
            p = np.array([p1[0] + s*(p2[0] - p1[0]), p1[1] + s*(p2[1] - p1[1]), p1[2] + s*(p2[2] - p1[2])])
            
            self.cf.commander.send_position_setpoint(p[0], p[1], p[2], yaw)
            if s >= 1:
                return
            else:
                time.sleep(0.1)

    def stop(self, dt):
        print(f'Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def disconnect(self):
        self.cf.close_link()

    def write_data(self, filename='logged_data.json'):
        with open(filename, 'w') as outfile:
            json.dump(self.data, outfile, indent=4, sort_keys=False)


if __name__ == '__main__':
    # Initialize everything
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    client = SimpleClient(uri, use_controller=True, use_observer=True)
    while not client.is_fully_connected:
        time.sleep(0.1)

    # Leave time at the start to initialize
    client.stop(1.0)

    # PLANE
    client.move(0.0, 0.0, 0.15, 0.0, 1.0)
    client.move_smooth([0.0, 0.0, 0.15], [0.0, 0.0, 0.35], 0.0, 0.2)
    client.move(0.0, 0.0, 0.35, 0.0, 1.0)

    client.move_smooth([0.0, 0.0, .35], [0.48606811145510836, 0.9936708860759493, 0.35], 0.0, 0.2)
    client.move(0.48606811145510836, 0.9936708860759493, 0.35, 0.0, 0.5)
    client.move_smooth([0.48606811145510836, 0.9936708860759493, .35], [0.5139318885448917, 1.0, 0.35], 0.0, 0.2)
    client.move(0.5139318885448917, 1.0, .35, 0.0, 0.5)
    client.move_smooth([0.5139318885448917, 1.0, .35], [0.5789473684210527, 0.8860759493670886, 0.35], 0.0, 0.2)
    client.move(0.5789473684210527, 0.8860759493670886, .35, 0.0, 0.5)
    client.move_smooth([0.5789473684210527, 0.8860759493670886, .35], [0.5882352941176471, 0.6962025316455697, 0.35], 0.0, 0.2)
    client.move(0.5882352941176471, 0.6962025316455697, .35, 0.0, 0.5)
    client.move_smooth([0.5882352941176471, 0.6962025316455697, .35], [0.7987616099071208, 0.560126582278481, 0.35], 0.0, 0.2)
    client.move(0.7987616099071208, 0.560126582278481, .35, 0.0, 0.5)
    client.move_smooth([0.7987616099071208, 0.560126582278481, .35], [1.0, 0.4240506329113924, 0.35], 0.0, 0.2)
    client.move(1.0, 0.4240506329113924, .35, 0.0, 0.5)
    client.move_smooth([1.0, 0.4240506329113924, .35], [1.0, 0.36075949367088606, 0.35], 0.0, 0.2)
    client.move(1.0, 0.36075949367088606, .35, 0.0, 0.5)
    client.move_smooth([1.0, 0.36075949367088606, .35], [0.7585139318885449, 0.4177215189873418, 0.35], 0.0, 0.2)
    client.move(0.7585139318885449, 0.4177215189873418, .35, 0.0, 0.5)
    client.move_smooth([0.7585139318885449, 0.4177215189873418, .35], [0.5789473684210527, 0.4651898734177215, 0.35], 0.0, 0.2)
    client.move(0.5789473684210527, 0.4651898734177215, .35, 0.0, 0.5)
    client.move_smooth([0.5789473684210527, 0.4651898734177215, .35], [0.56656346749226, 0.17405063291139242, 0.35], 0.0, 0.2)
    client.move(0.56656346749226, 0.17405063291139242, .35, 0.0, 0.5)
    client.move_smooth([0.56656346749226, 0.17405063291139242, .35], [0.6873065015479877, 0.04430379746835443, 0.35], 0.0, 0.2)
    client.move(0.6873065015479877, 0.04430379746835443, .35, 0.0, 0.5)
    client.move_smooth([0.6873065015479877, 0.04430379746835443, .35], [0.6904024767801857, 0.012658227848101266, 0.35], 0.0, 0.2)
    client.move(0.6904024767801857, 0.012658227848101266, .35, 0.0, 0.5)
    client.move_smooth([0.6904024767801857, 0.012658227848101266, .35], [0.5325077399380805, 0.056962025316455694, 0.35], 0.0, 0.2)
    client.move(0.5325077399380805, 0.056962025316455694, .35, 0.0, 0.5)
    client.move_smooth([0.5325077399380805, 0.056962025316455694, .35], [0.5139318885448917, 0.015822784810126583, 0.35], 0.0, 0.2)
    client.move(0.5139318885448917, 0.015822784810126583, .35, 0.0, 0.5)
    client.move_smooth([0.5139318885448917, 0.015822784810126583, .35], [0.49226006191950467, 0.012658227848101266, 0.35], 0.0, 0.2)
    client.move(0.49226006191950467, 0.012658227848101266, .35, 0.0, 0.5)
    client.move_smooth([0.49226006191950467, 0.012658227848101266, .35], [0.4674922600619195, 0.05379746835443038, 0.35], 0.0, 0.2)
    client.move(0.4674922600619195, 0.05379746835443038, .35, 0.0, 0.5)
    client.move_smooth([0.4674922600619195, 0.05379746835443038, .35], [0.3219814241486068, 0.0, 0.35], 0.0, 0.2)
    client.move(0.3219814241486068, 0.0, .35, 0.0, 0.5)
    client.move_smooth([0.3219814241486068, 0.0, .35], [0.3157894736842105, 0.0379746835443038, 0.35], 0.0, 0.2)
    client.move(0.3157894736842105, 0.0379746835443038, .35, 0.0, 0.5)
    client.move_smooth([0.3157894736842105, 0.0379746835443038, .35], [0.43034055727554177, 0.16455696202531644, 0.35], 0.0, 0.2)
    client.move(0.43034055727554177, 0.16455696202531644, .35, 0.0, 0.5)
    client.move_smooth([0.43034055727554177, 0.16455696202531644, .35], [0.42105263157894735, 0.4620253164556962, 0.35], 0.0, 0.2)
    client.move(0.42105263157894735, 0.4620253164556962, .35, 0.0, 0.5)
    client.move_smooth([0.42105263157894735, 0.4620253164556962, .35], [0.2260061919504644, 0.41455696202531644, 0.35], 0.0, 0.2)
    client.move(0.2260061919504644, 0.41455696202531644, .35, 0.0, 0.5)
    client.move_smooth([0.2260061919504644, 0.41455696202531644, .35], [0.0030959752321981426, 0.35443037974683544, 0.35], 0.0, 0.2)
    client.move(0.0030959752321981426, 0.35443037974683544, .35, 0.0, 0.5)
    client.move_smooth([0.0030959752321981426, 0.35443037974683544, .35], [0.0, 0.4240506329113924, 0.35], 0.0, 0.2)
    client.move(0.0, 0.4240506329113924, .35, 0.0, 0.5)
    client.move_smooth([0.0, 0.4240506329113924, .35], [0.20743034055727555, 0.560126582278481, 0.35], 0.0, 0.2)
    client.move(0.20743034055727555, 0.560126582278481, .35, 0.0, 0.5)
    client.move_smooth([0.20743034055727555, 0.560126582278481, .35], [0.4086687306501548, 0.6930379746835443, 0.35], 0.0, 0.2)
    client.move(0.4086687306501548, 0.6930379746835443, .35, 0.0, 0.5)
    client.move_smooth([0.4086687306501548, 0.6930379746835443, .35], [0.43343653250773995, 0.9367088607594937, 0.35], 0.0, 0.2)
    client.move(0.43343653250773995, 0.9367088607594937, .35, 0.0, 0.5)

    client.move_smooth([0.43343653250773995, 0.9367088607594937, 0.35], [0.43343653250773995, 0.9367088607594937, 0.15], 0.0, 0.2)
    client.move(0.43343653250773995, 0.9367088607594937, 0.15, 0.0, 1.0)

    # Land
    client.stop(1.0)

    # Disconnect from drone
    client.disconnect()

    # Write data from flight
    client.write_data('hardware_data.json')