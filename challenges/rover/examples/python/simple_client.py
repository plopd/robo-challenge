import paho.mqtt.client as mqtt
import json
import math

MAGIC_FUCKING_HIDDEN_NUMBER_IN_SIMULATOR = 3.328125
SERVER = "127.0.0.1"
# SERVER = "10.10.10.30"
PORT = 1883
PLAYER_NAME = "RoboHackHustlers"
moving = False
robot = None

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @staticmethod
    def dist(a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    def to_dict(self):
        return dict(x=self.x, y=self.y)

    def to_string(self):
        return str(self.x) + "," + str(self.y)


#############
### ROBOT ###
#############
class Robot:
    def __init__(self, x = 640, y = 480, theta = 0, radius = 0):
        self.last_tracked_point = Point(x,y)
        self.theta = theta
        self.state = "ready" # ready, target, hunt, stop
        self.target_point = None
        self.last_distance_to_goal = 0
        self.distance_to_goal_threshold = 1
        self.robot_radius = radius
        self.correction = False

    def current_location(self, game_state):
        return Point(game_state["robot"]["x"], game_state["robot"]["y"])

    def on_map(self, game_state):
        return Point.dist(self.current_location(game_state), Point(game_state["world"]["x_max"], game_state["world"]["x_max"])) > self.robot_radius

    def find_closest(self, game_state):
        min_dist = math.inf
        min_point = Point(-1,-1)
        current_point = self.current_location(game_state)
        for p in game_state["points"]:
            if p["collected"] == False:
                point = Point(p["x"], p["y"])
                d = Point.dist(current_point, point)
                if d < min_dist:
                    min_dist = d
                    min_point = point
        return min_point

    def correct(self, robot_state):
        self.theta += robot_state["angle"]

    def update(self, game_state):
        if self.state == 'ready':
            # find next point to hunt
            # might calibrate here, otherwise just switch to target state.
            self.state = 'target'
        elif self.state == 'target':
            # track next point
            self.target_point = self.find_closest(game_state)
            self.state = 'hunt'
            correction = False
            self.go_to(game_state, self.target_point)
        elif self.state == 'hunt':
            # TODO: check if target_point was lost, then need to switch back to target state.
            current_distance = Point.dist(self.current_location(game_state), self.target_point)
            if current_distance - self.last_distance_to_goal > self.distance_to_goal_threshold or \
            not self.on_map(game_state) or \
            current_distance <= self.robot_radius: # found point
                # issue stop command and switch to target state.
                stop()
                correction = True
                self.state = 'target'
            self.last_distance_to_goal = current_distance

    def go_to(self, game_state, target_point):
        current_point = self.current_location(game_state)
        print("Go from %s to %s" % (current_point.to_string(), self.target_point.to_string()))

        self.state = "hunt"
        self.last_tracked_point = current_point
        d_trans = Point.dist(target_point, current_point)
        d_rot = math.atan2(target_point.y - current_point.y, target_point.x - current_point.x) - self.theta

        d_rot_deg = int(math.degrees(d_rot))
        d_trans = int(d_trans*8)
        print("turn %d degrees and drive %d units" % (d_rot_deg, d_trans))

        turn(d_rot_deg)
        move_forward(d_trans)


def calibrate(game_state):
    global current_point, target_dist
    # from current position go to a random sufficiently distant position.
    current_point = Point(game_state["robot"]["x"], game_state["robot"]["y"])
    target_point = Point(400,400)
    target_point = Point(1000,400)
    target_dist = Point.dist(current_point, target_point)
    move({'x': current_point.x, 'y': current_point.y, 'theta': 0},{'x': target_point.x, 'y': target_point.y})



###############
### CLIENT ####
###############

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe('players/' + PLAYER_NAME + '/#')
    client.subscribe('robot/state')
    client.publish('players/' + PLAYER_NAME, json.dumps({"command": "register"}))
    client.subscribe('players/' + PLAYER_NAME + '/incoming')


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    # print(msg.topic)
    obj = json.loads(msg.payload.decode("utf-8"))
    # print(obj)

    if 'incoming' in msg.topic:
        client.publish('players/' + PLAYER_NAME, json.dumps({"command": "start"}))
    if 'game' in msg.topic:
        '''
        if not calibration_started:
            calibrate(obj)
            print("CALIBRATE")
            calibration_started = True
        else:
            final_distance = Point.dist(Point(obj["robot"]["x"], obj["robot"]["y"]), current_point)
            print("FINAL DISTANCE: %d TARGET DISTANCE: %d" %(final_distance, target_dist))
            distance_ratio = target_dist * 1.0 / final_distance
            print("DISTANCE_RATIO %.5lf" % distance_ratio)
        '''
        robot.update(obj)
    if 'state' in msg.topic and robot.correction:
        robot.correct(obj)

def move_forward(dist):
    print("move forward dist units", dist)
    client.publish('robot/process', json.dumps({"command": "forward", "args": dist}))

def move_backward(dist):
    print("move backward dist units", dist)
    client.publish('robot/process', json.dumps({"command": "backward", "args": dist}))

def turn(angle_deg):
    print("turn by %d degrees %s" % (angle_deg, "left" if angle_deg < 0 else "right"))
    if angle_deg < 0:
        client.publish('robot/process', json.dumps({"command": "left", "args": angle_deg}))
    else:
        client.publish('robot/process', json.dumps({"command": "right", "args": angle_deg}))

def stop():
    client.publish('robot/process', json.dumps({"command": "stop"}))



robot = Robot(x = 640, y = 480, theta = 0, radius = 15)
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(SERVER, PORT, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
try:
    client.loop_forever()
    client.publish('robot/process', json.dumps({"command" : "reset"}))
except (KeyboardInterrupt, SystemExit):
    print("Tearing down...")
    client.disconnect()
    raise
