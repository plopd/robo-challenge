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
        self.distance_to_goal_threshold = 0
        self.robot_radius = radius
        self.correction = False
        self.step_size = 4
        self.direction = "forward"
        self.game_state = None
        self.collected_points_count = 0

    @property
    def current_location(self):
        return Point(self.game_state["robot"]["x"], self.game_state["robot"]["y"])


    def on_map(self):
        current_location = self.current_location
        return abs(current_location.x) > self.robot_radius and abs(current_location.y) > self.robot_radius and \
        abs(current_location.x-self.game_state["world"]["x_max"]) > self.robot_radius and \
        abs(current_location.y-self.game_state["world"]["y_max"]) > self.robot_radius


    def find_closest(self):
        min_dist = math.inf
        min_point = Point(-1,-1)
        current_point = self.current_location
        for p in self.game_state["points"]:
            if p["collected"] == False and p["score"] > 0:
                point = Point(p["x"], p["y"])
                d = Point.dist(current_point, point)
                if d < min_dist:
                    min_dist = d
                    min_point = point
        return min_point

    def count_current_collected_points(self):
        ct = 0
        for p in self.game_state["points"]:
            if p["collected"] == True and p["score"] > 0:
                ct += 1
        return ct

    def correct(self, robot_state):
        print(" ===> CORRECT YOURSELF!")
        self.theta += robot_state["angle"]

    def flip_direction(self):
        if self.state == 'forward':
            self.state = 'backwards'
        else:
            self.state = 'forward'
        if self.theta <= 180:
            self.theta += 180
        else:
            self.theta -= 180

    def update(self, game_state):
        # self.last_tracked_point = self.current_location
        self.game_state = game_state
        if self.state == 'ready':
            # find next point to hunt
            # TODO: might calibrate here, otherwise just switch to target state.
            self.state = 'target'
        elif self.state == 'target':
            print("TARGET MODE")
            # track next point
            self.target_point = self.find_closest()
            self.state = 'hunt'
            correction = False
            self.step_size = 5
            self.go_to(self.target_point)
        elif self.state == 'hunt':
            print("HUNT MODE")
            # TODO: check if target_point was lost, then need to switch back to target state.
            current_distance = Point.dist(self.current_location, self.target_point)
            print("distance to goal: %d" % current_distance)
            if current_distance - self.last_distance_to_goal > self.distance_to_goal_threshold:
                print("DISTANCE TO TARGET INCREASING !!!!!!")
                stop()
                #correction = True
                self.state = 'target'
            tmp_cnt = self.count_current_collected_points()
            if self.collected_points_count < tmp_cnt:
                stop()
                self.state = 'target'
                self.collected_points_count = tmp_cnt
            self.last_distance_to_goal = current_distance

            # check for boundary
            if not self.on_map(game_state):
                print("FOUND WALL")
                # move backwards



    def go_to(self, game_state, target_point):
        current_point = self.current_location(game_state)
        print("Go from %s to %s" % (current_point.to_string(), target_point.to_string()))

        self.state = "hunt"
        self.last_tracked_point = current_point
        d_trans = Point.dist(target_point, current_point)
        d_rot = math.atan2(target_point.y - current_point.y, target_point.x - current_point.x) - self.theta

        d_rot_deg = int(math.degrees(d_rot))
        d_trans = int(d_trans*self.step_size)
        print("\tturn %d degrees and drive %d units" % (d_rot_deg, d_trans))

        turn(d_rot_deg)
        if self.direction == 'forward':
            move_forward(d_trans)
        else:
            move_backward(d_trans)


'''
def calibrate(game_state):
    global current_point, target_dist
    # from current position go to a random sufficiently distant position.
    current_point = Point(game_state["robot"]["x"], game_state["robot"]["y"])
    target_point = Point(400,400)
    target_point = Point(1000,400)
    target_dist = Point.dist(current_point, target_point)
    move({'x': current_point.x, 'y': current_point.y, 'theta': 0},{'x': target_point.x, 'y': target_point.y})
'''


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
    print("turn by %d degrees %s" % (angle_deg, "left" if angle_deg > 0 else "right"))
    if angle_deg > 0:
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
