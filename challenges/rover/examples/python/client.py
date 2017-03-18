import paho.mqtt.client as mqtt
import json
import math
# from game_logic import Logic


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @staticmethod
    def dist(a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    def to_dict(self):
        return dict(x=self.x, y=self.y)



class Logic:
    def __init__(self, max_stat_count=0, stat_err=0, fact_step=5):
        self.available = list()                      # List of all unotuched pos pts
        self.max_stat_count = max_stat_count         # if after max_stat_count the position did nto change, im stationary.
        self.stat_count = max_stat_count             # current_counter for finding stationary
        self.robot = None                            # Point of current position of robot.
        self.rob_angle = 0                           # Absolute orientation of robot in radians.

        self.stat_err = stat_err                     # tolerance for stationary, if its below this you increment stat_count.
        self.angle_err = 0                           # if difference target rotation and current rotation is (-eps, eps) then dont turn.
        self.target = None                           # Point as the target where robot wants to move.
        self.factor_step = fact_step                 # figure it out with initial calibration. If you have a target dist d you multiply it by this factor to make up for the noise in the real robot.

    def update(self, data):
        robo = Point(data['robot']['x'], data['robot']['y'])
        ava = [Point(p['x'], p['y']) for p in data['points'] if p['collected'] is False and p['score'] > 0]
        # print (robo.x, robo.y)
        if self.robot: #if its not the first update.
            stat = self.stationary(robo)
            self.rob_angle = math.atan2(robo.y - self.robot.y, robo.x - self.robot.x)
        else:
            stat = True
            self.available = ava
        self.robot = robo

        if stat:
            self.stat_count += 1
        else:
            self.stat_count = 0
        if (len(ava) < len(self.available)) or self.stat_count >= self.max_stat_count: # found or stopped
            self.available = ava
            p = self.find_closest()
            stop()
            # TODO: move a little to find your orientation
            self.move(p)
        else:
        # elif self.target and Point.dist(self.robot, self.target) > self.init_dist:
        #     self.move(self.target)
            self.available = ava

    def stationary(self, p):
        if abs(p.x-self.robot.x) > self.stat_err or abs(p.y-self.robot.y) > self.stat_err:
            return False
        else:
            return True

    def move(self, p):
        print("MOVE")
        print("robot", self.robot.x, self.robot.y)
        print("target", p.x, p.y)

        self.target = p
        d_trans = Point.dist(self.robot, p)
        self.init_dist = d_trans #
        d_rot = math.atan2(p.y - self.robot.y, p.x - self.robot.x) - self.rob_angle
        d_rot_deg = math.degrees(d_rot) % 360
        print("robangle", self.rob_angle,"rad ",  math.degrees(self.rob_angle),"deg")
        print ("rot_degree", d_rot_deg)
        if d_rot_deg > 180:
            d_rot_deg = -360+d_rot_deg

        # print ('rad-ang ', d_rot)
        turn(int(d_rot_deg), self.angle_err)
        move_forward(int(d_trans*self.factor_step))
        # self.rob_angle += d_rot

    def find_closest(self):
        mind = 2**31
        minp = None
        for p in self.available:
            d = Point.dist(self.robot, p)
            # print ("distance = ", d)
            if d < mind:
                mind = d
                minp = p
        return minp




SERVER = "127.0.0.1"
# SERVER = "10.10.10.30"
PORT = 1883
PLAYER_NAME = "RoboHackHustlers"

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe('players/' + PLAYER_NAME + '/#')
    client.subscribe('robot/state')
    client.publish('players/' + PLAYER_NAME, json.dumps({"command": "register"}))
    client.subscribe('players/' + PLAYER_NAME + '/incoming')

cur = dict()
cur['theta'] = 0
moving = False

logic = Logic(5, fact_step=8)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global cur, moving, logic
    # print(msg.topic)
    obj = json.loads(msg.payload.decode("utf-8"))
    # print(obj)

    if 'incoming' in msg.topic:
        print(obj)
        client.publish('players/' + PLAYER_NAME, json.dumps({"command": "start"}))
        logic.rob_angle=0
    elif 'game' in msg.topic and not moving:
        # cur['x'] = obj['robot']['x']
        # cur['y'] = obj['robot']['x']
        # move(cur, {'x':400, 'y':400})
        # moving = True
        if 'robot' in obj:
            logic.update(obj)
        #
def move_forward(dist):
    print("dist ", dist)
    client.publish('robot/process', json.dumps({"command": "forward", "args": dist}))

def move_backward(dist):
    client.publish('robot/process', json.dumps({"command": "backward", "args": dist}))

def turn(angle, err=0):
    print("angle ", angle)
    if angle < -err:
        client.publish('robot/process', json.dumps({"command": "right", "args": -angle}))
    elif angle > err:
        client.publish('robot/process', json.dumps({"command": "left", "args": angle }))

def stop():
    client.publish('robot/process', json.dumps({"command": "stop"}))

def move(c, g):
    d_trans = math.sqrt((g['x']-c['x'])**2 + (g['y']-c['y'])**2)
    d_rot = math.atan2(g['y'] - c['y'], g['x'] - c['x']) - c['theta']
    d_rot_deg = math.degrees(d_rot)

    turn(int(d_rot_deg))
    move_forward(int(d_trans*10))
    #return c['theta'] + d_rot

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
except (KeyboardInterrupt, SystemExit):
    print("Tearing down...")
    client.disconnect()
    raise
