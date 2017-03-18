import paho.mqtt.client as mqtt
import json
import math
from queue import Queue
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
    def __init__(self, hist_size=10):
        # self.found = list()
        # self.bad = list()
        self.available = list()                      # List of all unotuched pos pts
        self.hist_size = hist_size                 # int, no. op history pos
        self.history = Queue(self.hist_size)       # stores history positions of the robot
        self.robot = None                          # Current position of the robot
        self.rob_angle = 0
        self.start_pos = None

    def update(self, data):
        # print('bla')
        # print (data)
        robo = Point(data['robot']['x'], data['robot']['y'])
        ava = [Point(p['x'], p['y']) for p in data['points'] if p['collected'] is False and p['score'] > 0]
        # found = [Point(p['x'], p['y']) for p in data['points'] if p['collected'] and p['score'] == 1]

        if self.history.full():
            self.history.get()
        self.history.put(robo)
        self.robot = robo         # Cur
        print (self.history.qsize(), "robot", self.robot.x, self.robot.y)
        print ("stationary", self.is_stationary())
        print ("found ", len(self.available) - len(ava))

        if self.history.full() and ((len(ava) < len(self.available)) or self.is_stationary(0)): # found or stopped
            p = self.find_closest()
            self.history = Queue(self.hist_size)
            self.move(p)
        self.available = ava

    def is_stationary(self, error=0):
        for p in list(self.history.queue):
            if abs(p.x-self.robot.x) > error or abs(p.y-self.robot.y) > error:
                return False
        return True

    def move(self, p):
        self.start_pos = self.robot
        print ("moving ")
        print (p.x, p.y)
        # move(self.robot.to_dict(), p.to_dict())
        d_trans = Point.dist(self.robot, p)
        d_rot = math.atan2(p.y - self.robot.y, p.x - self.robot.x) - self.rob_angle
        d_rot_deg = math.degrees(d_rot)

        turn(int(d_rot_deg))
        move_forward(int(d_trans))
        self.rob_angle += d_rot

    def find_closest(self):
        mind = 2**31
        minp = None
        for p in self.available:
            d = Point.dist(self.robot, p)
            # print ("distance = ", d)
            if d < mind:
                mind = d
                minp = p
        # print ("min dist ",  mind)
        return minp




SERVER = "127.0.0.1"
PORT = 1883
PLAYER_NAME = "foo"


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

logic = Logic(5)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global cur, moving, logic
    # print(msg.topic)
    obj = json.loads(msg.payload.decode("utf-8"))
    # print(obj)

    if 'incoming' in msg.topic:
        print(obj)
        client.publish('players/' + PLAYER_NAME, json.dumps({"command": "start"}))
    elif 'game' in msg.topic and not moving:
        # cur['x'] = obj['robot']['x']
        # cur['y'] = obj['robot']['x']
        # move(cur, {'x':700, 'y':600})
        # moving = True
        if 'robot' in obj:
            logic.update(obj)
        
def move_forward(dist):
    print("move for")
    client.publish('robot/process', json.dumps({"command": "forward", "args": dist}))

def move_backward(dist):
    client.publish('robot/process', json.dumps({"command": "backward", "args": dist}))

def turn(angle):
    if angle >= 0:
        print("pos ang")
        client.publish('robot/process', json.dumps({"command": "right", "args": angle}))
    else:
        print("neg ang")
        client.publish('robot/process', json.dumps({"command": "left", "args": 360 + angle }))

def move(c, g):
    d_trans = math.sqrt((g['x']-c['x'])**2 + (g['y']-c['y'])**2)
    d_rot = math.atan2(g['y'] - c['y'], g['x'] - c['x']) - c['theta']
    d_rot_deg = math.degrees(d_rot)
    
    turn(int(d_rot_deg))
    move_forward(int(d_trans*10))
    return c['theta'] + d_rot
    
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
