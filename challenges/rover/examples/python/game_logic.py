from client import move
from queue import Queue
import math


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @staticmethod
    def dist(a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.Y)**2)

    def to_dict(self):
        return dict(x=self.x, y=self.y)

class Logic:
    def __init__(self, hist_size=100):
        # self.found = list()
        # self.bad = list()
        self.available = None                      # List of all unotuched pos pts
        self.hist_size = hist_size                 # int, no. op history pos 
        self.history = Queue(self.hist_size)       # stores history positions of the robot
        self.robot = None                          # Current position of the robot

    def update(self, data):
        robo = Point(data['robot']['x'], data['robot']['y'])
        ava = [Point(p['x'], p['y']) for p in data['points'] if p['collected'] is False and p['score'] > 0]
        # found = [Point(p['x'], p['y']) for p in data['points'] if p['collected'] and p['score'] == 1]
        if self.history.full():
            self.history.get()
        self.history.put(robo)
        self.robot = robo            # Cur 

        if self.history.full() and ((len(ava) < len(self.available)) or self.is_stationary(2)): # found or stopped
            p = self.find_closest()
            self.history = Queue(self.hist_size)
            self.available = ava
            self.move(p)

    def is_stationary(self, error=0):
        for p in list(self.history):
            if abs(p.x-self.robot.x) > error or abs(p.y-self.robot.y) > error:
                return False
        return True

    def move(self, p):
        move(self.robot.tp_dict(), p.to_dict())

    def find_closest(self):
        mind = 2**31
        minp = None
        for p in self.available:
            d = Point.dist(self.robot, p)
            if d < mind:
                mind = d
                minp = p

        return minp



