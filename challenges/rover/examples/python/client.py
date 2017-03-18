import paho.mqtt.client as mqtt
import json
import math

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


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    # print(msg.topic)
    obj = json.loads(msg.payload.decode("utf-8"))
    # print(obj)

    if 'incoming' in msg.topic:
        print(obj)
        client.publish('players/' + PLAYER_NAME, json.dumps({"command": "start"}))
    elif 'game' in msg.topic:
        print(obj)

    # TODO: implement algorithm

def move_forward(dist):
    client.publish('robot/process', json.dumps({"command": "forward", "args": dist}))

def move_backward(dist):
    client.publish('robot/process', json.dumps({"command": "backward", "args": dist}))

def turn(angle):
    if angle >= 0:
        client.publish('robot/process', json.dumps({"command": "right", "args": angle}))
    else:
        client.publish('robot/process', json.dumps({"command": "left", "args": angle}))

def move(c, g):
    d_trans = math.sqrt((g['x']-c['x'])**2 + (g['y']-c['y'])**2)
    d_rot = math.tan2(g['y'] - c['y'], g['x'] - c['x']) - c['theta']
    turn(d_rot)
    move_forward(d_trans)
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
