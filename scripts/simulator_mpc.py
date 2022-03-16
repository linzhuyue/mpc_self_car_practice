import argparse
import base64
from datetime import datetime
import os
import numpy as np
import socketio
import eventlet
import time
import eventlet.wsgi
from flask import Flask
# pip install python-engineio==3.8.2
# pip install python-socketio==4.2.1
sio = socketio.Server()
app = Flask(__name__)
model = None
class SimplePIController:
    def __init__(self, Kp, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.set_point = 0.
        self.error = 0.
        self.integral = 0.

    def set_desired(self, desired):
        self.set_point = desired

    def update(self, measurement):
        # proportional error
        self.error = self.set_point - measurement

        # integral error
        self.integral += self.error

        return self.Kp * self.error + self.Ki * self.integral
controller = SimplePIController(0.1, 0.002)
set_speed = 10
controller.set_desired(set_speed)

@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        try:
            # TODO list 1*/
            # data parsing code should be putted here!*/
            ptsx = [float(d) for d in data["ptsx"]]
            ptsy = [float(d) for d in data["ptsy"]]
            px = float(data["x"])
            py = float(data["x"])
            psi = float(data["psi"])
            v = float(data["speed"])
            delta = float(data["steering_angle"])
            acceleration = float(data["throttle"])
            print(data)
            # TODO list 2*/

            # Algorithm that generating feedback control commands code should be putted here!*/
            #this not worked with python, i do not debug it ok.
            # it can not send the data to the simulation, it can just receive the data from the simluation platfrom with uvwebsocket
            steering_angle=10
            throttle = 10*controller.update(v)

            print(steering_angle, throttle)
            time.sleep(0.01)
            send_control(steering_angle, throttle)
        except Exception as e:
            print(e)
    else:
    # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control(0, 0)


def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': steering_angle.__str__(),
            'throttle': throttle.__str__()
        },
        skip_sid=True)


if __name__ == '__main__':

    # model = load_model(args.model)
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)
    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)

