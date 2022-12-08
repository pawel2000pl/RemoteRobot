#!/usr/bin/python3 -O

import threading
import cv2
import numpy as np
import json
from os import system
from flask import Flask, Response, request
from serial import Serial
from time import sleep, time
from math import sqrt
from collections import defaultdict
from random import random

ID_LEFT_ENGINE = 1
ID_RIGHT_ENGINE = 2
ID_CAMERA_X = 3
ID_CAMERA_Y = 4
ID_GRAB = 5
ID_BEEP_ON = 6
ID_BEEP_OFF = 7
ID_LED_ON = 8
ID_LED_OFF = 9
ID_MAX_ENGINE_CORRECTION = 10

VALUES = ["left", "right", "speed", "grab", "cameraX", "cameraY", "maxEngineCorrection", "led", "qr", "buzzer"]

def tryValue(v):
    try:
        return int(v)
    except:
        pass
    try:
        return float(v)
    except:
        pass
    return v

def hypot(*args):
    return sqrt(sum([x*x for x in args]))

def areaOfTriangle(p1, p2, p3):
    a = hypot(p1[0] - p2[0], p1[1] - p2[1])
    b = hypot(p1[0] - p3[0], p1[1] - p3[1])
    c = hypot(p3[0] - p2[0], p3[1] - p2[1])
    p = (a + b + c) / 2
    return sqrt(p * (p - a) * (p - b) * (p - c))

def areaOfQuadrangle(p1, p2, p3, p4):
    return areaOfTriangle(p1, p2, p3) + areaOfTriangle(p1, p4, p3)

class RobotServer:

    def __init__(self, port="/dev/ttyS0"):
        self.currentFrame = np.zeros([640,480,3])
        self.lastSentMessage = defaultdict(lambda: None)
        self.stopping = False
        self.frameCount = 0
        self.arduinoData = dict()
        self.updateTime = time()
        self.lastFrameTime = time()
        self.lastQrDetectedTime = time()
        self.driving = {key: 0 for key in VALUES}
        self.serial = Serial(port, 115200)
        self.qrCodeDetector = cv2.QRCodeDetector()
        self.arduinoThread = threading.Thread(target=self.arduinoLoop)
        self.arduinoThread.start()
        self.detectedQRPoints = None
        self.QrThread = threading.Thread(target=self.detectQRLoop)
        self.QrThread.start()
        self.startCameraThread()
        self.app = Flask("robot")
        self.app.route('/')(self.index)
        self.app.route('/video_feed')(self.video_feed)   
        self.app.route('/setValue', methods=['POST'])(self.setValue)

    def __del__(self):
        self.stopping = True
        try:
            if self.cameraThread.is_alive():
                self.cameraThread.join()
            if self.QrThread.is_alive():
                self.QrThread.join()
            if self.arduinoThread.is_alive():
                self.arduinoThread.join()
        except:
            pass

    def startCameraThread(self):
        if (not 'cameraThread' in self.__dict__) or (not self.cameraThread.is_alive()):
            system("v4l2-ctl --set-ctrl auto_exposure=0")
            system("v4l2-ctl --set-ctrl iso_sensitivity_auto=1")
            self.cameraThread = threading.Thread(target=self.cameraLoop)
            self.cameraThread.start()

    def gen_frames(self):  
        while True:      
            img = self.currentFrame if self.detectedQRPoints is None else cv2.polylines(self.currentFrame, [self.detectedQRPoints.astype(int)], True, (0, 255, 0), 3)
            img = img.astype(float)
            img = 0.25*(img[::2, ::2, :] + img[1::2, ::2, :] + img[::2, 1::2, :] + img[1::2, 1::2, :])                    
            _ret, buffer = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 42])
            frame = buffer.tobytes()
            self.lastFrameTime = time()
            yield (b'--frame\r\n' + b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  
            sleep(1/30)

    def qrNotDetected(self):
        if time() - self.lastQrDetectedTime > 5:
            self.driving["cameraX"] = random()*200-100
            self.lastQrDetectedTime = time()

    def qrDetected(self):
        self.lastQrDetectedTime = time()
        self.driving["cameraX"] = str(float(self.driving["cameraX"]) * 0.65)

    def detectQRLoop(self):
        currentFrameNumber = 0
        TARGET = "target2"
        while not self.stopping:
            sleep(0.2)
            if self.driving["qr"]!=1:
                self.detectedQRPoints = None
            elif currentFrameNumber != self.frameCount:
                currentFrameNumber = self.frameCount
                anything, decodedText, points, _ = self.qrCodeDetector.detectAndDecodeMulti(self.currentFrame)
                if not anything:
                    self.detectedQRPoints = None
                    self.qrNotDetected()
                    continue
                results = [r for r in zip(decodedText, points) if r[0] == TARGET]
                if len(results) != 1:
                    self.detectedQRPoints = None
                    self.qrNotDetected()
                    continue
                decodedText, points = results[0]
                self.detectedQRPoints = points
                points = [(float(points[i, 0]), float(points[i, 1])) for i in range(4)]
                self.qrDetected()
                self.qrTargetting(points, self.currentFrame.shape)

    def qrTargetting(self, points, shape):
        DESTINATION_AREA = 0.15
        points = [(p[0]/shape[1], p[1]/shape[0]) for p in points]
        area = areaOfQuadrangle(*points)
        if area >= DESTINATION_AREA:
            self.driving["grab"] = 9
            return
        center = (sum([p[0] for p in points])/len(points), sum([p[1] for p in points])/len(points))
  
        self.driving["cameraY"] = str(int(float(self.driving["cameraY"]))-50*(center[1]-0.5))
        
        targetCenter = 0.5 - float(self.driving["cameraX"])/127
        leftPower = min(1, max(0, 0.5+2*(center[0] - targetCenter)))
        rightPower = min(1, max(0, 0.5+2*(targetCenter - center[0])))
        m = leftPower + rightPower
        if m != 0:
            leftPower = min(1, leftPower*2/m)
            rightPower = min(1, rightPower*2/m)
        self.driving["left"] = "%2f"%leftPower
        self.driving["right"] = "%2f"%rightPower
        print(self.driving["left"], self.driving["right"])
        sleep(0.15)
        self.driving["left"] = "0"
        self.driving["right"] = "0"
        sleep(0.5)


    def cameraLoop(self):    
        self.lastFrameTime = time()
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FPS, 30)
        try:
            while (not self.stopping) and (time() - self.lastFrameTime < 3.0):
                ret, newframe = camera.read()
                if ret:  
                    a, b = np.quantile(newframe, [0.05, 0.95])
                    newframe = (255/(b-a+1)) * (newframe-a)
                    newframe[newframe>255] = 255
                    newframe[newframe<0] = 0
                    self.currentFrame = newframe.astype(np.uint8)
                    self.frameCount += 1
        finally:        
            camera.release()

    def sendMessage(self, id, value=0):
        try:
            if self.lastSentMessage["LastTime"] is None:
                self.lastSentMessage["LastTime"] = time()
            if value == "":
                value = 0
            id = int(id)
            value = int(float(value))
            if value < 0:
                value += 256
            if self.lastSentMessage[id] == value and time()-self.lastSentMessage["LastTime"] < 0.5:
                return False
            self.lastSentMessage["LastTime"] = time()
            self.lastSentMessage[id] = value
            buf = id.to_bytes(1, 'little') + value.to_bytes(1, 'little')
            self.serial.write(buf)
            sleep(0.001)
            return True
        except Exception as err:
            print("error", err, "with:", id, value)
            return False

    def arduinoLoop(self):
        while not self.stopping:
            if time()-self.updateTime > 1.0:
                self.driving["left"] = 0
                self.driving["right"] = 0
                self.driving["led"] = 0

            if time()-self.updateTime > 10.0:
                sleep(1)
                continue
            
            if self.driving["led"]:
                self.sendMessage(ID_LED_ON)
            else:
                self.sendMessage(ID_LED_OFF)

            if self.driving["buzzer"]:
                self.sendMessage(ID_BEEP_ON)
            else:
                self.sendMessage(ID_BEEP_OFF)

            self.sendMessage(ID_LEFT_ENGINE, float(self.driving["speed"])*float(self.driving["left"]))
            self.sendMessage(ID_RIGHT_ENGINE, float(self.driving["speed"])*float(self.driving["right"]))
            self.sendMessage(ID_CAMERA_X, self.driving["cameraX"])
            self.sendMessage(ID_CAMERA_Y, self.driving["cameraY"])
            self.sendMessage(ID_GRAB, self.driving["grab"])
            self.sendMessage(ID_MAX_ENGINE_CORRECTION, self.driving["maxEngineCorrection"])

            try:
                while self.serial.in_waiting > 64:
                    line = self.serial.read_until(b'\n').decode(encoding="utf-8").strip()
                    if len(line)>2:
                        self.arduinoData = json.loads(line)
            except Exception as err:
                print("Error during recieving data from arduino:", err)
                print(line)
            
            sleep(0.015)

    def index(self):
        with open('index.html') as f:
            return f.read()

    def video_feed(self):
        self.startCameraThread()
        return Response(self.gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def setValue(self):
        dataToSend = self.arduinoData.copy()
        try:
            data = request.json
            if data.get("qr", False):
                data.pop("cameraX", None)
                data.pop("cameraY", None)
                data.pop("left", None)
                data.pop("right", None)
                data.pop("grab", None)
            for name in set(self.driving.keys()).intersection(set(data.keys())):
                self.driving[name] = data[name]
            self.updateTime = time()
            dataToSend["status"] = "ok"
            return json.dumps(dataToSend)
        except Exception as err:
            print(err)
            dataToSend["status"] = "error"
            return json.dumps(dataToSend)

    def run(self):
        self.app.run("0.0.0.0")
        s.stopping = True


if __name__ == "__main__":
    s = RobotServer()
    s.run()
    print("Done")

    
    

