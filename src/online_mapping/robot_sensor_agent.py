#!/usr/bin/env python
from __future__ import print_function
# ROS Kinetic Python2.7; sensor-only, no motor publishers or motion services.
import argparse
import json
import struct
import sys
import threading
import time
import SocketServer
import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class Sensor(object):
    def __init__(self, fps):
        self.condition=threading.Condition()
        self.packet=None
        self.seq=0
        self.last=0
        self.fps=fps
        self.color_info=None
        self.depth_info=None
        self.bridge=CvBridge()
        rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.color_info_cb,queue_size=1)
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info',CameraInfo,self.depth_info_cb,queue_size=1)
        color=message_filters.Subscriber('/camera/color/image_raw',Image,queue_size=2)
        depth=message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,queue_size=2)
        self.sync=message_filters.ApproximateTimeSynchronizer([color,depth],5,0.025)
        self.sync.registerCallback(self.pair)

    def color_info_cb(self,msg): self.color_info=msg
    def depth_info_cb(self,msg): self.depth_info=msg

    def pair(self,color,depth):
        now=time.time()
        ci,di=self.color_info,self.depth_info
        if now-self.last<1./self.fps or ci is None or di is None:
            return
        if ci.header.frame_id!=di.header.frame_id or list(ci.K)!=list(di.K) or (ci.width,ci.height)!=(di.width,di.height):
            rospy.logerr_throttle(5,'Aligned depth/color CameraInfo mismatch')
            return
        self.last=now
        try:
            bgr=self.bridge.imgmsg_to_cv2(color,'bgr8')
            dep=self.bridge.imgmsg_to_cv2(depth,'passthrough')
            if depth.encoding not in ('16UC1','mono16') or dep.dtype!=np.uint16:
                raise ValueError('Expected D435 millimeter uint16 depth')
            if color.header.frame_id!=ci.header.frame_id or depth.header.frame_id!=ci.header.frame_id or bgr.shape[:2]!=dep.shape:
                raise ValueError('Image frame/shape mismatch')
            okc,jpg=cv2.imencode('.jpg',bgr,[int(cv2.IMWRITE_JPEG_QUALITY),85])
            okd,png=cv2.imencode('.png',dep,[int(cv2.IMWRITE_PNG_COMPRESSION),1])
            if not (okc and okd): return
            c,d=jpg.tobytes(),png.tobytes()
            seq=self.seq+1
            meta={'seq':seq,'K':list(ci.K),'D':list(ci.D),'width':ci.width,'height':ci.height,
                  'frame_id':ci.header.frame_id,'depth_scale':1000.,'depth_encoding':depth.encoding,
                  'color_stamp':color.header.stamp.to_sec(),'depth_stamp':depth.header.stamp.to_sec(),
                  'pair_delta_ms':abs((color.header.stamp-depth.header.stamp).to_sec())*1000,
                  'color_bytes':len(c),'depth_bytes':len(d),'valid_depth_ratio':float(np.mean(dep>0)),
                  'encode_ms':(time.time()-now)*1000,'robot_encoded_wall':time.time()}
            header=json.dumps(meta).encode('utf-8')
            packet=struct.pack('!I',len(header))+header+c+d
            with self.condition:
                self.seq=seq
                self.packet=packet
                self.condition.notify_all()
        except Exception as exc:
            rospy.logwarn_throttle(5,'Capture: %s',str(exc))


class Handler(SocketServer.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(2.)
        last=-1
        while not rospy.is_shutdown():
            with self.server.sensor.condition:
                if self.server.sensor.seq==last or self.server.sensor.packet is None:
                    self.server.sensor.condition.wait(1.)
                packet=self.server.sensor.packet
                seq=self.server.sensor.seq
            if packet is None or seq==last: continue
            try:
                self.request.sendall(packet)
                # Exactly one frame in flight. After ACK choose the newest frame,
                # rather than filling TCP buffers with obsolete camera frames.
                if self.request.recv(1)!=b'1': break
            except Exception: break
            last=seq


class Server(SocketServer.ThreadingTCPServer):
    allow_reuse_address=True
    daemon_threads=True


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=18865)
    parser.add_argument('--fps',type=float,default=10.)
    args=parser.parse_args()
    if not 1<=args.fps<=15: raise ValueError('fps must be 1..15')
    rospy.init_node('roban_stage1_sensor',anonymous=True)
    server=Server(('127.0.0.1',args.port),Handler)
    server.sensor=Sensor(args.fps)
    print('READY:'+str(args.port))
    sys.stdout.flush()
    server.serve_forever()
