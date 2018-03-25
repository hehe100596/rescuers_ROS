#!/usr/bin/env python

# Edited by xbacov04

import rospy
from PyQt5 import QtCore, QtGui
from art_msgs.srv import TouchCalibrationPoints, TouchCalibrationPointsResponse
from std_msgs.msg import Bool, Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv, EmptyRequest


class TouchCalibrator(QtCore.QObject):

    touch_calibration_points_signal = QtCore.pyqtSignal()

    def __init__(self, scene, rpm):

        super(TouchCalibrator, self).__init__()

        self.scene = scene
        self.rpm = rpm
        self.touch_ns = "/art/interface/touchtable/"
        self.touch_calib_ns = "/art/interface/projected_gui/touch_calibration"

        self.touch_calib_srv = rospy.Service(self.touch_calib_ns, TouchCalibrationPoints, self.touch_calibration_points_cb)
        self.touched_sub = None
        self.calibrating_touch = False
        self.touch_calibration_points = None

        self.touch_calibration_points_signal.connect(self.touch_calibration_points_evt)

        self.touch_calibrate = rospy.ServiceProxy(self.touch_ns + "calibrate", EmptySrv)
        self.touch_calibrated = rospy.wait_for_message(self.touch_ns + "calibrated", Bool).data

    def wait_until_available(self):

        self.touch_calibrate.wait_for_service()

    def calibrate(self):

        if not self.touch_calibrated:
            req = EmptyRequest()
            self.touch_calibrate.call(req)
        else:
            rospy.loginfo("Touch already calibrated.")

    def touch_calibration_points_cb(self, req):

        self.touched_sub = rospy.Subscriber(self.touch_ns + "touch_detected", EmptyMsg, self.touch_detected_cb, queue_size=10)
        self.touch_calibration_points = []

        for pt in req.points:
            self.touch_calibration_points.append((pt.point.x, pt.point.y))

        self.touch_calibration_points_signal.emit()

        resp = TouchCalibrationPointsResponse()
        resp.success = True
        return resp

    def touch_calibration_points_evt(self):

        self.touch_calibrating = True

        try:
            rospy.loginfo("Switch to application window, wait until white point appears on the table and press it to calibrate touch.")
            p = self.touch_calibration_points.pop(0)
            self.scene.drawPoint(0.01*self.rpm, p[0]*self.rpm, self.scene.height() - p[1]*self.rpm)
        except IndexError:
            self.scene.stopCalibration()
            self.touched_sub.unregister()
            rospy.loginfo("Touch calibration has been completed.")

    def touch_detected_cb(self, data):

        self.touch_calibration_points_signal.emit()
