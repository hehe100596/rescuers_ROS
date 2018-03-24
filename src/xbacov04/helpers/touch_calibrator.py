#!/usr/bin/env python

# Edited by xbacov04

import rospy
from PyQt5 import QtCore
from art_msgs.srv import TouchCalibrationPoints, TouchCalibrationPointsResponse
from std_msgs.msg import Bool, Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv, EmptyRequest


class TouchCalibrator(QtCore.QObject):

    touch_calibration_points_signal = QtCore.pyqtSignal()

    def __init__(self):

        super(TouchCalibrator, self).__init__()

        self.touch_ns = "/art/interface/touchtable/"
        self.touch_calib_ns = "/art/interface/projected_gui/touch_calibration"

        self.touch_calib_srv = rospy.Service(self.touch_calib_ns, TouchCalibrationPoints, self.touch_calibration_points_cb)
        self.touched_sub = None
        self.calibrating_touch = False
        self.touch_calibration_points = None
        self.point_item = None

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

        #for it in self.scene.items():
        #    it.setVisible(False)

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
            p = self.touch_calibration_points.pop(0)
            # Add drawing of that point to scene
            #self.point_item = QtGui.QGraphicsEllipseItem(0, 0, 0.01*self.scene.rpm, 0.01*self.scene.rpm, None, self.scene)
            #self.point_item.setBrush(QtGui.QBrush(QtCore.Qt.white, style = QtCore.Qt.SolidPattern))
            #self.point_item.setPos(p[0]*self.scene.rpm, (self.height - p[1])*self.scene.rpm)
            rospy.loginfo("Wait until white point appears and press it to calibrate touch.")
        except IndexError:
            #for it in self.scene.items():
                # TODO fix this - in makes visible even items that are invisible by purpose
            #    it.setVisible(True)
            #    self.touched_sub.unregister()
            self.touched_sub.unregister()
            rospy.loginfo("There are no white points to press.")

    def touch_detected_cb(self, data):

        try:
            p = self.touch_calibration_points.pop(0)
            # Delete drawing of that point from scene and draw another one
            rospy.loginfo("Press another white point to calibrate touch.")
        except IndexError:
            #self.scene.removeItem(self.point_item)
            #del self.point_item
            #for it in self.scene.items():
                # TODO fix this - in makes visible even items that are invisible by purpose
                #it.setVisible(True)
                #self.touched_sub.unregister()
            self.touched_sub.unregister()
            rospy.loginfo("Touch was successfully calibrated.")
