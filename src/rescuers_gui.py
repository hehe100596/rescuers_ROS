#!/usr/bin/env python

# Edited by xbacov04

import os
import sys
import signal
import rospy
from PyQt5 import QtGui, QtWidgets, QtCore, QtNetwork, QtQml, QtTest
from xbacov04.helpers import ProjectorCalibrator, TouchCalibrator
from art_msgs.msg import Touch
from OpenGL import GL

scriptDir = os.path.dirname(os.path.realpath(__file__))


class RescuersGui(QtQml.QQmlApplicationEngine):

    def __init__(self, scene_server_port):

        super(RescuersGui, self).__init__()

        self.port = scene_server_port
        self.rpm = rospy.get_param("/art/interface/projected_gui/rpm")

        self.tcpServer = QtNetwork.QTcpServer(self)
        if not self.tcpServer.listen(port=self.port):
            rospy.logerr(
                'Failed to start scene TCP server on port ' + str(self.port))

        self.tcpServer.newConnection.connect(self.new_connection)
        self.connections = []

        self.scene_timer = QtCore.QTimer()
        self.scene_timer.timeout.connect(self.send_to_clients_evt)
        self.scene_timer.start(1.0 / 15 * 1000)

        self.projectors = [ProjectorCalibrator("localhost")]

        rospy.loginfo("Waiting for projector nodes...")
        for proj in self.projectors:
            proj.wait_until_available()
            if not proj.is_calibrated():
                rospy.loginfo("Starting calibration of projector: " + proj.proj_id)
                proj.calib_pub.publish(False)
                proj.calibrate(self.calibrated_cb)
            else:
                rospy.loginfo("Projector " + proj.proj_id + " already calibrated.")
                proj.calib_pub.publish(True)

    def calibrated_cb(self, proj):

        proj.calib_pub.publish(True)
        rospy.loginfo("Projector " + proj.proj_id + " calibrated: " + str(proj.is_calibrated()))

    def handle_touch(self, touching):

        os.system("xinput disable 8") # SHOULD PROBABLY BE COMMENTED OUT

        if touching.touch:
            point = QtCore.QPoint(touching.point.point.x*self.rpm, self.rootObjects()[0].height() - touching.point.point.y*self.rpm)
            event = QtGui.QMouseEvent(QtCore.QEvent.MouseButtonPress, point, QtCore.Qt.LeftButton, QtCore.Qt.NoButton, QtCore.Qt.NoModifier)
            QtWidgets.QApplication.postEvent(self.rootObjects()[0], event)
        else:
            point = QtCore.QPoint(touching.point.point.x*self.rpm, self.rootObjects()[0].height() - touching.point.point.y*self.rpm)
            event = QtGui.QMouseEvent(QtCore.QEvent.MouseButtonRelease, point, QtCore.Qt.LeftButton, QtCore.Qt.NoButton, QtCore.Qt.NoModifier)
            QtWidgets.QApplication.postEvent(self.rootObjects()[0], event)

    def continue_calibration(self, scene):

        self.scene = scene
        self.touches = [TouchCalibrator(self.scene, self.rpm)]

        rospy.loginfo("Waiting for touch nodes...")
        for touch in self.touches:
            touch.wait_until_available()
            rospy.loginfo("Starting calibration of touch.")
            touch.calibrate()
            rospy.Subscriber(touch.touch_ns + "touch", Touch, self.handle_touch)

    def new_connection(self):

        rospy.loginfo("Some projector node just connected.")
        self.connections.append(self.tcpServer.nextPendingConnection())
        self.connections[-1].setSocketOption(QtNetwork.QAbstractSocket.LowDelayOption, 1)
        self.continue_calibration(self.rootObjects()[0])

        # TODO deal with disconnected clients!
        # self.connections[-1].disconnected.connect(clientConnection.deleteLater)

    def send_to_clients_evt(self):

        if len(self.connections) == 0:
            return

        block = QtCore.QByteArray()
        out = QtCore.QDataStream(block, QtCore.QIODevice.WriteOnly)
        out.setVersion(QtCore.QDataStream.Qt_4_0)
        out.writeUInt32(0)

        pix = self.rootObjects()[0].screen().grabWindow(self.rootObjects()[0].winId()).toImage()
        pix = pix.mirrored()
        img = QtCore.QByteArray()
        buffer = QtCore.QBuffer(img)
        buffer.open(QtCore.QIODevice.WriteOnly)
        pix.save(buffer, "JPG", 95)
        out << img

        #screen = QtWidgets.QApplication.primaryScreen().grabWindow(0)
        #crop = QtCore.QRect(self.rootObjects()[0].x(), self.rootObjects()[0].y(), self.rootObjects()[0].width(), self.rootObjects()[0].height())
        #pixmap = screen.copy()
        #pixmap.fill(QtCore.Qt.black)
        #painter = QtGui.QPainter(pixmap)
        #painter.drawImage(crop, screen.copy(crop).toImage())
        #painter.end()
        #pix = pixmap.toImage()
        #pix = pix.mirrored()
        #img = QtCore.QByteArray()
        #buffer = QtCore.QBuffer(img)
        #buffer.open(QtCore.QIODevice.WriteOnly)
        #pix.save(buffer, "JPG", 95)
        #out << img

        out.device().seek(0)
        out.writeUInt32(block.size() - 4)

        for con in self.connections:
            con.write(block)

    def debug_show(self):
      
        self.load(QtCore.QUrl(scriptDir + os.path.sep + 'qml/TitleScreen.qml'))

def sigint_handler(*args):

    sys.stderr.write('\r')
    QtWidgets.QApplication.quit()


def main(args):

    rospy.init_node('rescuers_gui', anonymous=True, log_level=rospy.DEBUG)

    signal.signal(signal.SIGINT, sigint_handler)

    app = QtWidgets.QApplication(sys.argv)
    app.setWindowIcon(QtGui.QIcon(scriptDir + os.path.sep + 'img/icon.ico'))

    engine = RescuersGui(1234)
    engine.debug_show()
    engine.quit.connect(app.quit)

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())


if __name__ == '__main__':

    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
