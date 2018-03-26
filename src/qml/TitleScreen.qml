//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       TitleScreen.qml
 * Date:           21. 10. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for title screen
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import QtQuick.Window 2.0

Window
{
    id: window
    visible: true

    property string difficulty
    property string players
    property string building
    property string timer

    property bool calibrating
    property int calibPointSize
    property int calibPointX
    property int calibPointY

    difficulty: "Medium"
    players: "1"
    building: "1st Edition"
    timer: "OFF"

    calibrating: false
    calibPointSize: 0
    calibPointX: 0
    calibPointY: 0

    width: 1280
    height: 720

    minimumWidth: 640
    minimumHeight: 360

    title: "RESCUERS"

    function drawPoint (pointSize, pointX, pointY)
    {
        window.calibrating = true
        window.calibPointSize = pointSize
        window.calibPointX = pointX
        window.calibPointY = pointY
    }

    function stopCalibration ()
    {
        window.calibrating = false
    }

    function executeMenuButton (operation)
    {
        if (operation === "New Game") setupGameWindow ()

        if (operation === "Load Game") showErrorMessage ("Saving and loading is not implemented in this version.")

        if (operation === "Options") showSettings ()

        if (operation === "Instructions") Qt.openUrlExternally ("http://www.indieboardsandcards.com/fpfr.php")

        if (operation === "Exit Game") Qt.quit ()
    }

    function showErrorMessage (errorMessage)
    {
        var component = Qt.createComponent ("ErrorMessage.qml")
        var messagebox = component.createObject (window)
        messagebox.message = errorMessage
    }

    function showSettings ()
    {
        var component = Qt.createComponent ("SettingsScreen.qml")
        var settings = component.createObject (window)
    }

    function setupGameWindow ()
    {
        var component = Qt.createComponent ("GameScreen.qml")
        var settings = component.createObject (window)
    }

    Component.onCompleted:
    {
        setX (Screen.width / 2 - width / 2)
        setY (Screen.height / 2 - height / 2)
        window.showMaximized ()
    }

    Image
    {
        width: parent.width
        height: parent.height
        source: "../img/background.jpg"
    }

    Column
    {
        anchors.right: parent.right
        anchors.rightMargin: window.width / 15

        anchors.top: parent.top
        anchors.topMargin: window.height / 4

        spacing: window.height / 25

        MenuButton
        {
            operation: "New Game"
        }

        MenuButton
        {
            operation: "Load Game"
        }

        MenuButton
        {
            operation: "Options"
        }

        MenuButton
        {
            operation: "Instructions"
        }

        MenuButton
        {
            operation: "Exit Game"
        }
    }

    Rectangle
    {
        visible: window.calibrating
        color: "black"

        width: parent.width
        height: parent.height

        MouseArea
        {
            enabled: window.calibrating
            anchors.fill: parent
        }

        Rectangle
        {
            visible: window.calibrating
            color: "white"

            width: window.calibPointSize
            height: window.calibPointSize

            radius: width * 0.5

            x: window.calibPointX
            y: window.calibPointY
        }
    }
}

//--------------------------------------------------------------------------------
// End of file TitleScreen.qml
//--------------------------------------------------------------------------------
