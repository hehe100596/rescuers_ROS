//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       ErrorMessage.qml
 * Date:           17. 11. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for error messages
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import QtQuick.Window 2.0
import "../javascript/theme.js" as Theme

Rectangle
{
    id: messageBox

    visible: true

    width: window.width
    height: window.height

    color: "transparent"

    property string message

    MouseArea
    {
        anchors.fill: parent
        onClicked: messageBox.destroy ()
    }

    Rectangle
    {
        width: window.width / 1.5
        height: window.height / 15

        color: Theme.combo_box

        border.color: Theme.button
        border.width: (window.height + window.width) / 1000
        radius: (window.height + window.width) / 500

        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter

        Text
        {
            anchors.fill: parent

            font.pointSize: (window.height + window.width) / 140
            font.bold: true
            font.family: "Arial"
            color: Theme.button

            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter

            text: messageBox.message + " Click anywhere to close this."
        }
    }
}

//--------------------------------------------------------------------------------
// End of file ErrorMessage.qml
//--------------------------------------------------------------------------------
