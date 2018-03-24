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

Window
{
    id: messageBox

    visible: true
    modality: Qt.ApplicationModal
    flags: Qt.Dialog

    property string message

    width: 400
    height: 50

    maximumHeight: height
    maximumWidth: width

    minimumHeight: height
    minimumWidth: width

    color: Theme.button

    title: "MESSAGE"

    Text
    {
        anchors.fill: parent

        font.pointSize: 10
        font.bold: true
        font.family: "Arial"
        color: Theme.text

        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter

        text: message
    }

    MouseArea
    {
        anchors.fill: parent
        onClicked: messageBox.close ()
    }
}

//--------------------------------------------------------------------------------
// End of file ErrorMessage.qml
//--------------------------------------------------------------------------------
