//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       SettingsComboBox.qml
 * Date:           18. 11. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for combo box
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import "../javascript/theme.js" as Theme

Rectangle
{
    id: comboBox

    property variant items
    property alias selectedItem: chosenText.text
    property alias selectedIndex: listView.currentIndex
    property string title

    signal comboBoxClicked

    width: settings.width / 3
    height: settings.height / 12

    items: [""]
    title: "Undefined:"

    Text
    {
        text: title

        anchors.bottom: parent.top
        anchors.bottomMargin: parent.width / 30

        anchors.horizontalCenter: parent.horizontalCenter

        font.family: "Arial"
        font.bold: true
        font.pointSize: (window.height + window.width) / 120

        color: Theme.button
    }

    Rectangle
    {
        id: chosen

        width: parent.width
        height: comboBox.height

        radius: 4
        color: Theme.combo_box

        Text
        {
            id: chosenText
            text: comboBox.items [0]

            anchors.left: parent.left
            anchors.leftMargin: parent.width / 20

            anchors.verticalCenter: parent.verticalCenter

            font.family: "Arial"
            font.bold: true
            font.pointSize: (window.height + window.width) / 140

            color: Theme.button
        }

        MouseArea
        {
            anchors.fill: parent

            onClicked:
            {
                comboBox.state = comboBox.state === "dropDown" ? "droppedDown" : "dropDown"
            }
        }
    }

    Rectangle
    {
        id: dropDown

        width: comboBox.width
        height: 0

        clip: true
        radius: 4

        anchors.top: chosen.bottom
        anchors.margins: 2

        ListView
        {
            id: listView

            height: settings.height

            model: comboBox.items
            currentIndex: 0

            delegate: Item
            {
                width: comboBox.width
                height: comboBox.height

                Rectangle
                {
                    id: selection

                    width: parent.width
                    height: parent.height

                    color: selectionArea.containsMouse ? Theme.combo_box : Theme.combo_box_dropdown
                }

                Text
                {
                    text: modelData

                    anchors.left: parent.left
                    anchors.leftMargin: parent.width / 20

                    anchors.verticalCenter: parent.verticalCenter

                    font.family: "Arial"
                    font.bold: true
                    font.pointSize: (window.height + window.width) / 200

                    color: Theme.button
                }

                MouseArea
                {
                    id: selectionArea

                    hoverEnabled: true
                    anchors.fill: parent

                    onClicked:
                    {
                        comboBox.state = "droppedDown"

                        var prevSelection = chosenText.text
                        chosenText.text = modelData

                        if (chosenText.text != prevSelection) comboBox.comboBoxClicked ()

                        listView.currentIndex = index
                    }
                }
            }
        }
    }

    states: State
    {
        name: "dropDown"

        PropertyChanges
        {
            target: dropDown
            height: comboBox.height * comboBox.items.length
        }
    }

    transitions: Transition
    {
        NumberAnimation
        {
            target: dropDown
            properties: "height"
            easing.type: Easing.OutExpo
            duration: 500
        }
    }
}

//--------------------------------------------------------------------------------
// End of file SettingsComboBox.qml
//--------------------------------------------------------------------------------
