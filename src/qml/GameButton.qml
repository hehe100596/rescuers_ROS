//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       GameButton.qml
 * Date:           19. 11. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for game button
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import "../javascript/theme.js" as Theme

Rectangle
{
    id: game_button

    property bool pressed
    property string operation
    property string button_text

    border.color: Theme.border
    border.width: (game.height + game.width) / 600
    radius: (game.height + game.width) / 150

    pressed: false
    operation: "Inactive"
    button_text: operation

    width: game.width / 6
    height: game.height / 10
    color: pressed ? Theme.clicked_button : Theme.button

    Text
    {
        anchors.fill: parent

        font.bold: true
        font.pointSize: (game.height + game.width) / 140
        color: Theme.text

        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter

        text: button_text
    }

    MouseArea
    {
        anchors.fill: parent

        onPressed: game_button.pressed = true
        onReleased: game_button.pressed = false
        onClicked: executeGameButton (operation)
    }
}

//--------------------------------------------------------------------------------
// End of file GameButton.qml
//--------------------------------------------------------------------------------
