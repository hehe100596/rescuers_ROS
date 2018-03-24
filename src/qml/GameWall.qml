//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       GameSquare.qml
 * Date:           02. 12. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for gameboard wall
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import "../javascript/theme.js" as Theme

Rectangle
{
    id: gamewall

    property string state
    property bool isLeft
    property bool isClickable

    state: "none"
    color: state != "none" ? Theme.button : "transparent"

    width: 0
    height: 0

    enabled: false
    isLeft: false
    isClickable: gamewall.state != "full" && gamewall.state != "none" && gamewall.state != "destroyed" ? true : false

    function actualizeWall (actual_state)
    {
        if (actual_state === "damaged") return "../img/gamewall_damaged.jpg"

        else if (actual_state === "destroyed") return "../img/gamewall_destroyed.jpg"

        else if (actual_state === "opened") return "../img/gamewall_opened.jpg"

        else if (actual_state === "closed") return "../img/gamewall_closed.jpg"

        else return ""
    }

    Image
    {
        width: gamesquare.width / 2.5
        height: gamesquare.height / 2.5

        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter

        source: actualizeWall (gamewall.state)

        Rectangle
        {
            anchors.fill: parent

            color: enabled && isClickable ? Theme.doors : "transparent"
            opacity: 0.5
        }

        MouseArea
        {
            anchors.fill: parent

            enabled: isClickable ? true : false

            onClicked: game.moveDoors (gamesquare.column, gamesquare.row, gamewall.isLeft)
        }
    }
}

//--------------------------------------------------------------------------------
// End of file GameWall.qml
//--------------------------------------------------------------------------------
