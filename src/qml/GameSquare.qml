//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       GameSquare.qml
 * Date:           26. 11. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for gameboard square
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import "../javascript/theme.js" as Theme

Rectangle
{
    id: gamesquare

    property string state
    property bool alert
    property bool player
    property bool actualPlayer
    property bool smokedAlert
    property bool unpassable
    property bool extinguishableFire
    property bool leftDoorsEnabled
    property bool topDoorsEnabled
    property bool isImageClickable

    property int column
    property int row

    property string leftWall
    property string topWall

    width: gameboard.width / 10
    height: gameboard.height / 8

    x: column * width
    y: row * height

    column: 0
    row: 0

    color: "transparent"
    enabled: false

    leftWall: "none"
    topWall: "none"

    state: "nothing"
    alert: false
    player: false
    actualPlayer: false
    smokedAlert: false
    unpassable: false
    extinguishableFire: false
    leftDoorsEnabled: false
    topDoorsEnabled: false
    isImageClickable: (actualPlayer && state === "realAlert") || (state != "realAlert" && state != "nothing") ? true : false

    function actualizeState (actual_state)
    {
        if (actual_state === "smoke") return "../img/gamesquare_smoke.jpg"

        else if (actual_state === "fire") return "../img/gamesquare_fire.jpg"

        else if (actual_state === "questionMark") return "../img/gamesquare_questionmark.jpg"

        else if (actual_state === "realAlert") return "../img/gamesquare_realalert.jpg"

        else return ""
    }

    MouseArea
    {
        anchors.fill: parent

        onClicked: unpassable ? game.playerAction (column, row) : game.movePlayer (column, row)
    }

    Image
    {
        width: parent.width / 2
        height: parent.height / 2        

        z: gamesquare.z + 1

        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter

        source: actualizeState (gamesquare.state)

        Rectangle
        {
            anchors.fill: parent

            color: gamesquare.enabled && (! unpassable || extinguishableFire) && isImageClickable ? Theme.clickable_state : "transparent"
            opacity: 0.6

            Image
            {
                anchors.fill: parent

                source: smokedAlert ? "../img/gamesquare_questionmark.jpg" : ""
                opacity: 0.6
            }

            MouseArea
            {
                anchors.fill: parent

                enabled: isImageClickable ? true : false

                onClicked: game.playerAction (column, row)
            }
        }
    }

    Rectangle
    {
        anchors.fill: parent

        color: actualPlayer ? Theme.actual_player : gamesquare.enabled ? unpassable ? Theme.unpassable : Theme.clickable : "transparent"
        opacity: 0.5
    }

    GameWall
    {
        width: 3
        height: gamesquare.height

        state: leftWall
        enabled: leftDoorsEnabled
        isLeft: true
    }

    GameWall
    {
        width: gamesquare.width
        height: 3

        state: topWall
        enabled: topDoorsEnabled
        isLeft: false
    }
}

//--------------------------------------------------------------------------------
// End of file GameSquare.qml
//--------------------------------------------------------------------------------
