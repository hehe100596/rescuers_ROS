//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       HealthBar.qml
 * Date:           25. 02. 2018
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for health bar
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import "../javascript/theme.js" as Theme

Rectangle
{
    id: health_bar

    property int range
    property int lost
    property string label
    property bool fromleft

    border.color: Theme.button
    border.width: (game.height + game.width) / 1000

    range: 100
    lost: 0
    label: "None"
    fromleft: false

    width: game.width / 8
    height: game.height / 30
    color: fromleft ? Theme.saved_health : Theme.lost_health

    Rectangle
    {
        anchors.left: fromleft ? undefined : health_bar.left
        anchors.right: fromleft ? health_bar.right : undefined

        border.color: Theme.button
        border.width: (game.height + game.width) / 1000

        height: health_bar.height
        width: health_bar.width - Math.floor (lost / range * health_bar.width)
        color: fromleft ? Theme.lost_health : Theme.health_bar
    }

    Text
    {
        anchors.bottom: parent.top

        font.pointSize: (game.height + game.width) / 150
        color: Theme.button

        text: label
    }
}

//--------------------------------------------------------------------------------
// End of file HealthBar.qml
//--------------------------------------------------------------------------------
