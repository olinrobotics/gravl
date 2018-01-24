import QtQuick 2.4
import QtLocation 5.6

MapQuickItem {
    id: tractor
    property int heading: 0;

    anchorPoint.x: image.width/2
    anchorPoint.y: image.height/2

    sourceItem: Grid {
        horizontalItemAlignment: Grid.AlignHCenter
        Image {
            id: image
            rotation: heading
            source: "tractor.png"
        }
    }
}