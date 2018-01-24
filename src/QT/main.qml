import QtQuick 2.0
import QtQuick.Window 2.0
import QtLocation 5.6
import QtPositioning 5.6

Window {
    width: 512
    height: 512
    visible: true

    Map {
        anchors.fill: parent
        center: QtPositioning.coordinate(42.293124, -71.263522) // Olin
        zoomLevel: 16

        plugin: Plugin {
            name: "osm" // "mapboxgl", "esri", ...
        }

        Tractor {
            id: ku
            coordinate: kubo.position

            Component.onCompleted: {
                kubo.position = QtPositioning.coordinate(42.293124, -71.263522);
            }
        }
    }
}
