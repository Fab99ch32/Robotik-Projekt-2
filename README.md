# Projekt: Autonome Navigation von mobilen Robotern durch Rohrabschnitte

## Aufgabenbeschreibung

Das Ziel dieses Projekts ist es, mobile Roboter autonom durch Rohrabschnitte fahren zu lassen, die am Ende jeweils eine Wendeplattform haben.
Die Roboter sollen in der Lage sein, auf diesen Plattformen zu wenden und zurückzufahren. Wenn ein vorausfahrender Roboter erkannt wird, soll der eigene Roboter diesem folgen. 
Wenn ein entgegenkommender Roboter erkannt wird, muss der eigene Roboter diesem ausweichen.

## Nodes

- `camera_node`: Dieser Node steuert die Kamera des Roboters.
- `marker_detecte_node`: Dieser Node erkennt Markierungen oder Marker im Kamerabild.
- `logic_node`: Dieser Node enthält die Logik und die Entscheidungsfindung für den Roboter.
- `move_node`: Dieser Node steuert die Bewegungen des Roboters.
- `test_node`: Dieser Node dient zum Testen und Debuggen des Gesamtsystems.
- `Regelung_node`: Dieser Node dient zur Regelung der Roboter, damit der Roboter in der Mitte des Rohr fährt.
- `line_detection` : Dieser Node dient zur Erkennung der Linie in der Mitte des Rohr

## Ausführen der Nodes

Um jede Node auszuführen, navigieren Sie zunächst zum Verzeichnis `robotik_projekt_2/` und aktivieren Sie die ROS 2-Umgebung:

```bash
cd robotik_projekt_2/
source install/local_setup.bash

Dann können Sie einzelne Node wie folgt ausführen:

- "ros2 run camera_node camera_node"

- "ros2 run marker_detecte_node marker_detecte_node"

- "ros2 run logic_node logic_node"

- "ros2 run move_node move_node"

- "ros2 run test_node test_node"

- "ros2 run Regelung_node Regelung_node"

- "ros2 run line_detection line_detection "


