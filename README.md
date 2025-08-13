Projekt Beschreibung
=========================================================================
Integration Roboter mit Leserscanner in Gazebo
     -Set Up des Systems
     -Entwicklung eines Beispielhaften, Bildbasierten Navigationsverfahren

Programm Installation
========================
In Console 1, folgende Befehle im selben Verzeichnis Ausführen.
1. $ "colcon build"
2. $ source install/setup.bash
Projekt abhängigkeiten werden nun Compliert..,

Programm Ausführen
=====================
In Console 1, folgendes Befehl eingeben
$ ros2 launch depthcam_sim simulation.launch.py
Hiemit werden Gazebo-Sim, Rviz und Rqt-Imgae-viewer geladen 

In Console 2, folgendes Befehl eingeben
$ build/depthcam_sim/depthcam_node
Hiermit wird der Navigations-Programm gestartet.
