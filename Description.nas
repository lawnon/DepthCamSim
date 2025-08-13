/**************************************************************
Datei: Projekt Notizen
Datum: Sat Jan 18 10:32:09 2025
Autor: Chukwunonso Bob-Anyeji
Beschriebung: Aufgaben-Listen und Status und Beschreibung
**************************************************************/

Projekt Beschreibung
=========================================================================
Aufgabe: Integration Roboter mit Leserscanner in Gazebo
     -Set Up des Systems
     -Entwicklung eines Beispielhaften, Bildbasierten Navigationsverfahren

Beschreibung: Die Kernaufgabe besteht darin einer Mobilerroboter mit einem
Lidar (Light Detection and Ranging) Sensor auszustatten. Mittels diesen Sensor
Infomation/Daten über den Umgebung zuerfassen und Anschließend einen
Navigationsverfahren zu entwicklen. Damit die Umgebung von Mobilen Roboter Kollisionsfrei
navigiert werden kann. Wenn möglich soll der erfassten bereich auch Kartiert werden.

Vorgehensweise: Die umsetzung der Projekts wird aus einem zusammenspiel von Gazebo 9 (Iconic)
und ROS2 (Jazzy) bestehen. Die Simulationsumgebung u.a. Welt, physic, 3d-Modelle, Roboterkenimatic, Sensoren,  werden in Gazebo erstellt und verwaltet. Währen ROS2 u.a. die Robotersteuerung, Sensordaten
darstellung und Navigationsverfahren in ROS2 übernehmen wird.
Die Umsetzung wird in folgenden Subbereiche unterteilt und daraus werden wird folgend unser todo
herrauskrystallisiert.
1) System Setup:
   1.1) Simulationsumgebung (Gazebo):
        1.1.1) 3D-Modelle
        1.1.2) Physik
        1.1.3) Kinematic
        1.1.4) Robotermodell
               1.1.4.1) Roboterkenematic
        1.1.5) Kollision
        1.1.6) Sensoren
               1.1.6.1) Sensordatenerfassung
        1.1.7) Robotersteuerung
               1.2.1.1) IstPositions-/Lageerfassung
               1.2.1.2) Sollpositions vorgabe
   1.2) Steuerungungsumgebung (ROS2):
        1.2.2) Sensordaten Darstellung
               1.2.2.1) Grafische Darstellung der Sensordaten
   1.3) Datenaustausch (ROS2 <==> Gazebo)
        1.3.2) Publisher-Subscriber Model (Gzbridge)
2) Enwicklung, Bildbasierten Navigationsverfahren:
   2.1) Vorstellung gängige Bildbasierte Navigationsverfahren (Stand der Technik)
   2.2) Entwicklung eingene bildbasiertes Navigationverfahren
        2.2.1) Eingangsdaten (Sensor daten) Auswertung
        2.2.1) Difintion der Navigations-Parameter
        2.3.3) Ausgangsdaten (Sollpositions vorgabe) beschreiben
        2.3.4) Implementierung der Navigationsverfahren in einer Schrittkette
   2.3) Anwendung andere Naivgation verfahren
        2.3.1) Navigation => "http://wiki.ros.org/navigation"
        2.3.2) Moveti => "https://moveit.ai/documentation/concepts/"
        2.3.3) sbpl_lattice_planner => "https://wiki.ros.org/sbpl_lattice_planner"
3) Systemanalyse
   3.1) Navigationsverfahren vergleich anhand der Vorgabeparameter
        3.1.1) Mindestzeit
        3.1.2) Minimalestrecke
        3.1.3)

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
