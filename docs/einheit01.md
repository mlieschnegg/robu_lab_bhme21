# Einheit 01 – CandyBot vorbereiten & Master/Slave Steuerung

## Ziele
- ROS2-Workspace mit vorgegebenem Repository einrichten.  
- CandyBot-Simulation testen.  
- URDF erweitern und ein STL-Modell einbinden.  
- Hardware (Dynamixel + Waveshare-Adapter) vorbereiten und testen.  
- Master/Slave-Steuerung mit Publisher/Subscriber implementieren.  

---

## Aufgabenstellung

### 1. Repository einrichten & Simulation testen
- Ladet das Repository herunter:  
  [https://github.com/mlieschnegg/robu_lab_ahmba21](https://github.com/mlieschnegg/robu_lab_ahmba21)  
- Enthalten sind:  
  - **Anleitung** (README, Dokumentationshinweise)  
  - **CandyBot Master/Slave Roboter** mit `bringup`, `controller` und `description` Packages  
- Führt die Simulation aus und überprüft, ob sich der CandyBot in RViz/Gazebo korrekt starten lässt.  

---

### 2. URDF erweitern – Zuckerlbox
- Für den **Follower-Roboter** gibt es eine **Zuckerlbox als STL-Datei** (cage.stl).  
- Die Zuckerlbox soll im URDF des Followers eingebunden werden:  
  - Erzeugt ein **fixed joint** zwischen Roboter und Box.  
- Bearbeitet die STL zuvor mit **Blender**:  
  - Wände **transparent** machen, damit man die Zuckerl sehen kann.  
  - Boden mit einer **Holz-Textur** versehen.  

---

### 3. Hardware – Waveshare-Adapter für Dynamixel
- Die Dynamixel-Motoren werden über den **Waveshare 2PCS Serial Bus Servo Driver Board** angesteuert:  
  [Produktlink](https://www.amazon.de/dp/B0DK79JNNK?ref=ppx_yo2ov_dt_b_fed_asin_title)  
- Baut den Adapter so um, dass **Dynamixel-Stecker** anschließbar sind.  
- An einem Stecker müssen **12 V Versorgung** ausgegeben werden, am anderen Stecker **5 V**.  

---

### 4. Dynamixel-Motoren testen
- Nachdem der Adapter angepasst wurde, können die **Dynamixel-Motoren einzeln getestet** werden.  
- Verwendet dazu die Software **Dynamixel Wizard**.  
- Überprüft:  
  - Wird der Motor erkannt?  
  - Lässt er sich ansteuern?  
  - Stimmen ID und Baudrate?  

---

### 5. Test mit Waveshare-Konvertern
- Verwendet das bisherige Steuerungsprogramm für den CandyBot.  
- Testet die Steuerung über die Waveshare-Konverter.  
- Dokumentiert Funktion, Probleme und mögliche Lösungen.  

---

### 6. Master/Slave Publisher & Subscriber
- Implementiert einen ROS2 **Publisher**, der die **Motorpositionen des Leader-Roboters** veröffentlicht.  
- Implementiert einen **Subscriber**, der diese Positionen beim **Follower-Roboter** übernimmt.  
- Testet die Kommunikation:  
  - Leader bewegen → Follower soll dieselben Bewegungen ausführen.  

---

## Dokumentation
- Dokumentiert eure Arbeitsschritte und Ergebnisse in `docs/einheit01.md`.  
- Fügt Screenshots von RViz/Gazebo, URDF und ggf. Fotos der Hardware ein.  
- Speichert Bilder im Ordner `images/einheit01/`.  
- Nutzt die Vorlage aus [ANLEITUNG.md](../ANLEITUNG.md).  

# Kommentar von LI: 
Bitte die Doku verbessern (Bessere Beschreibung, Bilder, ... Das müssen Oma und Opa nachvollziehen können.

### 1. Bearbeitung des Cage mittels Blender (16.9.2025, Hausch und Schweitzer)
- STL in Blender eingefügt und versucht Material auf transparant zu ändern und in Gazebo zu importieren
- STL war in Blender transparent allerding nicht in Gazebo 
- Auf 0.001 sklaiert für richtige Größe

### 2. Aufsetzen des Betriebssystems für den CandyBot (16.9.2025, Huss und Frieß)

Es wurde nach dem Quick Start Guide (Jazzy) des Turtlebot3 vorgegangen: 
https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup

Installation von 3.2.SBC Setup
- PC mit den gleichen Hotspot wie der Raspberry verbinden
- PC verbinden mit: ssh robu@172.20.10.14 (IP-Adresse des Rpi)
- die IP-Adresse wurde vom Display des Raspberrys (waveshare 4.3 zoll Display) abgelesen -> hostname -I
- alternativ könnte man auch die IP-Adresse am Handy ablesen

beginnend bei Schritt 3.3.2.Raspberry Pi Camera
- die Befehle wurden bis zum Schritt 3 befolgt
- bei Schritt 4 - launchen der Kamera Node - tritt ein Fehler auf. Es ist keine Kamera verfügbar.
- in der /boot/firmware/config.txt wurde der device tree overlay für die Raspberry Kamera V2 aktiviert (dtoverlay=imx219) führte zu keiner Verbesserung und die Kamera wurde weiterhin nicht erkannt
- LI: Lösung des Kameraproblems: Es fehlten die Zugriffsrechte des Benutzers robu auf die Kamera. Der Benutzer robu muss zur Gruppe video und render hinzugefügt werden (sudo usermod -aG video,render $USER). Die Kamera läuft nun.

### 3. Es fehlt Bayer und Scheucher (haben den Adapter umgebaut)...
### 4. Divjak hat am Balancer gearbeitet....
