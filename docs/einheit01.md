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

### Blender (16.9.2025)
- STL in Blender eingefügt und versucht Material auf transparant zu ändern und in Gazebo zu importieren
- STL war in Blender transparent allerding nicht in Gazebo 
- Auf 0.001 sklaiert für richtige Größe
