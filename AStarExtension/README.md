# Projektaufgabe: Erweiterung des A*

| Author             | Contact                         |
|--------------------|---------------------------------|
| Paul Hannuschka    | paul.hannuschka@student.kit.edu |
| Eike Simon Scharpf | urxuj@student.kit.edu           | 

# Installation
Installation der benötigten Pakete in einer Python 3.10 Umgebung:

```bash
pip install -r requirements.txt
```

# Projektbeschreibung
## Aufgabe 1: Erweiterung des A* Algorithmus
### 1. Veränderliche Diskretisierung
Führen Sie eine veränderliche DIskretisierung ein. In der derzeitigen Implementierung wird sehr rudimentär die Schrittweise 1 angenommen, d.h. werden die Grenzen 0-25 angenommen, entspricht dies 25 Schritten.
Ermöglichen Sie es daher, den gegebenen Bereich in in beliebige zu Beginn einer Planung fest vorgegebene Anzahl von Schritten aufzuteilen, z.B. 0-25 in 100, 221 oder auch 1000 Schritte.
Dabei soll jede Achse des Konfigurationsraums unterschiedlich diskretisierbar sein.

Start- und Endwert könenn eventuell nicht auf diesem Gitter liegen. Wie können Sie dennoch einen kollisionsfreien Pfad von Start zu Ziel realisieren? Implementieren Sie eine geeignete Strategie.

Derzeit wird nur der Knoten auf Kollision überprüft. Realisieren Sie die Möglichkeit per Konfiguration einzuschalten, dass auch die Verbindung zwischen zwei Knoten auf Kollision überprüft wird.

### 2. Konfigurierbares Reopening
Derzeit führt der A* kein Reopening durch. Machen Sie konfigurierbar, ob Reopening verwendet oder nicht verwendet werden soll.

### 3. Vergleich anhand von 5 2-DoF-Benchmarks

Zeigen Sie die Ergebnisse grafisch und diskutieren Sie im Anschluss.
Die Variationen und Kriterien sind frei kombinierbar.

| Variationen                     |  Metriken    |
|---------------------------------|--------------|
| # Diskretisierungsschritte      | Suchzeit     |
| Kollision der Verbindung an/aus | Größe Roadmap |
| Gewichtungsfaktor w             | Anzahl Punkte im Pfad |
| Reopening an/aus                | Länge des Pfades | 

### 4. Vergleich anhand von 3 Planar-Benchmarks

Kollisionsüberprüfung der Verbindung soll immer an sein.
Zeigen Sie die Ergebnisse grafisch und diskutieren Sie im Anschluss.
Die Variationen und Kriterien sind frei kombinierbar.

| Variationen                | Metriken              |
|----------------------------|-----------------------|
| # Freiheitsgrade           | Suchzeit              |
| # Diskretisierungsschritte | Größe Roadmap         |
| Gewichtungsfaktor w        | Anzahl Punkte im Pfad |
| Reopening an/aus           | Länge des Pfades      |

### 5. Vergleich mit LazyPRM
Vergleichen Sie die Benchmark Umgebungen aus 4. mit LazyPRM. Wählen Sie geeignete Kriterien, stellen Sie die Ergebnisse grafisch dar und diskutieren Sie diese.

### 6. Erzeugen Sie die Animation der Suche in jedem Schritt des A*, die es ermöglicht im 2-DoF Fall das Verhalten exakt nachzuvollziehen.

## Aufgabe 2: Weitere Erläuterungen
### 1. Bewegungsbefehle
Mit welchen Bewegungsbefehlen würden Sie den Ergebnispfad des A* in ein Roboterprogramm umwandeln? Erläutern Sie, warum das zwar möglich, aber nicht unbedingt sinnvoll ist.
### 2. Praktische Anwendung
Was müssen Sie machen, damit er sinvoll abgefahren werden kann? Erläutern Sie die Vorgehensweise (nicht programmieren).

# Projektaufbau
Der erweitere A* Algorithmus ist in der Datei `core/IPAStarExtended.py` implementiert. Die schrittweise Visualisierung ist in `core/IPVISAStar.py` implementiert.
Eine schnellere Version des planaren Manipulators ist in der Datei `core/PlanarManipulator.py` implementiert.


Die Erweiterungen können im Notebook `AStar_extension.ipynb` auf Beispielumgebungen getestet werden.

Die 2-DoF Benchmarks können im Notebook `AStar_benchmark_point.ipynb` visualisiert und getestet werden.

Die 3-, 6- und 9-DoF Benchmarks können im Notebook `AStar_benchmark_planar.ipynb` visualisiert und getestet werden.

Gespeicherte Benchmark Ergebnisse und Visualisierungen sind im Ordner `evaluation/2DoFresults` zu finden.