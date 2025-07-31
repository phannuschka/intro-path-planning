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
**Führen Sie eine veränderliche Diskretisierung ein. In der derzeitigen Implementierung wird sehr rudimentär die Schrittweise 1 angenommen, d.h. werden die Grenzen 0-25 angenommen, entspricht dies 25 Schritten.
Ermöglichen Sie es daher, den gegebenen Bereich in in beliebige zu Beginn einer Planung fest vorgegebene Anzahl von Schritten aufzuteilen, z.B. 0-25 in 100, 221 oder auch 1000 Schritte.
Dabei soll jede Achse des Konfigurationsraums unterschiedlich diskretisierbar sein.**

Zuerst berechnen wir anhand der angegebenen Anzahl Diskretisierungsschritte und der Szenenlimits die Schrittweite entlang jeder Achse `core/IPAStarExtended.py Z.136-141`.
Anschließend benutzen wir die berechnete Schrittweite, um in `_handleNode()` die benachbarten Knoten zu erzeugen `core/IPAStarExtended.py Z.336`.

**Start- und Endwert können eventuell nicht auf diesem Gitter liegen. Wie können Sie dennoch einen kollisionsfreien Pfad von Start zu Ziel realisieren? Implementieren Sie eine geeignete Strategie.**

Durch unsere gewählte Strategie liegt der Startknoten immer auf dem Gitter. Der Endknoten hingegen kann außerhalb des Gitters liegen.
Deshalb überprüfen wir ob wir das Ziel gefunden haben nicht mehr anhand von GLeichheit der Position, sondern überprüfen ob der aktuell expandierte Knoten innerhalb derselben Gridzelle wie das Ziel liegt `core/IPAStarExtended.py Z. 275-281`.
Ist dies der Fall, überprüfen wir, ob die Verbindung zum Knoten kollisionsfrei ist.
Falls ja, wir der Zielknoten an den Pfad angehängt und die Suche beendet `core/IPAStarExtended.py Z. 202-207`.

**Derzeit wird nur der Knoten auf Kollision überprüft. Realisieren Sie die Möglichkeit per Konfiguration einzuschalten, dass auch die Verbindung zwischen zwei Knoten auf Kollision überprüft wird.**

Die Kollisionsüberprüfung der Verbindung zwischen zwei Knoten kann durch Setzen des Parameters `check_connection` in der Config auf `True` aktiviert werden.
Eine erste naive Idee war es, die Kollisionsüberprüfung vor dem Einfügen in die OpenList durchzuführen `core/IPAStarExtended.py Z. 342-344`.
Dies ist allerdings nicht sinnvoll, da wir die sehr teure Kollisionsüberprüfung für Knoten durchführen, die wir eventuell nie expandieren.

Deshalb haben wir den zweiten Parameter `lazy_check_connection` eingeführt, der es ermöglicht, die Kollisionsüberprüfung der Verbindung erst dann durchzuführen, wenn der Knoten expandiert wird `core/IPAStarExtended.py Z. 180-188`.
Da der Knoten hier bereits in der OpenList ist, reparieren wir den Knoten, falls die Verbindung zum Father in Kollision ist.
Dazu betrachten wir alle anderen Nachbarn, die bereits closed sind, und prüfen, ob eine Verbindung zu einem dieser Knoten kollisionsfrei ist.
Falls ja, wird der Knoten repariert und die Verbindung zum Father aktualisiert `core/IPAStarExtended.py Z. 221-259`.
Falls nein, setzen wir den Knoten auf unendlich `core/IPAStarExtended.py Z. 188`. Eventuell findet also später Reopening statt.


### 2. Konfigurierbares Reopening
**Derzeit führt der A* kein Reopening durch. Machen Sie konfigurierbar, ob Reopening verwendet oder nicht verwendet werden soll.**

Reopening kann durch Setzen des Parameters `reopening` in der Config auf `True` aktiviert werden.
Reopening wird in der Methode `_handleNode()` durchgeführt `core/IPAStarExtended.py Z. 348 - 364`.
Hier wird überprüft, ob der Knoten bereits in der OpenList ist und ob der neue Pfad zu diesem Knoten kürzer ist als der bisherige Pfad.
Falls ja, werden die Kosten des Knoten aktualisiert. Insbesondere muss der Knoten aus der OpenList entfernt und wieder hinzugefügt werden, was relativ teuer ist `core/IPAStarExtended Z. 308 - 312`.

### 3. Vergleich anhand von 5 2-DoF-Benchmarks

**Zeigen Sie die Ergebnisse grafisch und diskutieren Sie im Anschluss.
Die Variationen und Kriterien sind frei kombinierbar.**

| Variationen                     |  Metriken    |
|---------------------------------|--------------|
| # Diskretisierungsschritte      | Suchzeit     |
| Kollision der Verbindung an/aus | Größe Roadmap |
| Gewichtungsfaktor w             | Anzahl Punkte im Pfad |
| Reopening an/aus                | Länge des Pfades | 

Siehe Notebook `AStar_benchmark_point.ipynb` für die Visualisierung und Auswertung der Benchmarks.

### 4. Vergleich anhand von 3 Planar-Benchmarks

**Kollisionsüberprüfung der Verbindung soll immer an sein.
Zeigen Sie die Ergebnisse grafisch und diskutieren Sie im Anschluss.
Die Variationen und Kriterien sind frei kombinierbar.**

| Variationen                | Metriken              |
|----------------------------|-----------------------|
| # Freiheitsgrade           | Suchzeit              |
| # Diskretisierungsschritte | Größe Roadmap         |
| Gewichtungsfaktor w        | Anzahl Punkte im Pfad |
| Reopening an/aus           | Länge des Pfades      |

Siehe Notebook `AStar_benchmark_planar.ipynb` für die Visualisierung und Auswertung der Benchmarks.

### 5. Vergleich mit LazyPRM
**Vergleichen Sie die Benchmark Umgebungen aus 4. mit LazyPRM. Wählen Sie geeignete Kriterien, stellen Sie die Ergebnisse grafisch dar und diskutieren Sie diese.**

Siehe Notebook `AStar_benchmark_planar.ipynb` für die Visualisierung und Auswertung der Benchmarks.

### 6. Erzeugen Sie die Animation der Suche in jedem Schritt des A*, die es ermöglicht im 2-DoF Fall das Verhalten exakt nachzuvollziehen.

Da die ursprüngliche Animation der Ergebnisse in MAtplotlib ca. 2 Sekunden pro Frame benötigt, haben wir uns für die schrittweise Visualisierung mittels OpenCV entschieden.
Hierbei wird innerhlab kurzer Zeit jeder Schritt als png-Datei gespeichert, die anschließend zu einem Video zusammengefügt werden kann.
Die Implementierung befindet sich in `core/IPVISAStar.py Z. 502`.
Um nicht immer den gesamten Graph speichen zu müssen, wird nur pro Schritt im A* nur ein delta gespeichert `core/IPAStarExtended.py Z. 59`.


## Aufgabe 2: Weitere Erläuterungen
### 1. Bewegungsbefehle
**Mit welchen Bewegungsbefehlen würden Sie den Ergebnispfad des A\* in ein Roboterprogramm umwandeln? Erläutern Sie, warum das zwar möglich, aber nicht unbedingt sinnvoll ist.**

Der Ergebnispfad des A* Algorithmus besteht aus einer Reihe von diskreten Konfigurationen auf einem Grid, die der Roboter nacheinander anfahren soll.
Dies kann beispielsweise durch lineare Interpolation zwischen den Konfigurationen realisiert werden, wobei der Roboter in jedem Schritt eine neue Konfiguration ansteuert.
Dies ist allerdings nicht unbedingt sinnvoll: Einerseits würde durch abrupte Richtungsänderungen ruckartige Bewegungen entstehen.
Durch die konstanten Starts und Stops würde der Roboter zudem unnötig Zeit verlieren, da er in jedem Schritt zum Stillstand kommen und dann wieder beschleunigen müsste.
Verstärkt wird dieser Umstand noch, wenn keine Diagonalbewegungen erlaubt sind, da der Roboter dann nur einen Gelenkwinkel auf einmal ändern kann.

### 2. Praktische Anwendung
**Was müssen Sie machen, damit er sinnvoll abgefahren werden kann? Erläutern Sie die Vorgehensweise (nicht programmieren).**

Um den Ergebnispfad des A* Algorithmus sinnvoll abfahren zu können, müssen mehrere Schritte unternommen werden:
1. **Smoothing**: Wie in der Vorlesung / den Vorträgen besprochen, würde der von A* geplante Pfad stark von Smoothing profitieren. Durch die kleinschrittige Planung auf dem Grid verfügt der Pfad über viele unnötige Wegpunkte.
Durch Smoothing werden zum einen redundante Wegpunkte auf geraden Strecken entfernt, und eventuelle abrupte Richtungsänderungen minimiert. 
2. **Interpolation**: Nach dem Smoothing bleibt eine reduzierte Anzahl diskreter Konfigurationen mit abrupten Übergängen. 
Um nicht nur die diskreten Konfigurationen abzufahren, sondern auch die Zwischenräume zu nutzen, kann eine Interpolation zwischen den Konfigurationen durchgeführt werden.
Dies kann beispielsweise durch Parabolische Interpolation oder Splines geschehen, um eine glattere Bewegung zu erzeugen.



# Projektaufbau
Der erweitere A* Algorithmus ist in der Datei `core/IPAStarExtended.py` implementiert. Die schrittweise Visualisierung ist in `core/IPVISAStar.py` implementiert.
Eine schnellere Version des planaren Manipulators ist in der Datei `core/PlanarManipulator.py` implementiert.


Die Erweiterungen können im Notebook `AStar_extension.ipynb` auf Beispielumgebungen getestet werden.

Die 2-DoF Benchmarks können im Notebook `AStar_benchmark_point.ipynb` visualisiert und getestet werden.

Die 3-, 6- und 9-DoF Benchmarks können im Notebook `AStar_benchmark_planar.ipynb` visualisiert und getestet werden.

Gespeicherte Benchmark Ergebnisse und Visualisierungen sind im Ordner `evaluation/two_dof/results` zu finden.