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
# Projektaufbau und ausführbare Notebooks
- Der erweitere A* Algorithmus ist in der Datei `core/IPAStarExtended.py` implementiert. 
- Die schrittweise Visualisierung ist in `core/IPVISAStar.py` implementiert.  Für die Hindernisse wurde eine Methode in core/IPENvironment.py hinzugefügt.
- Eine schnellere Version des planaren Manipulators ist in der Datei `core/PlanarManipulator.py` implementiert.

Eine **Erklärung und beispielhafte Visualisierung** der Erweiterungen ist in `AStar_extension_explained.ipynb` zu finden.

Die Erweiterungen können im Notebook `AStar_extension_interactive.ipynb` auf Beispielumgebungen **interaktiv getestet** werden.

Die **2-DoF Benchmarks** können im Notebook `AStar_benchmark_point.ipynb` visualisiert und getestet werden. 
Der Benchmark Code befindet sich in `evaluation/two_dof`.

Die **3-, 6- und 9-DoF Benchmarks** können im Notebook `AStar_benchmark_planar.ipynb` visualisiert und getestet werden.
Der Benchmark Code befindet sich in `evaluation/robotic_arm`.

Gespeicherte Benchmark Ergebnisse und Visualisierungen sind in den Ordnern `evaluation/two_dof/results` und `evaluation/robotic_arm/results` zu finden.