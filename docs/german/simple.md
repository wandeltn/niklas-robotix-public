# Einfache deutsche Dokumentation

## Globale Konstanten

- DEPLOYMENT = `bool`

## Port Belegung

| Anschluss      | Port |
|----------------|------|
| Greif Motor    |      |
| Arm Motor      |      |
| Antrieb rechts |      |
| Antrieb links  |      |
| Distanz Sensor |      |
| Farbsensor     |      |

## `class Driver`

> Eine Klasse die sich Speziellen Fahrfunktionen wie 'Linie Folgen' kümmert.

### Driver Initialisierung

```python
__init__(self):
    self.COLOR_SENSOR_TRIGGER = 32
```

Variablen:

- `COLOR_SENSOR_TRIGGER` - Wann der Farbsensor auf das welchseln einer Farbe reagieren soll.

</br>

### Driver Funktionen

***

```python
LineFollower(self, distance: float, speed: int)
```

> Nutzt einen Farbsensor um einer schwarzen Linie zu folgen.

Übergabe Argumente:

- `distance` - Distanz die er der Linie folgen soll.
- `speed` - Geschwindigkeit die er beim folgen haben soll.

</br>

```python
DriveToColor(self, color: string, speed: int, direction: int)
```

Übergabe Argumente:

- `color` - Farve in der er anhalten soll.
- `speed` - Geschwindigkeit die er fahren soll.
- `direction` - Richtung von -90 bis 90 in die er fahren soll.

</br>

```python
TurnStationary(self, targetAngle: float, turnDirection: str)
```

Übergabe Argumente:

- `targetAngle` - Gyro Wert zu welchem er sich drehen soll.
- `turnDirection` - Richtung in die er sich drehen soll.

## `class FieldDriver`

### Initialisierung

```python
__init__(self), startPositionX: int, startPositionY: int): 
    self.START_POSITION_X = startPositionX
    self.START_POSITION_Y = startPositionY
    self.currentPositionX = startPositionX
    self.currentPositionY = startPositionY

    self.fieldSizeX = 200
    self.fieldSizeY = 114
    self.obstacles = [[0, 0, self.FieldSizeX, 0],
                        [0, 0, self.FieldSizeY, 0],
                        [self.FieldSizeY, 0, self.FieldSizeX, self.FieldSizeY],
                        [self.FieldSizeX, 0, self.FieldSizeY, self.FieldSizeX]]
    self.astarStraightLineWeight = 0.5
```

Übergabe Argumente:

- `startPositionX` - Start position des Roboters in der X-Achse.
- `startPositionY` - Start position des Roboters in der Y-Achse.

Variablen:

- `self.START_POSITION_X` - Start position des Roboters in der X-Achse.
- `self.START_POSITION_Y` - Start position des Roboters in der Y-Achse.
- `self.currentPositionX` - Current position des Roboters in der X-Achse.
- `self.currentPositionY` - Current position des Roboters in der Y-Achse.
- `self.fieldSizeX` - Feld größe in der X-Achse.
- `self.fieldSizeY` - Feld größe in der Y-Achse.
- `self.obstacles` - Hindernisse am Anfang des Programms.
- `self.astarStraightLineWeight` - Gewichtung des Geradeausfahen bei dem Path finding.

</br>

### FieldDriver Funktionen

```python
GetDistanceToWall(self, targetWall: str, currentPositionX= self.currentPositionX: int, currentPositionY= self.currentPositionY: int)
```

Übergabe Argumente:

- `targetWall` - Richtung der Wand von der er die Distanz berechnen soll.
- `currentPositionX` - Position auf der X-Achse von der die Distanz zur Wand berechen soll.
- `currentPositionY` - Position auf der Y-Achse von der die Distanz zur Wand berechen soll.

</br>

```python
MoveToPosition(self, positionX: int, positionY: int)
```

Übergabe Argumente:

- `positionX` - Position auf der X-Achse zu der er fahren soll.
- `positionY` - Position auf der Y-Achse zu der er fahren soll.

</br>

```python
CalibrateDistanceToWall(self, currentPositionX: int, currentPositionY: int)
```

Übergabe Argumente:

- `currentPositionX` - Position auf der X-Achse.
- `currentPositionY` - Position auf der Y-Achse.

</br>

```python
GetClosestWall(self, currentPositionX: int, currentPositionY: int)
```

Übergabe Argumente:

- `currentPositionX` - Position auf der X-Achse.
- `currentPositionY` - Position auf der Y-Achse.

```python
GetClosestWall(self, pointA_X: int, pointA_Y: int, pointB_X: int, pointB_Y: int)
```

Übergabe Argumente:

- `pointA_X` - Position auf der X-Achse bei welcher die Linie starten soll.
- `pointA_Y` - Position auf der Y-Achse bei welcher die Linie starten soll.
- `pointB_X` - Position auf der X-Achse bei welcher die Linie enden soll.
- `pointB_Y` - Position auf der Y-Achse bei welcher die Linie enden soll.

</br>

```python
GetSurroundingFields(self, currentPositionX: int, currentPositionY: int)
```

Übergabe Argumente:

- `currentPositionX` - Position auf der X-Achse.
- `currentPositionY` - Position auf der Y-Achse.

</br>

```python
CheckForCollisions(self, currentPositionX: int, currentPositionY: int, targetPositionX: int, targetPositionY: int, collisionLineStartX: int, collisionLineStartY: int, collisionLineEndX: int, collisionLineEndY: int)
```

Übergabe Argumente:

- `currentPositionX` - Position auf der X-Achse.
- `currentPositionY` - Position auf der Y-Achse.
- `targetPositionX` - Position auf der X-Achse zu der er überprüfen soll ob sich die Linien überschneiden.
- `targetPositionY` - Position auf der Y-Achse zu der er überprüfen soll ob sich die Linien überschneiden.
- `collisionLineStartX` - Position auf der X-Achse bei der das Hindernis startet.
- `collisionLineStartY` - Position auf der Y-Achse bei der das Hindernis startet.
- `collisionLineEndX` - Position auf der X-Achse bei der das Hindernis endet.
- `collisionLineEndY` - Position auf der Y-Achse bei der das Hindernis endet.

</br>

```python
GetAStarPath(self, targetPositionX: int, targetPositionY: int)
```

Übergabe Argumente:

- `targetPositionX` - Position auf der X-Achse zu der er einen Pfad finden soll.
- `targetPositionY` - Position auf der Y-Achse zu der er einen Pfad finden soll.
