# LEGO type:standard slot:0 autostart

DEPLOYMENT = False

import math
import turtle
import time

screen = turtle.getscreen()
t = turtle.Turtle()
t.speed(10)

class Draw():
    def __init__(self):
        pass
    
    def DrawLine(self, x1, y1, x2, y2):
        t.penup()
        t.goto(x1, y1)
        t.goto(x2, y2)
        t.pendown()

class Driver():
    def __init__(self):

        self.COLOR_SENSOR_TRIGGER = 32


    def LineFollower(self, distance, speed):

        if DEPLOYMENT:

            while distanceSensor.get_distance_cm(True) <= distance:

                drivePair.start((colorSensor.get_reflected_light() - self.COLOR_SENSOR_TRIGGER) * 0.5, speed)
        else:

            print("Inside LineFollower, speed: " + speed + "distance: " + distance)
    

    def DriveToColor(self, color, speed, direction):
        if DEPLOYMENT:
            drivePair.start(direction, speed)

            colorSensor.wait_until_color(str(color))

            drivePair.stop()
        else:
            print("driving to color: " + str(color))
    def TurnStationary(self, targetAngle, turnDirection='left'):
        if isinstance(targetAngle, float):
            print("WARNING: target angle is not accurate")
        if targetAngle > 180:
            ceilTargetAngle = math.ceil(targetAngle  / 180.0)
        else:
            ceilTargetAngle = math.ceil(targetAngle)
        if DEPLOYMENT:
            if turnDirection == 'left':
                drivePair.start_tank(10, -10)
            else:
                drivePair.start_tank(-10, 10)
            
            wait_until(hub.motion_sensor.get_yaw_angle, equal_to, ceilTargetAngle)
            
            drivePair.stop
        else:
            print("Turning to angle: " + str(ceilTargetAngle))
        return ceilTargetAngle

class Field():
    def __init__(self):
        pass
        
    
        
class FieldDriver():
    def __init__(self, startPositionX, startPositionY):
        self.START_POSITION_X = startPositionX
        self.START_POSITION_Y = startPositionY
        self.currentPositionX = startPositionX
        self.currentPositionY = startPositionY
        self.FIELD = []
        self.FieldSizeX = 114
        self.FieldSizeY = 200
        self.obstacles = [[0, 0, self.FieldSizeX, 0],
                          [0, 0, 0, self.FieldSizeY],
                          [0, self.FieldSizeY, self.FieldSizeX, self.FieldSizeY],
                          [self.FieldSizeX, 0, self.FieldSizeX, self.FieldSizeY]]
        #draw field borders
        for i in self.obstacles:
            t.penup()
            t.goto(i[0], i[1])
            
            t.pendown()
            t.goto(i[2], i[3])
            
        t.penup()
        t.goto(self.START_POSITION_X, self.START_POSITION_Y)
        t.pendown()
        
        self.astarStraightLineWeight = 0.0
    def MoveToPosition(self, positionX, positionY):
        diffX = positionX - self.currentPositionX
        diffY = positionY - self.currentPositionY
        targetAngle = math.degrees(math.atan(diffY / diffX))
        distanceToDestination = diffX / math.cos(targetAngle)
        driver = Driver()
        if DEPLOYMENT:
            driver.TurnStationary(targetAngle, 'left')
            drivePair.move(distanceToDestination)
            print(str(distanceToDestination))
        else:
            print("diffX:" + str(diffX) + "diffY:" + str(diffY))
            print("direction:" + str(math.degrees(targetAngle)))
            print("distance to destination:" + str(distanceToDestination))
        del driver
    
    def CalibrateDistanceToWall(self, Field):
        closestWall = Field.GetClosestWall(self.currentPosition)
        distanceSensorTurnTolerance = 5
        if closestWall[1] == "north":
            self.driver.TurnStationary(0 + Driver.DistanceSensorOffset - distanceSensorTurnTolerance)
        elif closestWall[1] == "south":
            self.driver.TurnStationary(90 + Driver.DistanceSensorOffset - distanceSensorTurnTolerance)
        elif closestWall[1] == "west":
            self.driver.TurnStationary(180 + Driver.DistanceSensorOffset - distanceSensorTurnTolerance)
        else:
            self.driver.TurnStationary(270 + Driver.DistanceSensorOffset - distanceSensorTurnTolerance)
        
        #turn until the distance sensor sees targetWall 
        motorPair.start_tank(5, -5)
        distanceSensor.wait_for_distance_closer_than(closeestWall, 'cm')    
        motorPair.stop()
        #reset gyro
        
    def GetField(self):

        return self.FIELD
    
    def GetDistanceToWall(self,currentPositionX, currentPositionY, targetWall):
        if targetWall == "north":
            return currentPositionY
        elif targetWall == "south":
            return abs(currentPositionY - self.FieldSizeX)
        elif targetWall == "east":
            return currentPositionX
        elif targetWall == "west":
            return abs(currentPositionX - self.FieldSizeY)
        else:
            raise Exception("Unknown targetWall")
    
    def GetClosestWall(self, currentPosition):
        wallDistances = [self.GetDistanceToWall(currentPosition, "north"),
                         self.GetDistanceToWall(currentPosition, "south"),
                         self.GetDistanceToWall(currentPosition, "east"),
                         self.GetDistanceToWall(currentPosition, "west")]
        if not DEPLOYMENT:
            print("wallDistances:", wallDistances)
        return wallDistances.index(min(wallDistances))
        
    def DrawStraightLine(self, pointA_X, pointA_Y, pointB_X, pointB_Y):
        self.obstacles.append([pointA_X, pointA_Y, pointB_X, pointB_Y])
        
    def GetSurroundingFields(self, currentPositionX, currentPositionY):
        return [
            (currentPositionX + 1, currentPositionY + 1),
            (currentPositionX - 1, currentPositionY - 1),
            (currentPositionX + 1, currentPositionY - 1),
            (currentPositionX - 1, currentPositionY + 1),
            (currentPositionX - 1, currentPositionY - 1),
            (currentPositionX + 1, currentPositionY),
            (currentPositionX - 1, currentPositionY),
            (currentPositionX, currentPositionY + 1),
            (currentPositionX, currentPositionY - 1)
        ]
        
    def CheckForCollisions(self, currentPositionX, currentPositionY, targetPositionX, targetPositionY, collisionLineStartX, collisionLineStartY, collisionLineEndX, collisionLineEndY):
        LambdaDividend = currentPositionX * collisionLineEndY - currentPositionX * collisionLineStartY - collisionLineStartX * collisionLineEndY - collisionLineStartX * collisionLineStartY - currentPositionY * collisionLineEndX + currentPositionY * collisionLineStartX + collisionLineStartY * collisionLineEndX + collisionLineEndX * collisionLineStartX
        LambdaDivisor = targetPositionY * collisionLineEndX - targetPositionY * collisionLineStartX - currentPositionY * collisionLineEndX - currentPositionY * collisionLineStartX - targetPositionX * collisionLineEndY + targetPositionX * collisionLineStartX + currentPositionX * collisionLineEndY + currentPositionX * collisionLineStartY
        try:
            LambdaResult1 = LambdaDividend / LambdaDivisor
        except ZeroDivisionError:
            LambdaResult1 = math.inf
        VectorX = targetPositionX + LambdaResult1 * (targetPositionX - currentPositionX)
        VectorY = targetPositionY + LambdaResult1 * (targetPositionY - currentPositionY)
        
        # Check intersection between target and currentPosition
        try:
            slope = (currentPositionY - targetPositionY) / (targetPositionX - currentPositionX)
        except ZeroDivisionError:
            slope = math.inf
        VectorOnLine = (VectorY - targetPositionY) == slope * (VectorX - targetPositionX)
        VectorIsBetween = (min(currentPositionX, targetPositionX) <= VectorX <= max(currentPositionX, targetPositionX)) and (min(currentPositionY, targetPositionY) <= VectorY <= max(currentPositionY, targetPositionY))
        if not VectorIsBetween and not VectorOnLine:
            return False
        
        # Check intersection between target and currentPosition obstacles
        try:
            slope = (collisionLineStartY - collisionLineEndY) / (collisionLineEndX - collisionLineStartX)
        except ZeroDivisionError:
            slope = math.inf
        VectorOnLine = (VectorY - collisionLineEndY) == slope * (VectorX - collisionLineEndX)
        VectorIsBetween = (min(collisionLineStartX, collisionLineEndX) <= VectorX <= max(collisionLineStartX, collisionLineEndX)) and (min(collisionLineStartY, collisionLineEndY) <= VectorY <= max(collisionLineStartY, collisionLineEndY))
        if not VectorIsBetween and not VectorOnLine:
            return False
        else:
            return True
    
    def GetAStarPath(self, targetPositionX, targetPositionY):
        diffX = targetPositionX - self.currentPositionX
        diffY = targetPositionY - self.currentPositionY
        targetAngle = math.degrees(math.atan(diffY / diffX))
        distanceToDestination = diffX / math.cos(targetAngle)
        # Determine if astar is neccessary
        collision = False
        for i in self.obstacles:
            collision = self.CheckForCollisions(self.currentPositionX, self.currentPositionY, targetPositionX, targetPositionY, i[0], i[1], i[2], i[3])
            if not collision:
                return [targetPositionX, targetPositionY]
        MoveChain = []
        # calculate astar path
        if collision:
            onDestination = False

            while not onDestination:
                prevMoveChainElement = [math.inf , math.inf]
                bestMoveDirectionX, bestMoveDirectionY, bestAstarValue = 0, 0, math.inf

                for x, y in self.GetSurroundingFields(self.currentPositionX, self.currentPositionY):
                    
                    invalidMoveOption = False
                    
                    for obstacle in self.obstacles:
                        
                        if self.CheckForCollisions(self.currentPositionX, self.currentPositionY, x, y, obstacle[0], obstacle[1], obstacle[2], obstacle[3]):
                        
                            # invalidMoveOption = True
                            invalidMoveOption = False
                            continue
                    if invalidMoveOption:
                        continue
                    
                    astarDistance = math.sqrt(math.pow(x - self.currentPositionX, 2) + math.pow(y - self.currentPositionY, 2)) + 1
                    if not (prevMoveChainElement[0] * -1) == x and not (prevMoveChainElement[1] * -1) == y:
                        astarDistance -= self.astarStraightLineWeight
                        
                    if astarDistance < bestAstarValue:
                        bestMoveDirectionX = x
                        bestMoveDirectionY = y
                        bestAstarValue = astarDistance
                if prevMoveChainElement[0] == bestMoveDirectionX and prevMoveChainElement[1] == bestMoveDirectionY:
                    MoveChain[len(MoveChain) - 1] = [bestMoveDirectionX, bestMoveDirectionY]
                else:
                    MoveChain.append([bestMoveDirectionX, bestMoveDirectionY])
                prevMoveChainElement = [bestMoveDirectionX, bestMoveDirectionY]
                print(prevMoveChainElement)
                time.sleep(0.5)
                if prevMoveChainElement[0] == targetPositionX and prevMoveChainElement[1] == targetPositionY:
                    onDestination = True
            
                
        return MoveChain

driver = Driver()

fieldDriver = FieldDriver(5, (200 - 5))

field = Field()

draw = Draw()

print(fieldDriver.GetAStarPath(80, 100))

for target in fieldDriver.GetAStarPath(80, 100):
    # fieldDriver.MoveToPosition(target[0], target[1])
    t.pendown()
    t.goto(target[0], target[1])



time.sleep(3)