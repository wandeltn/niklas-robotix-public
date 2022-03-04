# LEGO type:standard slot:0
# pyright: reportMissingImports=false

DEPLOYMENT = True

import math

from mindstorms import MSHub, Motor, MotorPair, ColorSensor, DistanceSensor
from mindstorms.control import wait_until
from mindstorms.operator import equal_to

#set ports
hub: type = MSHub() 
hub.status_light.on('orange') # type: ignore
verticalMotor: type = Motor('F')
verticalMotor.set_degrees_counted(0)  # type: ignore
clawMotor: type = Motor('E')
clawMotor.set_degrees_counted(0) # type: ignore
# colorSensorRight = ColorSensor('D')
colorSensor: type = ColorSensor('D') 
distanceSensor: type = DistanceSensor('C') 
tempDriveMotorLeft: type = Motor('A')
tempDriveMotorRight: type = Motor('B') 

tempDriveMotorRight.set_degrees_counted(0) # type: ignore
tempDriveMotorLeft.set_degrees_counted(0) # type: ignore
del tempDriveMotorLeft
del tempDriveMotorRight

drivePair: type = MotorPair('B', 'A')
#set default

colorSensor.light_up_all(50) # type: ignore
# colorSensorRight.light_up_all(100)
distanceSensor.light_up_all(100) # type: ignore
drivePair.set_stop_action('hold') # type: ignore
drivePair.set_motor_rotation(5.6 * math.pi, 'cm') # type: ignore
hub.motion_sensor.reset_yaw_angle() # type: ignore
hub.status_light.on('green') # type: ignore

class Driver():
    def __init__(self):
        self.DistanceSensorTurnOffset = 0
        self.COLOR_SENSOR_TRIGGER = 32
        
    def LineFollower(self, distance: float, speed: int) -> None:
        while distanceSensor.get_distance_cm(True) <= distance: # type: ignore
            drivePair.start((colorSensor.get_reflected_light() - self.COLOR_SENSOR_TRIGGER) * 0.5, speed) # type: ignore
                
    def DriveToColor(self, color: str, speed: int, direction: str) -> None:
        if DEPLOYMENT:
            drivePair.start(direction, speed) # type: ignore
            colorSensor.wait_until_color(str(color)) # type: ignore
            drivePair.stop() # type: ignore
        else:
            print("driving to color: " + str(color))
            
    def TurnStationary(self, targetAngle: float, turnDirection: str='left') -> int:
        GYROSCOPE_TOLERANCE: float = 2.0 
        if targetAngle - GYROSCOPE_TOLERANCE < hub.motion_sensor.get_yaw_angle() and targetAngle + GYROSCOPE_TOLERANCE > hub.motion_sensor.get_yaw_angle(): # type: ignore 
            return math.ceil(targetAngle)
        
        if math.ceil(targetAngle) == targetAngle:
            print("WARNING: target angle is not accurate: " + str(targetAngle))
        if targetAngle > 180:
            ceilTargetAngle = math.ceil(targetAngle  / 180.0)
        else:
            ceilTargetAngle = math.ceil(targetAngle)
            
        if turnDirection == 'left':
            drivePair.start_tank(10, -10) # type: ignore
        else:
            drivePair.start_tank(-10, 10) # type: ignore
        
        wait_until(hub.motion_sensor.get_yaw_angle, equal_to, ceilTargetAngle) # type: ignore
        
        drivePair.stop # type: ignore

        return ceilTargetAngle

        
class FieldDriver():
    def __init__(self, startPositionX: int, startPositionY: int) -> None:
        self.START_POSITION_X: int = startPositionX
        self.START_POSITION_Y: int = startPositionY
        self.currentPositionX: float = startPositionX
        self.currentPositionY: float = startPositionY
        
        self.FieldSizeX: int = 200
        self.FieldSizeY: int = 114
        self.obstacles: list[tuple[float, float, float, float, str]] = [(0, 0, self.FieldSizeX, 0, "S"),
                          (0, 0, 0, self.FieldSizeY, "W"),
                          (self.FieldSizeX, 0, self.FieldSizeX, self.FieldSizeY, "E"),
                          (0, self.FieldSizeY, self.FieldSizeX, self.FieldSizeY, "N")]
        self.orientationLines: list[tuple[float, float, float, float]] = []
        self.astarStraightLineWeight = 0.5
    
    def GetDistanceToWall(self, targetWall: str) -> float:
        if targetWall == "north":
            return self.currentPositionY
        elif targetWall == "south":
            return abs(self.currentPositionY - self.FieldSizeX)
        elif targetWall == "east":
            return self.currentPositionX
        elif targetWall == "west":
            return abs(self.currentPositionX - self.FieldSizeY)
        else:
            raise Exception("UnknownTargetWall")
        
    def GetDistanceToPoint(self, startPositionX: float, startPositionY: float, targetPositionX: float, targetPositionY: float) -> float:
        return math.sqrt(math.pow(startPositionX - targetPositionX, 2) + math.pow(startPositionY - targetPositionY, 2))

    def BewegeZuPosition(self, targetPositionX: float, targetPositionY: float) -> None:
        diffX: float = targetPositionX - self.currentPositionX
        diffY: float = targetPositionY - self.currentPositionY
        collision: bool = False
        if diffX == 0.0:
            if diffY > 0.0:
                targetAngle: float = 0
            elif diffY < 0.0:
                targetAngle: float = 180
            else:
                raise Exception("PositionAngleNotMovable")
        else:
            targetAngle: float = math.degrees(math.atan(diffY / diffX))
            
        distanceToDestination: float = self.GetDistanceToPoint(targetPositionX, targetPositionY, self.currentPositionX, self.currentPositionY)
        driver: Driver = Driver()

        for obstacle in self.obstacles:
            if not collision:
                collision = self.CheckForIntersection(self.currentPositionX, self.currentPositionY, targetPositionX, targetPositionY, obstacle[0], obstacle[1], obstacle[2], obstacle[3])
            else: 
                break

        if collision:
            # raise Exception("Collision detected")
            pass
        else:
            pass
        for orientationLine in self.orientationLines:
            orientationLineIntersection = self.GetIntersection(self.currentPositionX, self.currentPositionY, targetPositionX, targetPositionY, orientationLine[0], orientationLine[1], orientationLine[2], orientationLine[3])
            if orientationLineIntersection != None:
                self.BewegeZuPosition(orientationLineIntersection[0], orientationLineIntersection[1])
                break
            
        if targetAngle >= targetAngle + 5 and targetAngle <= targetAngle - 5:
            drivePair.move(distanceToDestination) # type: ignore
        else:
            driver.TurnStationary(targetAngle, 'left')
            drivePair.move(distanceToDestination) # type: ignore
        del driver
        self.currentPositionX = targetPositionX
        self.currentPositionY = targetPositionY
    
    def CalibrateDistanceToWall(self):
        closestWall: str = self.GetClosestWall()
        distanceSensorTurnTolerance: int = 5
        driver: Driver = Driver()
        if closestWall == "north":
            driver.TurnStationary(0 + driver.DistanceSensorTurnOffset - distanceSensorTurnTolerance)
        elif closestWall == "south":
            driver.TurnStationary(90 + driver.DistanceSensorTurnOffset - distanceSensorTurnTolerance)
        elif closestWall == "west":
            driver.TurnStationary(180 + driver.DistanceSensorTurnOffset - distanceSensorTurnTolerance)
        else:
            driver.TurnStationary(270 + driver.DistanceSensorTurnOffset - distanceSensorTurnTolerance)
        
        #turn until the distance sensor sees targetWall 
        drivePair.start_tank(5, -5) # type: ignore
        distanceSensor.wait_for_distance_closer_than(closestWall, 'cm') # type: ignore
        drivePair.stop() # type: ignore
        #reset gyro
    
    def GetClosestWall(self) -> str:
        wallDistances: list[tuple[float, str]] = [(self.GetDistanceToWall("north"), "north"),
                            (self.GetDistanceToWall("east"), "east"),
                            (self.GetDistanceToWall("south"), "south"),
                            (self.GetDistanceToWall("west"), "west")]
        if not DEPLOYMENT:
            print("wallDistances:", wallDistances)
        closestWall = ""
        closestWallDistance = 0
        for i in range(len(wallDistances)):
            if wallDistances[i][0] < closestWallDistance:
                closestWallDistance = wallDistances[i][0]
                closestWall = wallDistances[i][1]
        return closestWall
        
    def RegisterObstacle(self, pointA_X: float, pointA_Y: float, pointB_X: float, pointB_Y: float, name: str):
        self.obstacles.append((pointA_X, pointA_Y, pointB_X, pointB_Y, name))
    
    def RegisterOrientationLine(self, pointA_X: float, pointA_Y: float, pointB_X: float, pointB_Y: float):
        self.orientationLines.append((pointA_X, pointA_Y, pointB_X, pointB_Y))

    def RegisterParallelRectangle(self, pointA_X: float, pointA_Y: float, pointB_X: float, pointB_Y: float, name: str):
        self.obstacles.append((pointA_X, pointA_Y, pointA_X, pointB_Y, name))
        self.obstacles.append((pointA_X, pointB_Y, pointB_X, pointB_Y, name))
        self.obstacles.append((pointB_X, pointB_Y, pointB_X, pointA_Y, name))
        self.obstacles.append((pointA_X, pointA_Y, pointB_X, pointA_Y, name))
        self.FieldSizeX
        return None
    
    def RegisterRectangle(self, pointA_X: float, pointA_Y: float, pointB_X: float, pointB_Y: float, pointC_X: float, pointC_Y: float, name: str) -> None:
        self.obstacles.append((pointA_X, pointA_Y, pointC_X, pointC_Y, name))
        self.obstacles.append((pointB_X, pointB_Y, pointC_X, pointC_Y, name))
        diffX: float = pointA_X - pointB_X
        diffY: float = pointA_Y - pointB_Y
        self.obstacles.append((pointB_X, pointB_Y, pointB_X + diffX, pointB_Y + diffY, name))
        diffX = pointA_X - pointC_X
        diffY = pointA_Y - pointC_Y
        self.obstacles.append((pointA_X, pointA_Y, pointA_X + diffX, pointA_Y + diffY, name))
        return None
    
    def GetSurroundingFields(self, currentPositionX: float, currentPositionY: float) -> list[tuple[float, float]]:
        return [
            (currentPositionX + 1, currentPositionY + 1),
            (currentPositionX + 1, currentPositionY),
            (currentPositionX + 1, currentPositionY - 1),
            (currentPositionX, currentPositionY - 1),
            (currentPositionX - 1, currentPositionY - 1),
            (currentPositionX - 1, currentPositionY),
            (currentPositionX - 1, currentPositionY + 1),
            (currentPositionX, currentPositionY + 1)]
        
    def GetIntersection(self, currentPositionX: float, currentPositionY: float, targetPositionX: float, targetPositionY: float, collisionLineStartX: float, collisionLineStartY: float, collisionLineEndX: float, collisionLineEndY: float) -> tuple[float, float] | None:
        LambdaDividend: float = collisionLineStartY * currentPositionX - collisionLineStartX * currentPositionY - collisionLineStartY * targetPositionX + currentPositionY * targetPositionX + collisionLineStartX * targetPositionY - currentPositionX * targetPositionY
        LambdaDivisor: float = collisionLineStartY * currentPositionX - collisionLineEndY * currentPositionX - collisionLineStartX * currentPositionY + collisionLineEndX * currentPositionY - collisionLineStartY * targetPositionX + collisionLineEndY * targetPositionX + collisionLineStartX * targetPositionY - collisionLineEndX * targetPositionY
        try:
            lambdaResult: float = LambdaDividend / LambdaDivisor
        except ZeroDivisionError:
            return None
        if lambdaResult == 0:
            return None
        vectorX: float = currentPositionX + lambdaResult * (targetPositionX - currentPositionX)
        vectorY: float = currentPositionY + lambdaResult * (targetPositionY - currentPositionY)
        VectorIsBetween: bool = (min(currentPositionX, targetPositionX) <= vectorX <= max(currentPositionX, targetPositionX)) and (min(currentPositionY, targetPositionY) <= vectorY <= max(currentPositionY, targetPositionY))
        if VectorIsBetween:
            return (vectorX, vectorY)
        else:
            return None
    
    def CheckForIntersection(self, currentPositionX: float, currentPositionY: float, targetPositionX: float, targetPositionY: float, collisionLineStartX: float, collisionLineEndX: float, collisionLineStartY: float, collisionLineEndY: float) -> bool:
        if self.GetIntersection(currentPositionX, currentPositionY, targetPositionX, targetPositionY, collisionLineStartX, collisionLineStartY, collisionLineEndX, collisionLineEndY) != None:
            return True
        else:
            return False
    
    def CheckPathCollision(self, targetPositionX: float, targetPositionY: float) -> bool:
        collision: bool = False
        for obstacle in self.obstacles:
            if not collision:
                if self.GetIntersection(self.currentPositionX, self.currentPositionY, targetPositionX, targetPositionY, obstacle[0], obstacle[1], obstacle[2], obstacle[3]) != None:
                    collision = True
            else:
                break
        return collision

    
    def GetAStarPath(self, targetPositionX: float, targetPositionY: float) -> list[tuple[float, float]]:
        collision: bool = False
        for obstacle in self.obstacles:
            if not collision:
                collision = self.CheckForIntersection(self.currentPositionX, self.currentPositionY, targetPositionX, targetPositionY, obstacle[0], obstacle[1], obstacle[2], obstacle[3])
            else:
                break
        if not collision:
            return [(targetPositionX, targetPositionY)]
        
        del obstacle
        MoveChain: list[tuple[float, float]] = []
        prevMoveChainElement: list[float] = [self.currentPositionX, self.currentPositionY]
        onDestination: bool = False
        bestAStar: float = 2147483647
        
        while not onDestination:
            moveDirectionX: float = 0
            moveDirectionY: float = 0
            # print(self.GetSurroundingFields(prevMoveChainElement[0], prevMoveChainElement[1]))
            for x, y in self.GetSurroundingFields(prevMoveChainElement[0], prevMoveChainElement[1]):
                collision = False
                for obstacle in self.obstacles:
                    if not collision:
                        collision = self.CheckForIntersection(prevMoveChainElement[0], prevMoveChainElement[1], x, y, obstacle[0], obstacle[1], obstacle[2], obstacle[3])
                    else:
                        break
                if collision:
                    continue
                aStarWeighting = math.sqrt(math.pow(x - targetPositionX, 2) + math.pow(y - targetPositionY, 2)) + 1
                if aStarWeighting < bestAStar:
                    bestAStar = aStarWeighting
                    moveDirectionX = x
                    moveDirectionY = y
                if x == targetPositionX and y == targetPositionY:
                    if prevMoveChainElement[0] == x and prevMoveChainElement[1] == y:
                        MoveChain[len(MoveChain) - 1] = (x, y)
                    else:
                        MoveChain.append((x, y))
                    onDestination = True
                    break
            else:
                MoveChain.append((moveDirectionX, moveDirectionY))
                prevMoveChainElement = [moveDirectionX, moveDirectionY]
        return MoveChain

driver: Driver = Driver()
f: FieldDriver = FieldDriver(15, 14)
# fieldDriver = FieldDriver(2, 2)
# print(fieldDriver.GetDistanceToPoint(15, 30, fieldDriver.currentPositionX, fieldDriver.currentPositionY))

f.BewegeZuPosition(15.0, 30.0)


if DEPLOYMENT:
    hub.status_light.on('cyan') # type: ignore
else:
    print("finished")
    