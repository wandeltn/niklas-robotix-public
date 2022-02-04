DEPLOYMENT = False

import math

if DEPLOYMENT:
    from mindstorms import MSHub, Motor, MotorPair, ColorSensor, DistanceSensor
    from mindstorms.control import wait_until
    from mindstorms.operator import equal_to

if DEPLOYMENT:
    #set ports
    
    hub = MSHub()
    hub.status_light.on('orange')
    verticalMotor = Motor('F')
    clawMotor = Motor('E')
    # colorSensorRight = ColorSensor('D')
    colorSensorLeft = ColorSensor('D')
    distanceSensor = DistanceSensor('C')
    tempDriveMotorLeft = Motor('A')
    tempDriveMotorRight = Motor('B')
    
    tempDriveMotorRight.set_degrees_counted(0)
    tempDriveMotorLeft.set_degrees_counted(0)
    del tempDriveMotorLeft
    del tempDriveMotorRight
    
    drivePair = MotorPair('B', 'A')
    #set default
    
    colorSensorLeft.light_up_all(100)
    # colorSensorRight.light_up_all(100)
    distanceSensor.light_up_all(100)
    drivePair.set_stop_action('hold')
    drivePair.set_motor_rotation(5.6 * math.pi, 'cm')
    hub.motion_sensor.reset_yaw_angle()
    hub.status_light.on('green')
    
else:
    #set ports
    print("No DEPLOYMENT \n")
    print("verticalMotor: B\n")
    print("clawMotor: A\n")
    print("drivePair: F, E\n")
    print("colorSensor: D\n")
    print("distanceSensor: C\n")
    #set default
    print("colorSensor light up all\n")
    print("distanceSensor light up all\n")
    print("motor stop action: hold\n")
    print("motor rotaion distance:" + str(5.6 * math.pi))
    print("gyro reset to 0\n")


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
            print("WARNING: target angle is not accurate: " + str(targetAngle))
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

        
class FieldDriver():
    def __init__(self, startPositionX, startPositionY):
        self.START_POSITION_X = startPositionX
        self.START_POSITION_Y = startPositionY
        self.currentPositionX = startPositionX
        self.currentPositionY = startPositionY
        
        self.FIELD = []
        self.FieldSizeX = 200
        self.FieldSizeY = 114
        self.obstacles = [[0, 0, self.FieldSizeX, 0],
                          [0, 0, self.FieldSizeY, 0],
                          [self.FieldSizeY, 0, self.FieldSizeX, self.FieldSizeY],
                          [self.FieldSizeX, 0, self.FieldSizeY, self.FieldSizeX],
                          [4, 6, 6, 4]]
        self.astarStraightLineWeight = 0.5
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
    def MoveToPosition(self, positionX, positionY):
        diffX = positionX - self.currentPositionX
        diffY = positionY - self.currentPositionY
        if diffX == 0.0:
            diffX = math.inf
        targetAngle = math.degrees(math.atan(diffY / diffX))
        print(math.cos(targetAngle))
        if math.cos(targetAngle) == 0.0:
            distanceToDestination = diffX / math.cos(targetAngle)
        else:
            distanceToDestination = math.inf
        driver = Driver()
        if DEPLOYMENT:
            driver.TurnStationary(targetAngle, 'left')
            drivePair.move(distanceToDestination)

            self.currentPositionX = positionX
            self.currentPositionY = positionY
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
        LambdaDividend = collisionLineStartY * currentPositionX - collisionLineStartX * currentPositionY - collisionLineStartY * targetPositionX + currentPositionY * targetPositionX + collisionLineStartX * targetPositionY - currentPositionX * targetPositionY
        LambdaDivisor = collisionLineStartY * currentPositionX - collisionLineEndY * currentPositionX - collisionLineStartX * currentPositionY + collisionLineEndX * currentPositionY - collisionLineStartY * targetPositionX + collisionLineEndY * targetPositionX + collisionLineStartX * targetPositionY - collisionLineEndX * targetPositionY
        try:
            LambdaResult1 = LambdaDividend / LambdaDivisor
        except ZeroDivisionError:
            LambdaResult1 = math.inf
            
        VectorX = currentPositionX + LambdaResult1 * (targetPositionX - currentPositionX)
        VectorY = currentPositionY + LambdaResult1 * (targetPositionY - currentPositionY)
        
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
        try:
            distanceToDestination = math.sqrt(math.pow(targetPositionX - self.currentPositionX, 2) + math.pow(targetPositionY - self.currentPositionY, 2))
        except ZeroDivisionError:
            distanceToDestination = math.inf
        # Determine if astar is neccessary
        collision = False
        for i in self.obstacles:
            collision = self.CheckForCollisions(self.currentPositionX, self.currentPositionY, targetPositionX, targetPositionY, i[0], i[1], i[2], i[3])
            if not collision:
                return [targetPositionX, targetPositionY]
            else:
                break
        MoveChain = []
        # calculate astar path
        if collision:
            onDestination = False
            prevMoveChainElement = [self.currentPositionX , self.currentPositionY]
            while not onDestination:
                bestMoveDirectionX, bestMoveDirectionY, bestAstarValue = 0, 0, math.inf

                for x, y in self.GetSurroundingFields(prevMoveChainElement[0], prevMoveChainElement[1]):
                    invalidMoveOption = False
                    
                    for obstacle in self.obstacles:
                        
                        if self.CheckForCollisions(self.currentPositionX, self.currentPositionY, x, y, obstacle[0], obstacle[1], obstacle[2], obstacle[3]):
                        
                            invalidMoveOption = True
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
                if prevMoveChainElement[0] == targetPositionX and prevMoveChainElement[1] == targetPositionY:
                    onDestination = True
            
                
        return MoveChain
        
    

driver = Driver()

# fieldDriver = FieldDriver(15, 5)
fieldDriver = FieldDriver(1, 1)

fieldDriver.CheckForCollisions(0, 0, 20, 20, 0, 20, 20, 0)

print(fieldDriver.GetAStarPath(50, 50))

fieldDriver.MoveToPosition(15, 20)
#drivePair.move(20, 'cm', 0, 50)

if DEPLOYMENT:
    hub.status_light.on('cyan')
else:
    print("finished")