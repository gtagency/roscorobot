import math

class DeadReckoningLocalizer:
    def __init__(wheelRadius, wheelSeparation, speedAdjustment):
        self.wheelRadius = wheelRadius
        self.wheelSeparation = wheelSeparation
        self.speedAdjustment = speedAdjustment

        self.posX = 0
        self.posY = 0
        self.posTheta = 0

    def update(leftSpeed, rightSpeed, time):
        v = self.speedAdjustment * self.wheelRadius * (leftSpeed + rightSpeed) / 2
        w = self.speedAdjustment * self.wheelRadius * (rightSpeed - leftSpeed) / self.wheelSeparation
        # Robot frame deltas
        xR = 0
        yR = 0
        if w:
            xR = v * math.sin(w * time) / w
            yR = v * (1 - math.cos(w * time)) / w
        else:
            xR = v * time
        thetaR = w * time
        # World frame coordinates
        t = self.posTheta
        newX = math.cos(t) * xR - math.sin(t) * xY + self.posX
        newY = math.sin(t) * xR + math.cos(t) * xY + self.posY
        newTheta = (t + thetaR) % (2 * math.pi)
        self.posX = newX
        self.posY = newY
        self.posTheta = newTheta
        return self.getPose()

    def getPose():
        return (self.posX, self.posY, self.posTheta)