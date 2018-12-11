class VehicleState:

    def __init__(self, x, y, theta1, theta2):
        self.x = x
        self.y = y
        self.theta1 = theta1
        self.theta2 = theta2
    def getPoint(self):
        return (self.x, self.y)
    def getAttributes(self):
        return (self.x, self.y ,self.theta1, self.theta2)
    def copy(self):
        return VehicleState(*self.getAttributes())
