class RobotInterface:
    def __init__(self):
        self.forward()
        self.backward()
        self.orbit()
        self.spinLeft()


    def forward(self):
        raise NotImplementedError("Method forward not implemented")

    
    def backward(self):
        raise NotImplementedError("Method backward not implemented")


    def orbit(self):
        raise NotImplementedError("Method orbit not implemented")

    
    def spinLeft(self):
        raise NotImplementedError("Method spinLeft not implemented")


    def spinRight(self):
        raise NotImplementedError("Method spinRight not implemented")

    
    def left(self):
        raise NotImplementedError("Method left not implemented")


    def right(self):
        raise NotImplementedError("Method right not implemented")


    def stop(self):
        raise NotImplementedError("Method stop not implemented")
