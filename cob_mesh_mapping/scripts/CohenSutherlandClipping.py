

class Clipper:

    def __init__(self, xlim = [-1, 1], ylim = [-1, 1], zlim = [-1, 1]):
        self.F = 0b100000
        self.N = 0b010000
        self.T = 0b001000
        self.B = 0b000100
        self.R = 0b000010
        self.L = 0b000001
        self.lim = [xlim, ylim, zlim]

    def compute_code(self, x, y, z=0):
        c = 0
        if x < self.lim[0][0]:
            c |= self.L
        elif x > self.lim[0][1]:
            c |= self.R
        if y < self.lim[1][0]:
            c |= self.T
        elif y > self.lim[1][1]:
            c |= self.B
        if z < self.lim[2][0]:
            c |= self.N
        elif z > self.lim[2][1]:
            c |= self.F

        return c


c = Clipper()
print c.compute_code(2,2)
