class Motor:
    def __init__(self):
        self.angle = 0.0    # degrees
        self.speed = 0.0    # degrees per second

    def set_speed(self, speed):
        self.speed = speed

    def update(self, dt):
        self.angle += self.speed * dt

        # clamp like a servo
        if self.angle > 180:
            self.angle = 180
        elif self.angle < 0:
            self.angle = 0
