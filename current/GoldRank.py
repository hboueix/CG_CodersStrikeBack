import sys
import math


def getStartAngle(x, y, next_checkpoint_x, next_checkpoint_y):
    if next_checkpoint_y < y and next_checkpoint_x < x:
        tangente = abs(next_checkpoint_y - y) / abs(next_checkpoint_x - x)
        angle = 180
    elif next_checkpoint_y > y and next_checkpoint_x < x:
        tangente = abs(next_checkpoint_x - x) / abs(next_checkpoint_y - y)
        angle = 90
    elif next_checkpoint_y > y and next_checkpoint_x > x:
        tangente = abs(next_checkpoint_y - y) / abs(next_checkpoint_x - x)
        angle = 0
    elif next_checkpoint_y < y and next_checkpoint_x > x:
        tangente = abs(next_checkpoint_x - x) / abs(next_checkpoint_y - y)
        angle = 270
    elif next_checkpoint_x == x:
        return 270 if next_checkpoint_y > y else 90
    elif next_checkpoint_y == y:
        return 180 if next_checkpoint_x < x else 0
    angle += abs(math.atan(tangente) * 180 / math.pi)
    return round(angle)


def play(pods, checkpoints):
    # This tracks the time during the turn. The goal is to reach 1.0
    t = 0
    while t < 1:
        firstCollision = None

        # We look for all the collisions that are going to occur during the turn
        for i in range(len(pods)):
            # Collision with another pod?
            for j in range(i + 1, len(pods)):
                col = pods[i].collision(pods[j])

                # If the collision occurs earlier than the one we currently have we keep it
                if col != None and col.t + t < 1 and (firstCollision == None or col.t < firstCollision.t):
                    firstCollision = col

            # Collision with another checkpoint?
            col = pods[i].collision(checkpoints[pods[i].nextCheckpointId])
            # If the collision occurs earlier than the one we currently have we keep it
            if col != None and col.t + t < 1 and (firstCollision == None or col.t < firstCollision.t):
                firstCollision = col

        if firstCollision == None:
            # No collision, we can move the pods until the end of the turn
            for i in range(len(pods)):
                pods[i].move(1 - t)
                # End of turn
            t = 1
        else:
            # Move the pods to reach the time `t` of the collision
            for i in range(len(pods)):
                pods[i].move(firstCollision.t)
            # Play out the collision
            firstCollision.unitA.bounce(firstCollision.unitB)
            for i in range(len(pods)):
                pods[i].move(1 - t)
                # End of turn
            t = 1
            #t += firstCollision.t

    for i in range(len(pods)):
        pods[i].end()


def test(pods, checkpoints, thrust):
    for i in range(len(pods)):
        pods[i].rotate(checkpoints[-1])
        pods[i].boost(thrust)  # BOOST=650
    play(pods, checkpoints)


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance2(self, point):
        return (self.x - point.x)**2 + (self.y - point.y)**2

    def distance(self, point):
        return math.sqrt(self.distance2(point))

    def closest(self, a, b):
        da = b.y - a.y
        db = a.x - b.x
        c1 = da * a.x + db * a.y
        c2 = -db * self.x + da * self.y
        det = da ** 2 + db ** 2
        cx, cy = 0, 0
        if det != 0:
            cx = (da * c1 - db * c2) / det
            cy = (da * c2 - db * c1) / det
        else:
            cx = self.x
            cy = self.y
        return Point(cx, cy)


class Unit(Point):

    nb_units = 0

    def __init__(self, x, y, id, r, vx, vy):
        Point.__init__(self, x, y)
        self.id = Unit.nb_units if id == None else Unit.nb_units + id
        Unit.nb_units += 1
        self.r = r
        self.vx = vx
        self.vy = vy

    def collision(self, unit):
        dist = self.distance2(unit)  # square of distance
        sum_radius = (self.r + unit.r) ** 2  # squared
        if dist < sum_radius:
            return Collision(self, unit, 0)
        if self.vx == unit.vx and self.vy == unit.vy:
            return
        # Reference frame -> unit
        x, y = self.x - unit.x, self.y - unit.y
        vx, vy = self.vx - unit.vx, self.vy - unit.vy
        myPoint = Point(x, y)
        unitPoint = Point(0, 0)
        # Closest point to unit on our speed vector line
        closest = unitPoint.closest(myPoint, Point(x + vx, y + vy))
        unitDist = unit.distance2(closest)
        myDist = myPoint.distance2(closest)
        if unitDist < sum_radius:
            length = math.sqrt(vx ** 2 + vy ** 2)
            backdist = math.sqrt(sum_radius - unitDist)
            closest.x -= backdist * (vx / length)
            closest.y -= backdist * (vy / length)
            if myPoint.distance2(closest) > myDist:
                return
            unitDist = closest.distance(myPoint)
            if unitDist > length:
                return
            t = unitDist / length
            return Collision(self, unit, t)
        return


class Pod(Unit):
    def __init__(self, x, y, angle):
        Unit.__init__(self, x, y, 100, 400, 0, 0)
        self.angle = angle
        self.nextCheckpointId = 0
        self.checked = 0
        self.timeout = 100
        self.partner = None
        self.shield = False

    def getAngle(self, point):
        dist = self.distance(point)
        dx = (point.x - self.x) / dist
        dy = (point.y - self.y) / dist
        angle = math.acos(dx) * 180 / math.pi
        if dy < 0:
            angle = 360 - angle
        return round(angle)

    def diffAngle(self, point):
        angle = self.getAngle(point)
        right = angle - self.angle if self.angle <= angle else 360 - self.angle + angle
        left = self.angle - angle if self.angle >= angle else self.angle + 360 - angle
        if right < left:
            return right
        else:
            return -left

    def rotate(self, point):
        angle = self.diffAngle(point)
        if angle > 18:
            angle = 18
        elif angle < -18:
            angle = -18
        self.angle += angle
        if self.angle >= 360:
            self.angle -= 360
        elif self.angle < 0:
            self.angle += 360

    def boost(self, thrust):
        if self.shield:
            return
        rad = self.angle * math.pi / 180
        self.vx += math.cos(rad) * thrust
        self.vy += math.sin(rad) * thrust

    def move(self, t):
        self.x += self.vx * t
        self.y += self.vy * t

    def end(self):
        self.x = round(self.x)
        self.y = round(self.y)
        self.vx = round(self.vx * 0.85)
        self.vy = round(self.vy * 0.85)
        self.timeout -= 1

    def play(self, point, thrust):
        self.rotate(point)
        self.boost(thrust)
        self.move(1.0)
        self.end()

    def bounce(self, unit):
        print('COLLISION !', file=sys.stderr)
        if isinstance(unit, Checkpoint):
            # Collision with a checkpoint
            self.checked += 1
            self.timeout = 100
        else:
            # If a pod has its shield active its mass is 10 otherwise it's 1
            m1 = 10 if self.shield else 1
            m2 = 10 if unit.shield else 1
            m_coeff = (m1 + m2) / (m1 * m2)

            nx, ny = self.x - unit.x, self.y - unit.y
            nxny_square = nx ** 2 + ny ** 2

            dvx, dvy = self.vx - unit.vx, self.vy - unit.vy

            # fx and fy are the components of the impact vector. product is just there for optimisation purposes
            product = nx * dvx + ny * dvy
            fx = (nx * product) / (nxny_square * m_coeff)
            fy = (ny * product) / (nxny_square * m_coeff)

            # Apply the impact vector once
            self.vx -= fx / m1
            self.vy -= fy / m1
            unit.vx += fx / m2
            unit.vy += fy / m2

            # If the norm of the impact vector is less than 120, we normalize it to 120
            impulse = math.sqrt(fx ** 2 + fy ** 2)
            if impulse < 120:
                fx = fx * 120 / impulse
                fy = fy * 120 / impulse

            # Apply the impact vector a second time
            self.vx -= fx / m1
            self.vy -= fy / m1
            unit.vx += fx / m2
            unit.vy += fy / m2


class Collision:
    def __init__(self, unitA, unitB, t):
        self.unitA = unitA
        self.unitB = unitB
        self.t = t


class Checkpoint(Unit):

    all = []

    def __init__(self, x, y):
        Unit.__init__(self, x, y, 0, 600, 0, 0)
        Checkpoint.all.append(self)

    def bounce(self, unit):
        print(f'bounce {unit}')


class Move:
    def __init__(self, angle, thrust):
        self.angle = angle # Between -18.0 and +18.0
        self.thrust = thrust # Between -1 and 100

    def mutate(self, amplitude):
        print(f'mutate w/ amplitude : {amplitude}')


class Solution:
    def randomize(self):
        print('randomize')

laps = int(input())
checkpoint_count = int(input())
checkpoints = []
for i in range(checkpoint_count):
    checkpoint_x, checkpoint_y = [int(j) for j in input().split()]
    checkpoints.append((checkpoint_x, checkpoint_y))
print(checkpoints, file=sys.stderr)

# game loop
while True:
    for i in range(2):
        # x: x position of your pod
        # y: y position of your pod
        # vx: x speed of your pod
        # vy: y speed of your pod
        # angle: angle of your pod
        # next_check_point_id: next check point id of your pod
        x, y, vx, vy, angle, next_check_point_id = [int(j) for j in input().split()]
    for i in range(2):
        # x_2: x position of the opponent's pod
        # y_2: y position of the opponent's pod
        # vx_2: x speed of the opponent's pod
        # vy_2: y speed of the opponent's pod
        # angle_2: angle of the opponent's pod
        # next_check_point_id_2: next check point id of the opponent's pod
        x_2, y_2, vx_2, vy_2, angle_2, next_check_point_id_2 = [int(j) for j in input().split()]

    # Write an action using print
    # To debug: print("Debug messages...", file=sys.stderr)


    # You have to output the target position
    # followed by the power (0 <= thrust <= 100)
    # i.e.: "x y thrust"
    print("8000 4500 100")
    print("8000 4500 100")
