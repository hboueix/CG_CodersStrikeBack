import sys
import math

turn = 0
x = 0
y = 0

# game loop
while True:
    
    prevX = x
    prevY = y

    x, y, next_checkpoint_x, next_checkpoint_y, next_checkpoint_dist, next_checkpoint_angle = [int(i) for i in input().split()]
    opponent_x, opponent_y = [int(i) for i in input().split()]
    
    if prevX == 0:
        prevX = x
        prevY = y

    # Adapt speed to checkpoint distance
    if next_checkpoint_dist < 1500: 
        thrust = 30
    if next_checkpoint_dist < 650: 
        thrust = 19
    else:
        thrust = 100
    
    # Slow if need to turn a lot
    if abs(next_checkpoint_angle) > 90:
        thrust = 0
    
    # Go to middle if next checkpoint is near border and if we'll certainly reach it
    distToMiddle = math.sqrt((next_checkpoint_x - 8000) ** 2 + (next_checkpoint_y - 4500) ** 2)
    if abs(next_checkpoint_angle) < 50 and next_checkpoint_dist < 2600 and distToMiddle > 3000:
        next_checkpoint_x = 8000
        next_checkpoint_y = 4500
        thrust = 30
    
    # Boost if far from checkpoint
    if next_checkpoint_dist > 7000 and next_checkpoint_angle == 0: 
        thrust = "BOOST"
    
    # Don't move first turn to avoid opponent collision
    if turn == 0:
        thrust = 0
    turn += 1

    # Adjust target with precedent move
    x_target = next_checkpoint_x - (x - prevX) * 3
    y_target = next_checkpoint_y - (y - prevY) * 3

    print(f'{x_target} {y_target} {thrust}')