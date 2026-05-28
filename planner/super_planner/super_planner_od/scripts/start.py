import math
import turtle

RADIUS = 10

angleSin18 = math.sin(math.pi * 0.1) * RADIUS
angleCos18 = math.cos(math.pi * 0.1) * RADIUS
angleSin54 = math.sin(math.pi * 0.3) * RADIUS
angleCos54 = math.cos(math.pi * 0.3) * RADIUS




# turtle.width(10)
#
# turtle.color("red")
# turtle.penup()
# turtle.goto(-angleCos18, angleSin18)
# turtle.pendown()
#
# turtle.goto(angleCos18, angleSin18)
#
# turtle.goto(-angleCos54, -angleSin54)
#
# turtle.goto(0, RADIUS)
#
# turtle.goto(angleCos54, -angleSin54)
#
# turtle.goto(-angleCos18, angleSin18)

z = 1.5
print(angleCos18, angleSin18, z)
print(-angleCos54, -angleSin54, z)
print(0, RADIUS, z)
print(angleCos54, -angleSin54, z)
print(-angleCos18, angleSin18, z)