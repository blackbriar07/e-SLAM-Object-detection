import numpy as np
import math


d2r	=0.0174533
r2d=57.2958

x = 37.271555
y = -0.069634
angle = -0.67857

X1 = x + 10 * math.cos(angle)
Y1 = y + 10 * math.sin(angle)

print(X1)
print(Y1)

X2 = 400
Y2 = 0

vec1 = (X1 - x, Y1 - y)
vec2 = (X2 - x, Y2 - y)

Mod_vec1 = math.sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1])
Mod_vec2 = math.sqrt(vec2[0]*vec2[0] + vec2[1]*vec2[1])

Dot_product = vec1[0]*vec2[0] + vec1[1]*vec2[1]

angle = math.acos(Dot_product/(Mod_vec1*Mod_vec2))
angle = angle * r2d
determinant = vec1[0] * vec2[1] - vec1[1] * vec2[0]

print("angle ", angle)
print("determinant ",determinant)