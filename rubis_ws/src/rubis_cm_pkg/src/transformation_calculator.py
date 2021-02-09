import numpy as np

GPS_points = [[4161757.50275, 312899.490661, 36.3045611059, 1], [4161339.53763, 313544.673745, 37.4391644837, 1],
              [4161146.49031, 314016.720516, 35.2621279006, 1], [4161167.17155, 314177.606897, 38.9439076488, 1]]

NDT_points = [[1918.83837891,  -1100.59619141, 0.705016136169, 1], [1879.30480957, -332.478485107, -8.46676731119, 1],
              [1757.7755127, 162.494659424, -9.41207313538, 1], [1645.28942871, 279.143066406, -5.62943172455, 1]]

# G*R = N
G = np.array(GPS_points)
N_X = np.array([NDT_points[0][0], NDT_points[1][0], NDT_points[2][0], NDT_points[3][0]])
N_Y = np.array([NDT_points[0][1], NDT_points[1][1], NDT_points[2][1], NDT_points[3][1]])
N_Z = np.array([NDT_points[0][2], NDT_points[1][2], NDT_points[2][2], NDT_points[3][2]])
N_1 = np.array([NDT_points[0][3], NDT_points[1][3], NDT_points[2][3], NDT_points[3][3]])


ans1 = np.linalg.solve(G, N_X)
ans2 = np.linalg.solve(G, N_Y)
ans3 = np.linalg.solve(G, N_Z)
ans4 = np.linalg.solve(G, N_1)


print(format(ans1[0],'f'), format(ans1[1],'f'), format(ans1[2],'f'), format(ans1[3],'f'))
print(format(ans2[0],'f'), format(ans2[1],'f'), format(ans2[2],'f'), format(ans2[3],'f'))
print(format(ans3[0],'f'), format(ans3[1],'f'), format(ans3[2],'f'), format(ans3[3],'f'))
print(format(ans4[0],'f'), format(ans4[1],'f'), format(ans4[2],'f'), format(ans4[3],'f'))




T = [ans1, ans2, ans3, ans4]

X = [4161460.0317, 313270.34409, 38.1143565199, 1]
# X = [4161757.50275, 312899.490661, 36.3045611059, 1]
print("=============================")

A = np.matmul(T, X)
print(format(A[0],'f'), format(A[1],'f'), format(A[2],'f'), format(A[3],'f'))





