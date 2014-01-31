

def area(L, H):
    A = 0.0
    cx = 0.0
    cy = 0.0
    l = len(L)
    for i in range(l):
        vi = L[i]
        vii = L[(i+1)%l]
        ti =  vi[0] * vii[1] - vii[0] * vi[1]
        A += ti
        cx += (vi[0] + vii[0]) * ti
        cy += (vi[1] + vii[1]) * ti

    A = (0.5 * A)
    cx /= (6.0 * A)
    cy /= (6.0 * A)

    #weighted holes:
    A_diff = A
    cx_diff = 0.0
    cy_diff = 0.0
    if (len(H) > 0) :
        for i in range(len(H)):
            cx_diff += H[i][1] * abs(H[i][0])
            cy_diff += H[i][2] * abs(H[i][0])
            A_diff -= abs(H[i][0])

    return [A_diff, (cx * A - cx_diff) / A_diff, (cy * A - cy_diff) / A_diff]

L = [[0.0, 0.0],
     [8.0, 0.0],
     [8.0, 6.0],
     [0.0, 6.0]]

L2 = [[5.0, 3.0],
      [5.0, 6.0],
      [7.0, 6.0],
      [7.0, 3.0]]

L3 = [[1.0, 3.0],
      [1.0, 6.0],
      [3.0, 6.0],
      [3.0, 3.0]]

H = []
E = []
H.append(area(L2, E))
H.append(area(L3, E))
H.append(area(L, H))
print 'A=', H[0][0], ' P=(',H[0][1],'|',H[0][2],')'
print 'A=', H[1][0], ' P=(',H[1][1],'|',H[1][2],')'
print 'A=', H[2][0], ' P=(',H[2][1],'|',H[2][2],')'
