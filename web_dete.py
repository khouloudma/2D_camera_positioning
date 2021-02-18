import cv2
import detector as t
import numpy as np
import math as m
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cir

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        out, x1, y1, x2, y2, x3, y3 = t.detect(gray)
        # calcul
        l = [x1, x2, x3]
        l.sort()
        centre = 640  # 1280 / 2
        foc = 802
        theta1 = m.atan((l[0] - centre) / foc)
        theta2 = m.atan((l[1] - centre) / foc)
        theta3 = m.atan((l[2] - centre) / foc)
        alpha = theta2 - theta1
        beta = theta3 - theta2
        print(theta1)
        print(theta2)
        print(theta3)

        p1 = np.array([[0], [-55]])
        p2 = np.array([[55], [0]])
        p3 = np.array([[110], [0]])

        lm1 = np.array([p1[0] + p2[0], p1[1] + p2[1]]) / 2
        lm2 = np.array([p2[0] + p3[0], p2[1] + p3[1]]) / 2
        print("lm1= ", lm1)
        print("lm12 ", lm2)

        # print("Point 1 : ",l[0],y1)#on suppose que les 3 cercles sont à la même hauteur
        # print("Point 2 : ",l[1],y1)# //
        # print("Point 3 : ",l[2],y1)# //

        print("Point milieu entre 1 et 2 = ", lm1)
        print("Point milieu entre 2 et 3 = ", lm2)

        b1 = np.array(p1 - p2)
        b2 = np.array(p2 - p3)
        print('b1 = ', b1)
        print('b2 = ', b2)
        # print('circle 1 coord : ',l[0],':',y1)
        # print('circle 2 coord : ',l[1],':',y2)

        mat = np.array([[0, -1], [1, 0]])
        produit1 = np.dot(mat, b1)
        produit2 = np.dot(mat, b2)
        tang1 = 1 / (2 * m.tan(alpha + 0.0000001))
        tang2 = 1 / (2 * m.tan(beta + 0.0000001))

        c1 = lm1 + tang1 * produit1

        r1 = abs(78 / (2 * m.sin(alpha + 0.00001)))
        print("rayon 1 =", r1, " cm")
        c2 = lm2 + tang2 * produit2
        # print("centre 2 de coordonnées :",c2)
        r2 = abs(55 / (2 * m.sin(beta + 0.00001)))
        print("rayon 2 =", r2, " cm")

        intersections = cir.get_intersections(c1[0], c1[1], r1, c2[0], c2[1], r2)
        (a, b, c, d) = intersections
        # print(intersections)
        centree = "Position du robot : (" + str(int(a)) + "," + str(int(b)) + ")"

        # circle1 = (int(c1[0][0]),int(c1[1][0]),int(r1))
        # circle2 = (int(c2[0][0]),int(c2[1][0]),int(r2))

        # print('cercle 1 {x1,y1,r1} = ',circle1)
        # print('cercle 2 {x2,y2,r2} = ',circle2)
        # print('Position du cercle au mileu : ',l[1],y2)
        # (a, b, c, d) = cir.get_intersections(int(c1[0][0]),int(c1[1][0]),int(r1), int(c2[0][0]),int(c2[1][0]),int(r2))
        # print(a, b, c, d)
        # print ("intersection 1 =",a,b)
        # print ("intersection 2 =",c,d)
        if (l[2] - l[1] != 0):
            dr = (35 * foc) / (l[2] - l[1])
            # print (dr)
        cv2.putText(out, centree, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)

        # plot

        # fig, ax = plt.subplots()
        # ax.set_xlim((-1000, 1000))
        # ax.set_ylim((-1000, 1000))

        if intersections is not None:
            i_x3, i_y3, i_x4, i_y4 = intersections
            plt.plot([i_x3, i_x4], [i_y3, i_y4], '.', color='y')

        # circle1 = plt.Circle((c1[0], c1[1]), r1, color='r')
        # circle2 = plt.Circle((c2[0], c2[1]), r2, color='g')
        # circle3 = plt.Circle((0, -40), 10, color='black')
        # circle4 = plt.Circle((30, 0), 10, color='black')
        # circle5 = plt.Circle((60, 0), 10, color='black')
        # ax.add_patch(circle1)
        # ax.add_patch(circle2)
        # ax.add_patch(circle3)
        # ax.add_patch(circle4)
        # ax.add_patch(circle5)
        # plt.gca().set_aspect('equal', adjustable='box')
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(0, -40, 5, c='b')
        # ax.scatter(30, 0, 5, c='b')
        # ax.scatter(60, 0, 5, c='b')
        # ax.scatter(a, b, 5, c='r')
        # print("x1 = ",l[1])
        # print("x2 = ",l[2])
        distance = (50 * foc) / (l[2] - l[1] + 0.0001)
        dist = "Distance =" + str(round(distance))
        cv2.putText(out, "alpha = " + str(alpha), (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(out, "beta = " + str(beta), (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)
        rayon1 = "Rayon 1 : " + str(int(r1))
        rayon2 = "Rayon 2 : " + str(int(r2))
        cerclem = "Cercle au mileu : " + str(l[1])
        cv2.putText(out, rayon1, (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(out, rayon2, (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(out, cerclem, (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 1, cv2.LINE_AA)

        # cv2.putText(out, dist, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1, cv2.LINE_AA)
        cv2.imshow('Mr.Robot_Position', out)

        print("position cercle a gauche : ", l[0])
        print("position cercle au milieu : ", l[1])
        print("position cercle a droite : ", l[2])

        # print("distance = ",l[2]-l[1])
        # plt.show()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break
cap.release()
cv2.destroyAllWindows()
