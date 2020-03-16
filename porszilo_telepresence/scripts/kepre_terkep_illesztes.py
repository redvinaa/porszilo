# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from PIL import Image
import rospy


def kepreIllesztes(**kwargs):

    pos_y        = kwargs["pos_y"]
    pos_x        = kwargs["pos_x"]
    h            = kwargs["h"]
    view_angle_v = kwargs["view_angle_v"]
    view_angle_h = kwargs["view_angle_h"]
    deg          = kwargs["deg"]
    orig_map     = kwargs["orig_map"]
    cam          = kwargs["cam"]

    # az általam rajzolt térképen 1 pixel 1 cm a valóságban
    
    #  pos_y = 84  # pozíciónk a térképen pixelben
    #  pos_x = 128
    #  h = 25  # a kamera optikai tengelyének magassága a földtől pixel egységgé alakítva
    #  view_angle_v = 44  # kamera vertikális látószöge fokban
    #  view_angle_h = 76  # kamera horizontális látószöge fokban
    #  deg = 195  # hány fokkal kell elforgatnunk a térépet hogy a robot nézési iránya függőlegesen legyen
    
    #  orig_map = cv2.imread('map.png', 0)  # beolvassuk a térképet és a látott képet
    #  cam = cv2.imread('kep2.jpg')
    
    
    cam_gray = cv2.cvtColor(cam, cv2.COLOR_BGR2GRAY)
    
    # cv2.imshow('map', map)
    # cv2.waitKey()
    
    map_cols = orig_map.shape[1]
    map_rows = orig_map.shape[0]
    
    cam_cols = cam.shape[1]
    cam_rows = cam.shape[0]
    
    map_diag = int(round(1.2 * (math.sqrt(math.pow(map_cols, 2) + math.pow(map_rows, 2)))))  # egy nagy térkép amibe beletéve a kicsit szabadon el tudjuk forgatni
    big_map = np.zeros((map_diag, map_diag), 'uint8')
    
    shift_cols = int(round((map_diag - map_cols) / 2))  # beletesszük a közepére a térképet
    shift_rows = int(round((map_diag - map_rows) / 2))
    big_map[shift_rows:(map_rows + shift_rows), shift_cols:(map_cols + shift_cols)] = orig_map
    
    pos_y_big = shift_rows + pos_y  # átszámítva a koordinátkat a nagy térképre
    pos_x_big = shift_cols + pos_x
    
    # big_map[pos_y_big,pos_x_big]=0
    # cv2.imshow('big_map', big_map)
    # cv2.waitKey()
    
    
    M_rot = cv2.getRotationMatrix2D((map_diag / 2, map_diag / 2), deg, 1)  # elforgatjuk hogy függőlegesen fölfelé legyen a robot nézési iránya
    map_rot = cv2.warpAffine(big_map, M_rot, (map_diag, map_diag))
    
    # Hol vagyunk a forgatás után, a nagy térképen?
    big_map_half = int(round(map_diag / 2))
    rad = deg * math.pi / 180
    pos_x_rot = int((pos_x_big - big_map_half) * math.cos(rad) + (pos_y_big - big_map_half) * math.sin(rad) + big_map_half)
    pos_y_rot = int(-(pos_x_big - big_map_half) * math.sin(rad) + (pos_y_big - big_map_half) * math.cos(rad) + big_map_half)
    
    # map_rot[pos_y_rot, pos_x_rot] = 0
    # cv2.imshow('map_rot', map_rot)
    # cv2.waitKey()
    
    
    # keressük meg a perspektívikusan torzítandó képrészlet 4 sarkát
    # ez itt puszta koordinátageometria kiegyszerűsítve, átláthatatlanul
    
    # print(pos_y_rot,pos_x_rot)
    
    y3 = pos_y_rot - int(round(h * math.tan((90 - view_angle_v / 2) * math.pi / 180)))  # 324
    y4 = np.where(big_map == 255)[0][0]  # 14
    
    m = math.tan((90 - view_angle_h / 2) / 180 * math.pi)  # 1.28
    b1 = pos_y_rot + m * pos_x_rot  # 711.6
    b2 = pos_y_rot - m * pos_x_rot  # 20.4
    
    p1x = int(round((y3 - b2) / m))
    p1y = int(round(m * p1x + b2))
    
    p2x = int(round((y4 - b2) / m))
    p2y = int(round(m * p2x + b2))
    
    p3x = int(round((-y4 + b1) / m))
    p3y = int(round(-m * p3x + b1))
    
    p4x = int(round((-y3 + b1) / m))
    p4y = int(round(-m * p4x + b1))
    
    # map_rot[p1y, p1x] = 0
    # map_rot[p4y, p4x] = 0
    # map_rot[p2y, p2x] = 255
    # map_rot[p3y, p3x] = 255
    # cv2.imshow('map_rot2', map_rot)
    # cv2.waitKey()
    
    # mennyire kell összenyomni a képet
    beta = math.atan((pos_y_rot - y4) / h) * 180 / math.pi - 90 + view_angle_v / 2
    ratio = beta / view_angle_v
    print(ratio)
    
    output_rows = int(round(cam_rows * ratio))
    output_cols = int(round(cam_cols))
    
    pts1 = np.float32([[p2x, p2y], [p3x, p3y], [p1x, p1y], [p4x, p4y]])
    pts2 = np.float32([[0, 0], [output_cols, 0], [0, output_rows], [output_cols, output_rows]])
    M_persp = cv2.getPerspectiveTransform(pts1, pts2)
    output = cv2.warpPerspective(map_rot, M_persp, (output_cols, output_rows))
    output = output*0.8
    
    output2 = np.array([cam_gray], dtype=int)
    output2[0, cam_rows - output_rows - 1:-1, :] = output
    output2 = np.where(output2 == 0, cam_gray, output2)
    
    final2 = np.zeros((cam_rows, cam_cols, 3))
    final2[:, :, 0] = cam_gray
    final2[:, :, 1] = output2
    final2[:, :, 2] = cam_gray
    
    
    
    #plt.matshow(final2)
    #plt.imshow(final2)
    #plt.show()
    #  cv2.imwrite("D:\\BME\\MSZO\\hostess robot\\2020\\kep2_res_rgb.jpg", final2)

    return final2
