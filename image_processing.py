#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mai 13 2024
@author: Ichikawa Eisei
"""
import numpy as np
import cv2

ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)


def get_orientation(contour):
    # Calculate the bounding box of the contour
    orientation = ""
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # Get the angle from the rect
    angle = rect[2]

    # Adjust the angle
    if angle < -45:
        angle = 90 + angle

    if -5 <= angle <= 5 or angle == 90 or angle == -90:
        orientation = "horizontally"
    elif angle < -5:
        orientation = "The contour is tilted_left"
    elif angle > 5:
        orientation = "The contour is tilted_right"
    else:
        orientation = "vertical"

    return orientation,angle


def get_moment(mask, threshold):
    orientation, angle = "",0
    num_labels, label_image, stats, center = cv2.connectedComponentsWithStats(mask)
    num_labels = num_labels - 1
    stats = np.delete(stats, 0, 0)
    center = np.delete(center, 0, 0)
    isExist = False
    x1, y1 = 0, 0
    area = 0
    if num_labels > 0:
        max_label = np.argmax(stats[:, 4])
        area = stats[max_label][4]
        x1, y1 = int(center[max_label][0]), int(center[max_label][1])
        if area > threshold:
            x1, y1 = int(center[max_label][0]), int(center[max_label][1])
            cv2.circle(mask, (x1, y1), 4, 100, 2, 4)
            isExist = True
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                orientation,angle = get_orientation(largest_contour)
    return [isExist, (x1, y1), area, mask.copy(), orientation, angle]


def find_line(roi_img, threshold):
    # Converter para escala de cinza
    imagem_cinza = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    # Limiarização
    _, imagem_binaria = cv2.threshold(imagem_cinza, 20, 255, cv2.THRESH_BINARY)  # buffalo = 127
    # Encontrar contornos
    contornos, _ = cv2.findContours(imagem_binaria, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # Criar uma máscara em branco do mesmo tamanho da imagem original
    mask = np.zeros_like(imagem_cinza)
    # Desenhar os contornos na máscara
    cv2.drawContours(mask, contornos, -1, 255, -1)  # Here the contour will be filled to give more accurance to line
    mask = cv2.bitwise_not(mask)
    # cv2.imshow("Trace", mask)
    return get_moment(mask, threshold)


def find_aruco(roi_ar):
    if roi_ar is None:
        return None, None, None
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(roi_ar, ARUCO_DICTIONARY)
    cv2.aruco.drawDetectedMarkers(roi_ar, corners, ids, (0, 255, 0))
    return corners, ids, roi_ar
