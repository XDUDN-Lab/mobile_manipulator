# -*- coding: utf-8 -*-
# canny 边缘提取 最终没有用
import cv2
import numpy as np
import random
def img_to_edge(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #将图像转为灰度图像
    gray = cv2.GaussianBlur(gray,(5,5),0)       #将图像进行高斯模糊
    edged = cv2.Canny(gray,75,200) 
    return edged  
def img_process(img):
    pass

if __name__=="__main__":
    img = cv2.imread("/home/derek/project/draw_robot/src/drawing_manipulator/draw_core/scripts/img/neu.jpg")
    cv2.imshow("img_ori",img)
    e = img_to_edge(img)
    cv2.imshow("img",e)
    _,coutours,_ = cv2.findContours(e,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    print len(coutours)
    coutours_img = np.zeros_like(img)
    for i in range(len(coutours)):
        cv2.drawContours(coutours_img,coutours,i,(random.randint(0,255),random.randint(0,255),random.randint(0,255)),2)
        cv2.imshow("img2",coutours_img)
        cv2.waitKey(500)
    cv2.waitKey(0)