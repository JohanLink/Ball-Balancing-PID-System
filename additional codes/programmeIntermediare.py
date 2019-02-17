import cv2
import numpy as np
import time
import imutils

cam = cv2.VideoCapture(0)
cv2.namedWindow("image")

cam.set(3,320)
cam.set(4,240)

getPixelColor = False
H,S,V = 0,0,0

mouseX,mouseY = 0,0

def getMousePositionOnClick(event,x,y,flags,param):
    global mouseX,mouseY
    global getPixelColor
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX,mouseY = x,y
        getPixelColor = True
        print(getPixelColor)
    elif event == cv2.EVENT_LBUTTONUP:
        getPixelColor = False


cv2.setMouseCallback("image",getMousePositionOnClick)

while True:
    start_time = time.time()
    ret, img=cam.read()
    imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    if getPixelColor == True:
        pixelColorOnClick = img[mouseY,mouseX]
        pixelColorOnClick = np.uint8([[pixelColorOnClick]])
        pixelColorOnClick = cv2.cvtColor(pixelColorOnClick,cv2.COLOR_BGR2HSV)
        H = pixelColorOnClick[0,0,0]
        S = pixelColorOnClick[0,0,1]
        V = pixelColorOnClick[0,0,2]
        print(H, S, V, mouseX, mouseY)
    
    lowerBound=np.array([H-7,S-70,V-70])
    upperBound=np.array([H+7,S+70,V+70])
    
    mask=cv2.inRange(imgHSV,lowerBound,upperBound)
    mask = cv2.blur(mask,(7,7))                        # ajoute du flou à l'image
    mask = cv2.erode(mask, None, iterations=2)         # retire les parasites
    mask = cv2.dilate(mask, None, iterations=2)        # retire les parasites
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    center = None
    
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)
        
        if radius > 2:
            cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)

    cv2.imshow("mask",mask)
    cv2.imshow("image",img)
    cv2.waitKey(10)
    print("FPS: ", 1.0 / (time.time() - start_time))

