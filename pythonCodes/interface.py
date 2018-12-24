import cv2
import numpy as np
import time
import imutils
import tkinter as tk
import tkinter.messagebox
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
from math import *


lines = open("/Users/johanlink/Desktop/TM/python/data.txt").read().splitlines()
lines = lines[:-11]     #enlève les 11 dernières lignes du fichier
lines = lines[1:]       #enlève la première ligne du fichier

dataDict = {}

camHeight = 480
camWidth = 640
cam = cv2.VideoCapture(0)
cam.set(3,camWidth)
cam.set(4,camHeight)

getPixelColor = False
H,S,V = 0,0,0

mouseX,mouseY = 0,0

for i in range(0,len(lines)):
    key, value = lines[i].split("#")
    alpha, beta = key.split("|")
    angleA, angleB, angleC = value.split("|")
    dataDict[(float(alpha),float(beta))] = (float(angleA), float(angleB), float(angleC))

controllerWindow = tk.Tk()
controllerWindow.title("fenêtre de contrôle")
controllerWindow.geometry("820x500")
controllerWindow["bg"]="white"
controllerWindow.resizable(0, 0)

videoWindow = tk.Toplevel(controllerWindow)
videoWindow.title("retour caméra")
videoWindow.resizable(0, 0)  #empeche de modifier les dimensions de la fenêtre
lmain = tk.Label(videoWindow)
lmain.pack()
videoWindow.withdraw()

graphWindow = tk.Toplevel(controllerWindow)
graphWindow.title("Position en fonction du temps")
graphWindow.resizable(0, 0)
graphCanvas = tk.Canvas(graphWindow,width=camHeight+210,height=camHeight)
graphCanvas.pack()
graphWindow.withdraw()

pointsListCircle = []
def createPointsListCircle(rayon):
    global pointsListCircle
    for angle in range(0,360):
        angle=angle-90
        pointsListCircle.append([rayon*cos(radians(angle))+240,rayon*sin(radians(angle))+240])
createPointsListCircle(150)

pointsListEight = []
def createPointsListEight(rayon):
    global pointsListEight
    for angle in range(270,270+360):
        pointsListEight.append([rayon*cos(radians(angle))+240,rayon*sin(radians(angle))+240+rayon])
    for angle in range(360,0,-1):
        angle=angle+90
        pointsListEight.append([rayon*cos(radians(angle))+240,rayon*sin(radians(angle))+240-rayon])
createPointsListEight(80)

drawCircleBool = False
def startDrawCircle():
    global drawCircleBool, drawEightBool, consigneX, consigneY
    if drawCircleBool == False:
        drawCircleBool = True
        BballDrawCircle["text"] = "Centrer la bille"
    else:
        drawCircleBool = False
        consigneX, consigneY = 240, 240
        sliderCoefP.set(sliderCoefPDefault)
        BballDrawCircle["text"] = "Faire tourner la bille en cercle"

drawEightBool = False
def startDrawEight():
    global drawEightBool, drawCircleBool, consigneX, consigneY
    if drawEightBool == False:
        drawEightBool = True
        BballDrawEight["text"] = "Centrer la bille"
    else:
        drawEightBool = False
        consigneX, consigneY = 240, 240
        sliderCoefP.set(sliderCoefPDefault)
        BballDrawEight["text"] = "Faire tourner la bille en huit"

pointCounter = 0
def drawWithBall():
    global pointCounter, consigneX, consigneY
    if drawCircleBool == True:
        sliderCoefP.set(15)
        if pointCounter >= len(pointsListCircle):
            pointCounter = 0
        point = pointsListCircle[pointCounter]
        consigneX, consigneY = point[0], point[1]
        pointCounter += 7
    if drawEightBool == True:
        sliderCoefP.set(15)
        if pointCounter >= len(pointsListEight):
            pointCounter = 0
        point = pointsListEight[pointCounter]
        consigneX, consigneY = point[0], point[1]
        pointCounter += 7


def setConsigneWithMouse(mousePosition):
    global consigneX, consigneY
    if mousePosition.y > 10:
        refreshGraph()
        consigneX,consigneY = mousePosition.x,mousePosition.y


def getMouseClickPosition(mousePosition):
    global mouseX,mouseY
    global getPixelColor
    mouseX,mouseY = mousePosition.x,mousePosition.y
    getPixelColor = True

showVideoWindow = False
def showCameraFrameWindow():
    global showVideoWindow, showGraph
    global BRetourVideoTxt
    if showVideoWindow == False:
        if showGraph == True:
            graphWindow.withdraw()
            showGraph = False
            BafficherGraph["text"] = "Afficher graphique"
        videoWindow.deiconify()
        showVideoWindow = True
        BRetourVideo["text"] = "Cacher le retour vidéo "
    else:
        videoWindow.withdraw()
        showVideoWindow = False
        BRetourVideo["text"] = "Afficher le retour vidéo"

showCalqueCalibrationBool = False
def showCalqueCalibration():
    global showCalqueCalibrationBool
    showCalqueCalibrationBool = not showCalqueCalibrationBool

showGraph = False
def showGraphWindow():
    global showGraph, showVideoWindow
    global BafficherGraph
    
    if showGraph == False:
        if showVideoWindow == True:
            videoWindow.withdraw()
            showVideoWindow = False
            BRetourVideo["text"] = "Afficher le retour vidéo"
        showGraph = True
        BafficherGraph["text"] = "Cacher graphique "
    else:
        showGraph = False
        BafficherGraph["text"] = "Afficher graphique"

t = 480
consigneY = 240
consigneX = 240
def paintGraph():
    global t,consigneY,x,y,prevX,prevY,alpha,prevAlpha
    global showGraphPositionX,showGraphPositionY, showGraphAlpha
    if showGraph == True:
        graphWindow.deiconify()
        if showGraphPositionX.get() == 1:
            graphCanvas.create_line(t-3,prevX,t,x, fill="#b20000", width=2)
        if showGraphPositionY.get() == 1:
            graphCanvas.create_line(t-3,prevY,t,y, fill="#0069b5", width=2)
        if showGraphAlpha.get() == 1:
            graphCanvas.create_line(t-3,240-prevAlpha*3,t,240-alpha*3, fill="#8f0caf", width=2)
        if t >= 480:
            t = 0
            graphCanvas.delete("all")
            graphCanvas.create_line(3,3,480,3,fill="black", width=3)
            graphCanvas.create_line(3,480,480,480,fill="black", width=3)
            graphCanvas.create_line(3,3,3,480,fill="black", width=3)
            graphCanvas.create_line(480,3,480,480,fill="black", width=3)
            graphCanvas.create_line(550,32,740,32,fill="#b20000", width=5)
            graphCanvas.create_line(550,53,740,53,fill="#0069b5", width=5)
            graphCanvas.create_line(550,73,740,73,fill="#8f0caf", width=5)
            if showGraphPositionX.get() == 1:
                graphCanvas.create_line(3,consigneX,480,consigneX,fill="#ff7777", width=2)
            if showGraphPositionY.get() == 1:
                graphCanvas.create_line(3,consigneY,480,consigneY,fill="#6f91f7", width=2)
        t += 3
    else:
        graphWindow.withdraw()

def refreshGraph():
    global t
    t=480

def endProgam():
    controllerWindow.destroy()


sliderHDefault = 15
sliderSDefault = 70
sliderVDefault = 70
sliderCoefPDefault = 10
sliderCoefIDefault = 0.1
sliderCoefDDefault = 5.7

def resetSlider():
    sliderH.set(sliderHDefault)
    sliderS.set(sliderSDefault)
    sliderV.set(sliderVDefault)
    sliderCoefP.set(sliderCoefPDefault)
    sliderCoefI.set(sliderCoefIDefault)
    sliderCoefD.set(sliderCoefDDefault)

def donothing():
    pass

def rangerPlateau():
    if arduinoIsConnected == True:
        if tkinter.messagebox.askokcancel("Avertissement", "Pensez à retirer le plateau."):
            print("abaissement des bras")
            ser.write(("descendreBras\n").encode())
    else:
        if tkinter.messagebox.askokcancel("Avertissement","L'Arduino n'est pas connecté"):
            donothing()


def eleverPlateau():
    global alpha
    if arduinoIsConnected == True:
        if tkinter.messagebox.askokcancel("Avertissement", "Pensez à retirer le plateau."):
            print("Elevation des bras")
            ser.write((str(dataDict[(0,0)])+"\n").encode())
            alpha = 0
    else:
        if tkinter.messagebox.askokcancel("Avertissement","L'Arduino n'est pas connecté"):
            donothing()

def servosTest():
    if arduinoIsConnected == True:
        if tkinter.messagebox.askokcancel("Avertissement", "Le plateau doit être en place."):
            for i in range(2):
                beta = 0
                alpha = 35
                while beta < 360:
                    ser.write((str(dataDict[(alpha,beta)])+"\n").encode())
                    ser.flush()
                    time.sleep(0.002)
                    beta = round(beta+0.2,2)
                    print(alpha,beta)
            time.sleep(1)
            ser.write((str(dataDict[(0,0)])+"\n").encode())
    else:
        if tkinter.messagebox.askokcancel("Avertissement","L'Arduino n'est pas connecté"):
            donothing()


arduinoIsConnected = False
def connectArduino():
    global ser
    global label
    global arduinoIsConnected
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" in p.description:
            print(p)
            ser = serial.Serial(p[0], 19200, timeout=1)
            time.sleep(1) #give the connection a second to settle
            label.configure(text="Arduino connecté", fg="#36db8b")
            arduinoIsConnected = True

startBalanceBall = False
def startBalance():
    global startBalanceBall
    if arduinoIsConnected == True:
        if startBalanceBall == False:
            startBalanceBall = True
            BStartBalance["text"] = "Arrêter"
        else:
            startBalanceBall = False
            BStartBalance["text"] = "Commencer"
    else:
        if tkinter.messagebox.askokcancel("Avertissement","L'Arduino n'est pas connecté"):
            donothing()

sommeErreurX = 1
sommeErreurY = 1
timeInterval = 1
alpha, beta, prevAlpha, prevBeta = 0,0,0,0
omega = 0.2
def PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, consigneX, consigneY):
    global omega
    global sommeErreurX, sommeErreurY
    global alpha, beta, prevAlpha, prevBeta
    global startBalanceBall, arduinoIsConnected
    
    Kp = sliderCoefP.get()
    Ki = sliderCoefI.get()
    Kd = sliderCoefD.get()

    Ix = Kp*(consigneX-ballPosX) + Ki*sommeErreurX + Kd*((prevBallPosX-ballPosX)/0.0333)
    Iy = Kp*(consigneY-ballPosY) + Ki*sommeErreurY + Kd*((prevBallPosY-ballPosY)/0.0333)
    
    Ix = round(Ix/10000, 4)
    Iy = round(Iy/10000, 4)
    
    if Ix == 0 and Iy == 0:
        alpha = 0
        beta = 0
    
    elif Ix != 0 and sqrt(Ix**2 + Iy**2) < 1:
        beta = atan(Iy/Ix)
        alpha = asin(sqrt(Ix**2 + Iy**2))
        beta = degrees(beta)
        alpha = degrees(alpha)
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180-abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180+abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360-abs(beta)

    elif Ix == 0 and sqrt(Ix**2 + Iy**2) < 1:
        if Iy > 0:
            beta = 90
            alpha = asin(sqrt(Ix**2 + Iy**2))
        elif Iy < 0:
            beta = 270
            alpha = asin(sqrt(Ix**2 + Iy**2))
        alpha = degrees(alpha)

    elif Ix != 0 and sqrt(Ix**2 + Iy**2) > 1:
        beta = degrees(atan(Iy/Ix))
        alpha = 35
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180-abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180+abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360-abs(beta)

    elif Ix == 0 and sqrt(Ix**2 + Iy**2) > 1:
        alpha = 35
        if Iy > 0:
            beta = 90
        elif Iy < 0:
            beta = 270

    if alpha > 35:
        alpha = 35

    alpha = prevAlpha * omega + (1-omega) * alpha
    beta = prevBeta * omega + (1-omega) * beta

    alpha = round(round(alpha / 0.2) * 0.2, -int(floor(log10(0.2))))   ## permet d'arrondire avec 0.2 de précision
    beta = round(round(beta / 0.2) * 0.2, -int(floor(log10(0.2))))

    if alpha <= 35 and beta <= 360 and arduinoIsConnected == True and startBalanceBall == True:
        ser.write((str(dataDict[(alpha,beta)])+"\n").encode())

    #print(alpha, beta)
    print(Ix,Iy,alpha,beta,ballPosX,ballPosY,prevBallPosX,prevBallPosY,sommeErreurX,sommeErreurY)

    if startBalanceBall == True:
        sommeErreurX += (consigneX-ballPosX)
        sommeErreurY += (consigneY-ballPosY)


prevX,prevY = 0,0
prevConsigneX, prevConsigneY = 0,0
start_time = 0
def main():
    start_timeFPS = time.time()
    global H,S,V
    global getPixelColor
    global x,y, alpha, beta
    global prevX, prevY, prevAlpha, prevBeta, prevConsigneX, prevConsigneY
    global consigneX, consigneY, sommeErreurX, sommeErreurY
    global camWidth,camHeight
    global timeInterval, start_time
    global showVideoWindow
    
    _, img=cam.read()
    img = img[0:int(camHeight),int((camWidth-camHeight)/2):int(camWidth-((camWidth-camHeight)/2))] #[Y1:Y2,X1:X2]
    imgCircle = np.zeros(img.shape, dtype=np.uint8)
    cv2.circle(imgCircle, (240,240), 270, (255, 255, 255), -1, 8, 0)
    img = img & imgCircle
    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    if getPixelColor == True and mouseY > 0 and mouseY < 480 and mouseX < 480:
        pixelColorOnClick = img[mouseY,mouseX]
        pixelColorOnClick = np.uint8([[pixelColorOnClick]])
        pixelColorOnClick = cv2.cvtColor(pixelColorOnClick,cv2.COLOR_BGR2HSV)
        H = pixelColorOnClick[0,0,0]
        S = pixelColorOnClick[0,0,1]
        V = pixelColorOnClick[0,0,2]
        print(mouseX, mouseY)
        getPixelColor = False
    
    lowerBound=np.array([H-sliderH.get(),S-sliderS.get(),V-sliderV.get()])
    upperBound=np.array([H+sliderH.get(),S+sliderS.get(),V+sliderV.get()])

    mask=cv2.inRange(imgHSV,lowerBound,upperBound)
    mask = cv2.blur(mask,(6,6))                        # ajoute du flou à l'image
    mask = cv2.erode(mask, None, iterations=2)         # retire les parasites
    mask = cv2.dilate(mask, None, iterations=2)        # retire les parasites
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    center = None

    cv2.circle(img, (int(consigneX), int(consigneY)), int(4),(255, 0, 0), 2)
    if showCalqueCalibrationBool == True:
        cv2.circle(img, (240,240), 220,(255, 0, 0), 2)
        cv2.circle(img, (240,240), 160,(255, 0, 0), 2)
        cv2.line(img, (240, 240), (240, 240+160), (255,0,0), 2)
        cv2.line(img, (240, 240), (240+138, 240-80), (255,0,0), 2)
        cv2.line(img, (240, 240), (240-138, 240-80), (255,0,0), 2)
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        timeInterval = time.time() - start_time
        (x, y), radius = cv2.minEnclosingCircle(c)
        if radius > 10:
            cv2.putText(img,str(int(x)) + ";" + str(int(y)).format(0, 0),(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
            cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            PIDcontrol(int(x),int(y),prevX,prevY,consigneX,consigneY)
            start_time = time.time()
    else:
        sommeErreurX, sommeErreurY = 0,0


    if showVideoWindow == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        imgtk = ImageTk.PhotoImage(image=img)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)
    lmain.after(5, main)

    drawWithBall()
    if prevConsigneX != consigneX or prevConsigneY != consigneY:
        sommeErreurX, sommeErreurY = 0,0

    paintGraph()
    prevX,prevY = int(x), int(y)
    prevConsigneX, prevConsigneY = consigneX, consigneY
    prevAlpha = alpha
    prevBeta = beta

    print("FPS: ", 1.0 / (time.time() - start_timeFPS))



FrameVideoControl = tk.LabelFrame(controllerWindow, text="Vidéo contrôle")
FrameVideoControl.place(x=20,y=20,width=380)
BRetourVideo = tk.Button(FrameVideoControl, text="Afficher le retour vidéo", command=showCameraFrameWindow)
BRetourVideo.pack()
BPositionCalibration = tk.Button(FrameVideoControl, text="Calque", command=showCalqueCalibration)
BPositionCalibration.place(x=290,y=0)

sliderH = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Sensibilité H", length=350, tickinterval = 10)
sliderH.set(sliderHDefault)
sliderH.pack()
sliderS = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Sensibilité S", length=350, tickinterval = 10)
sliderS.set(sliderSDefault)
sliderS.pack()
sliderV = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Sensibilité V", length=350, tickinterval = 10)
sliderV.set(sliderVDefault)
sliderV.pack()



FrameServosControl = tk.LabelFrame(controllerWindow, text="Servos contrôle")
FrameServosControl.place(x=20,y=315,width=380)
BAbaissementPlateau = tk.Button(FrameServosControl, text="Ranger les bras", command=rangerPlateau)
BAbaissementPlateau.pack()
BElevationBras = tk.Button(FrameServosControl, text="Mettre en place le plateau", command=eleverPlateau)
BElevationBras.pack()
BTesterServos = tk.Button(FrameServosControl, text="Tester les servomoteurs", command=servosTest)
BTesterServos.pack()
BStartBalance = tk.Button(FrameServosControl, text="Démarrer", command=startBalance, highlightbackground = "#36db8b")
BStartBalance.pack()



FramePIDCoef = tk.LabelFrame(controllerWindow, text="PID coefficients")
FramePIDCoef.place(x=420,y=20,width=380)
BafficherGraph = tk.Button(FramePIDCoef, text="Afficher graphique", command=showGraphWindow)
BafficherGraph.pack()
sliderCoefP = tk.Scale(FramePIDCoef, from_=0, to=15, orient="horizontal", label="P", length=350, tickinterval = 3, resolution=0.01)
sliderCoefP.set(sliderCoefPDefault)
sliderCoefP.pack()
sliderCoefI = tk.Scale(FramePIDCoef, from_=0, to=1, orient="horizontal", label="I", length=350, tickinterval = 0.2, resolution=0.001)
sliderCoefI.set(sliderCoefIDefault)
sliderCoefI.pack()
sliderCoefD = tk.Scale(FramePIDCoef, from_=0, to=10, orient="horizontal", label="D", length=350, tickinterval = 2, resolution=0.01)
sliderCoefD.set(sliderCoefDDefault)
sliderCoefD.pack()


FrameBallControl = tk.LabelFrame(controllerWindow, text="Bille contrôle")
FrameBallControl.place(x=420,y=315,width=380, height= 132)
BballDrawCircle = tk.Button(FrameBallControl, text="Faire tourner la bille en cercle", command=startDrawCircle)
BballDrawCircle.pack()
BballDrawEight = tk.Button(FrameBallControl, text="Faire tourner la bille en huit", command=startDrawEight)
BballDrawEight.pack()




label = tk.Label(controllerWindow, text="Arduino déconnecté  ", fg="red", anchor="ne")
label.pack(fill="both")
BReset = tk.Button(controllerWindow, text = "Reset", command = resetSlider)
BReset.place(x=20, y=460)
BConnect = tk.Button(controllerWindow, text = "Connexion", command = connectArduino, background="black")
BConnect.place(x=100, y=460)
BQuit = tk.Button(controllerWindow, text = "Quitter", command = endProgam)
BQuit.place(x=730, y=460)


showGraphPositionX = tk.IntVar()
showGraphPositionX.set(1)
CheckbuttonPositionX = tk.Checkbutton(graphWindow, text="Position en X", variable=showGraphPositionX, command=refreshGraph)
CheckbuttonPositionX.place(x=500,y=20)
showGraphPositionY = tk.IntVar()
showGraphPositionY.set(1)
CheckbuttonPositionY = tk.Checkbutton(graphWindow, text="Position en Y", variable=showGraphPositionY, command=refreshGraph)
CheckbuttonPositionY.place(x=500,y=40)
showGraphAlpha = tk.IntVar()
CheckbuttonAlpha = tk.Checkbutton(graphWindow, text="Inclinaison du plateau", variable=showGraphAlpha, command=refreshGraph)
CheckbuttonAlpha.place(x=500,y=60)



videoWindow.protocol("WM_DELETE_WINDOW",donothing)
videoWindow.bind("<Button-2>",getMouseClickPosition)
videoWindow.bind("<Button-1>",setConsigneWithMouse)

main()
tk.mainloop()






