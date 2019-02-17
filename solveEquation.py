from scipy.optimize import fsolve
from math import *


# valeurs en cm
L = 8.5         # distance entre un moteur et l'origine
d = 11.5        # hauteur à laquelle le plateau se situe par rapport à l'origine
D = 18          # distance entre le bout des trois bras des moteurs
r, l = 7,8.5    # r est la longueur de la première partie qui compose un bras, l est la longueur de la deuxième partie(celle sur laquelle se trouve la bille qui est en contact avec le plateau)


print("...")

alpha = 0
beta = 0
def equationsKMT(p):
    k, m, t = p
    equation1 = (-cos(alpha)*cos(pi/6)*k)**2 + (-cos(alpha)*m-cos(alpha)*sin(pi/6)*k)**2 + (sin(beta)*sin(alpha)*m+cos(pi/6)*cos(beta)*sin(alpha)*k+sin(beta)*sin(alpha)*sin(pi/6)*k)**2 - 1 # le -1 correspond normalement a D**2 donc avec -1 il y a une distance de 1 entre les point A, B et C
    equation2 = (-cos(alpha)*cos(pi/6)*t)**2 + (cos(alpha)*sin(pi/6)*t+cos(alpha)*m)**2 + (cos(beta)*sin(alpha)*cos(pi/6)*t-sin(beta)*sin(alpha)*sin(pi/6)*t-sin(beta)*sin(alpha)*m)**2 - 1
    equation3 = (cos(alpha)*cos(pi/6)*k+cos(alpha)*cos(pi/6)*t)**2 + (cos(alpha)*sin(pi/6)*k-cos(alpha)*sin(pi/6)*t)**2 + (-cos(pi/6)*cos(beta)*sin(alpha)*k-sin(beta)*sin(alpha)*sin(pi/6)*k-cos(beta)*sin(alpha)*cos(pi/6)*t+sin(beta)*sin(alpha)*sin(pi/6)*t)**2 - 1
    return (equation1, equation2, equation3)

def getMaxMinPoint(): #retourne le point le plus haut et le point le plus bas
    distanceMax = 0
    distanceMin = 100000
    for beta in range(0, 361):
        beta = radians(beta)
        for alpha in range(0, 36):
            alpha = radians(alpha)
            k,m,t = fsolve(equationsKMT, (1, 1, 1))
            k,m,t = -D*k, -D*m, -D*t
            Xa, Ya, Za = k*cos(alpha)*cos(pi/6), k*cos(alpha)*sin(pi/6), k*(-cos(pi/6)*cos(beta)*sin(alpha)-sin(beta)*sin(alpha)*sin(pi/6))+d
            
            ############################################################# trouver le point le plus eloigné et le plus proche du moteur
            if Za >= distanceMax:
                distanceMax =  sqrt((sqrt(Xa**2 + Ya**2)-L)**2 + Za**2)
                XaMax = Xa
                YaMax = Ya
                ZaMax = Za

            elif Za <= distanceMin:
                distanceMin =  sqrt((sqrt(Xa**2 + Ya**2)-L)**2 + Za**2)
                XaMin = Xa
                YaMin = Ya
                ZaMin = Za

    return (XaMax, YaMax, ZaMax, XaMin, YaMin, ZaMin)

def equationsRL(p): # système d'équation pour trouver les longueurs r et l
    xmax,ymax,zmax,xmin,ymin,zmin = getMaxMinPoint()
    print(p)
    r, l = p
    equation1 = (sqrt((xmin)**2 + (ymin)**2)-L+cos(radians(0))*r)**2 + (zmin-sin(radians(0))*r)**2 - l**2
    equation2 = (sqrt((xmax)**2 + (ymax)**2)-L+cos(radians(90))*r)**2 + (zmax-sin(radians(90))*r)**2 - l**2
    return (equation1, equation2)

def equationTeta(teta, *pointsPosition):
    x, y, z = pointsPosition
    return ((L-cos(teta)*r-sqrt(x**2+y**2))**2 + (sin(teta)*r-z)**2) - l**2       #### L + cos pour bras orienté vers l'extérieur et L - cos pour orienté vers l'intérieur

#r, l = fsolve(equationsRL, (9, 8))
#r, l = 9.05625,8.28459

fichier = open("/Users/johanlink/Desktop/TM/python/data.txt", "w") #le fichier txt est écrasé
premièreLigne = "alpha|beta|AngleservoA|AngleservoB|AngleservoC\n"
fichier.write(premièreLigne)


maxPos = 0
minPos = 1000
nbProbleme = 0
minMotorAngle = 100000
maxMotorAngle = 0

for beta in range(0, 360*5+1):
    beta = radians(beta)/5
    for alpha in range(0, 35*5+1):
        alpha = radians(alpha)/5
        k,m,t = fsolve(equationsKMT, (1, 1, 1))
        k,m,t = -D*k, -D*m, -D*t
        Xa, Ya, Za = k*cos(alpha)*cos(pi/6), k*cos(alpha)*sin(pi/6), k*(-cos(pi/6)*cos(beta)*sin(alpha)-sin(beta)*sin(alpha)*sin(pi/6))+d
        Xb, Yb, Zb = 0, m*(-cos(alpha)), m*sin(beta)*sin(alpha)+d
        Xc, Yc, Zc = t*(-cos(alpha)*cos(pi/6)), t*cos(alpha)*sin(pi/6), t*(cos(beta)*sin(alpha)*cos(pi/6)-sin(beta)*sin(alpha)*sin(pi/6))+d
        tetaA = degrees(fsolve(equationTeta, 1, args=(Xa, Ya, Za)))
        tetaB = degrees(fsolve(equationTeta, 1, args=(Xb, Yb, Zb)))
        tetaC = degrees(fsolve(equationTeta, 1, args=(Xc, Yc, Zc)))
        tetaA = round(tetaA, 2)
        tetaB = round(tetaB, 2)
        tetaC = round(tetaC, 2)

        separateur = "|"
        data = str(round(degrees(alpha),2)) +  separateur + str(round(degrees(beta),2)) + "#" + str(tetaA) + separateur + str(tetaB) + separateur + str(tetaC) + "\n"
        fichier.write(data)

        if tetaA > maxMotorAngle:
            maxMotorAngle = tetaA
            angleMaxPoint = (Xa, Ya, Za)

        if tetaA < minMotorAngle:
            minMotorAngle = tetaA
            angleMinPoint = (Xa, Ya, Za)
                


infos = "\nDistance L entre servo et origine:"+str(L)+"\nHauteur d du plateau:"+str(d)+"\nDistance D entre les points A,B,C:"+str(D)+"\nLongueur r du bras du servo:"+str(r)+"\nLongueur l du bras du servo:"+str(l)
minmaxAngle = "\nminMotorAngle = " + str(minMotorAngle) + "\nmaxMotorAngle = " + str(maxMotorAngle)
XaMax, YaMax, ZaMax, XaMin, YaMin, ZaMin = getMaxMinPoint()
minmaxPos = "\nle point le plus proche du servo:"+str(XaMin)+"    "+str(YaMin)+"    "+str(ZaMin)+"\nle point le plus éloigné du servo:"+str()+str(XaMax)+"    "+str(YaMax)+"    "+str(ZaMax)
fichier.write("\n")
fichier.write("valeurs en cm")
fichier.write(infos)
fichier.write(minmaxAngle)
fichier.write(minmaxPos)

fichier.close()

print("Terminé")
