#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

#Setup path parameters
sel = rospy.get_param("/select", "speed")
speedR = rospy.get_param("/speed", 1.0)
timeR = rospy.get_param("/time", 2.0)
kw = rospy.get_param("/kw", 1.2)
iw = rospy.get_param("/iw", 0.00)
dw = rospy.get_param("/dw", 0.00)
kl = rospy.get_param("/kl", 0.45)
il = rospy.get_param("/il", 0.00)
dl = rospy.get_param("/dl", 0.00)

#Setup trayectory parameters
pathMode = np.array(rospy.get_param("/path", "square"))
basePosition = np.array(rospy.get_param("/bPosition", [0.0, 0.0]))
if (pathMode == "custom"):
   trayPoints = np.array([(0.0, 0.0)])
elif (pathMode == "square"):
   trayPoints = np.array([ (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)])
else:
   trayPoints = [(0.0, 0.0)]



#Setup checkpoints
checkPoints = np.zeros(len(trayPoints))

#Setup variables
lVel = 0.0
aVel = 0.0


time_elapsed = 0.0
init_time = 0.0
prev_Time = 0.0
dt = 0.0

msg = Twist()
msg.linear.x = 0
msg.angular.z = 0


#Flags
aMov = 0
lMov = 0
corOp = 1
stop = 0

#Variables del puzzlebot
r = 0.05
l = 0.19


currPosition = basePosition
currAngle = 0.0

dist = 0.0
angle = 0.0
totalDist = 0.0
totalAng = 0.0

aVel = 0.6

wr = 0.0
wl = 0.0

#Error parameters
errorPl = 0
errorIl = 0
errorDl = 0
prev_errorL = 0

errorPa = 0
errorIa = 0
errorDa = 0
prev_errorA = 0

uw = 0
ul = 0

#Callbacks
def wrCallback(ar):
   global wr
   wr = ar.data

def wlCallback(al):
   global wl
   wl = al.data

def pointCallback(point):
   global trayPoints, pathMode, checkPoints
   pathPoint = [point.x, point.y]
   if (np.any(pathPoint != trayPoints[len(trayPoints)-1]) and pathMode == "custom"):
      trayPoints = np.append(trayPoints, [pathPoint], axis=0)
      checkPoints = np.append(checkPoints, [0], axis=0)


   
#Realiza los calculos necesarios para ver a donde moverse
def trayectory(n):
   global dist, angle, checkPoints, pathMode
   updatePos()
   difPos(n)
   move(dist, angle)
   #Si llega al punto deseado lo marca
   if(dist == 0.0 and angle == 0.0):
      checkPoints[n] = True
      

# Calcula el error entre la ubicacion actual del robot y su destino
# Calcula el error de orientacion entre la actual y la necesaria para tener el punto de frente
def difPos(n):
   global trayPoints, currPosition, currAngle, dist, angle

   xd = trayPoints[n][0] - currPosition[0]
   yd = trayPoints[n][1] - currPosition[1]

   if(xd > 0.2 or xd < -0.2 or yd > 0.2 or yd < -0.2):
      dist = np.sqrt(np.square(xd) + np.square(yd))
      angle = wrapToPi(np.arctan2(yd,xd) - currAngle)
      if (np.abs(angle) < 0.1):
         angle = 0
   else:
      dist = 0.0
      angle = 0.0

   print(angle)
   print(dist)

#Controlador de velocidades angulares y lineales
#Resuelve primero la diferencia de angulo y luego resuelve la diferencia de posicion
def move(dist, angle):
   global aMov, lMov, stop, errorPl, errorIl, errorDl, errorPa, errorIa, errorDa, prev_errorA, prev_errorL
   global kw, kl, il, iw, dw, dl, wr, wl, uw, ul

   errorPl = dist 
   errorIl = errorIl + errorPl*dt
   errorDl = (errorPl - prev_errorL)/dt
   
   errorPa = angle
   errorIa = errorIa + errorPa*dt
   errorDa = (errorPa - prev_errorA)/dt

   uw = kw * errorPa + iw *errorIa + dw * errorDa
   ul = kl * errorPl + il *errorIl + dl * errorDl

   if(uw > 1): uw = 11
   if(ul > 1): ul = 1
   

   if (np.abs(angle) > 0.1):
      msg.linear.x = 0.0
      msg.angular.z = uw
      aMov = 1
      #if (stop == 0):
       #  rospy.sleep(0.1)
        # stop = 1
      
   else:
         
      msg.angular.z = 0.0

      if(dist > 0.02 and (np.abs(r * (wr - wl) / l )<0.2)):
         msg.linear.x = ul
         lMov = 1
      else:
         msg.linear.x = ul         
         lMov = -1
         stop = 0
   

   prev_errorA = errorPa
   prev_errorL = errorPl

#Actualiza la posicion y orientacion del robot con la velocidad angular y lineal si se esta moviendo
#Asume un sistema con respuesta inmediata
def updatePos():
   global currPosition, currAngle, aMov, lMov, aVel, lVel, wr, wl
   tAngle = r * (wr - wl) / l * dt
   currAngle = aMov*tAngle + currAngle
   tDistance = r * (wr + wl) / 2 * dt
   currPosition[0] = lMov*(np.cos(currAngle) * tDistance) + currPosition[0]
   currPosition[1] = lMov*(np.sin(currAngle) * tDistance) + currPosition[1]
   #print(currPosition)
   #print(currAngle)
   

def wrapToPi(ang):
   result = np.fmod((ang + np.pi),(2*np.pi))
   if(result < 0):
      result += 2 * np.pi
   return result - np.pi

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pubVel.publish(msg)
    print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Controller")
    loop_rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers
    pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/wl', Float32, wlCallback)
    rospy.Subscriber('/wr', Float32, wrCallback)
    rospy.Subscriber('/pose', Point, pointCallback)
    print("The Controller is Running")

    #Run the node
    while not rospy.is_shutdown():
      time_elapsed = float(rospy.get_time())
      dt = float(time_elapsed - prev_Time)
      #Evita saltos grandes en el tiempo
      if(dt > 1.0 or dt == 0): dt = 0.01

      #Revisa si el puzzlebot puede operar correctamente
      if(corOp):
      
      #Revisa si no se ha cruzado el checkpoint y se mueve hacia el
         for i in range(0, len(checkPoints)):
            if (not(checkPoints[i])):
               n = i
               trayectory(n)
               break

      pubVel.publish(msg)
      prev_Time = float(time_elapsed)

      loop_rate.sleep()

