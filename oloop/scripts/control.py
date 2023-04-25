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

##Setup trayectory parameters
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

#Variables del puzzlebot
# r = 0.05
# l = 0.18
maxVel = 1.0
minVel = 0.4

currPosition = basePosition
currAngle = 0.0

dist = 0.0
angle = 0.0
totalDist = 0.0
totalAng = 0.0

aVel = 0.6

def pointCallback(point):
   global trayPoints, pathMode, checkPoints
   pathPoint = [point.x, point.y]
   if (np.any(pathPoint != trayPoints[len(trayPoints)-1]) and pathMode == "custom"):
      trayPoints = np.append(trayPoints, [pathPoint], axis=0)
      checkPoints = np.append(checkPoints, [0], axis=0)

#Setup speed
if (sel == "speed"):
   lVel = speedR

   #Determina si la velocidad es adecuada o se sale de rango
   if (lVel < minVel):
      lVel = 0.0
      corOp = 0
      print("Speed too low for correct operation")
      print("Recommended speed: 0.5")
   if (lVel > maxVel):
      lVel = 0.0
      corOp = 0
      print("Speed too high for correct operation")
      print("Recommended speed: 0.5")

elif(sel == "time"):
   #Calcula distancia total y angulo total de giro
   xd = trayPoints[0][0] - currPosition[0]
   yd = trayPoints[0][1] - currPosition[1]
   totalDist = np.sqrt(np.square(xd) + np.square(yd))
   totalAng = abs(np.arctan2(yd,xd))

   for i in range(1, len(trayPoints)):
      xd = trayPoints[i][0] - trayPoints[i-1][0]
      yd = trayPoints[i][1] - trayPoints[i-1][1]

      totalDist = np.sqrt(np.square(xd) + np.square(yd))  + totalDist
      totalAng = abs(np.arctan2(yd,xd)) + totalAng
   
   #Calcula la velocidad linear necesaria para cumplir el tiempo considerando el tiempo de giro
   if ((timeR - (totalAng / aVel)) > 0):
      lVel = totalDist / (timeR - (totalAng / aVel))
   else:
      lVel = 0.0

   #Determina si la velocidad es adecuada o se sale de rango
   if (lVel < maxVel):
      lVel = 0.0
      corOp = 0
      print("Time too long for correct operation")
   if (lVel > maxVel):
      lVel = 0.0
      corOp = 0
      print("Time too short for correct operation")
      
   
#Realiza los calculos necesarios para ver a donde moverse
def trayectory(n):
   global dist, angle, checkPoints
   updatePos()
   difPos(n)
   move(dist, angle)
   #Si llega al punto deseado lo marca
   if(dist == 0.0 and angle == 0.0):
      checkPoints[n] = True

# Calcula la distancia entre la ubicacion actual del robot y su destino
# Calcula la diferencia de orientacion entre la actual y la necesaria para tener el punto de frente
def difPos(n):
   global trayPoints, currPosition, currAngle, dist, angle

   xd = trayPoints[n][0] - currPosition[0]
   yd = trayPoints[n][1] - currPosition[1]

   if(xd > 0.02 or xd < -0.02 or yd > 0.02 or yd < -0.02):
      dist = np.sqrt(np.square(xd) + np.square(yd))
      angle = wrapToPi(np.arctan2(yd,xd) - currAngle)
   else:
      dist = 0.0
      angle = 0.0

   print(trayPoints[n])
   print(angle)

#Resuelve primero la diferencia de angulo y luego resuelve la diferencia de posicion
def move(dist, angle):
   global aMov, lMov
   if (angle > 0.01):
      msg.linear.x = 0.0
      msg.angular.z = aVel
      aMov = 1
   elif (angle < -0.01):
      msg.linear.x = 0.0
      msg.angular.z = -aVel
      aMov = -1
   else:
      msg.angular.z = 0.0
      aMov = 0
      if(dist > 0.02):
         msg.linear.x = lVel
         lMov = 1
      else:
         msg.linear.x = 0.0
         lMov = 0

#Actualiza la posicion y orientacion del robot con la velocidad angular y lineal si se esta moviendo
#Asume un sistema con respuesta inmediata
def updatePos():
   global currPosition, currAngle, aMov, lMov, aVel, lVel
   tAngle = aVel * dt
   currAngle = aMov*tAngle + currAngle
   tDistance = lVel * dt
   currPosition[0] = lMov*(np.cos(currAngle) * tDistance) + currPosition[0]
   currPosition[1] = lMov*(np.sin(currAngle) * tDistance) + currPosition[1]
   print(currPosition)
   print(currAngle)

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
    rospy.Subscriber('/pose', Point, pointCallback)
    print("The Controller is Running")

    #Run the node
    while not rospy.is_shutdown():
      time_elapsed = float(rospy.get_time())
      dt = float(time_elapsed - prev_Time)
      #Evita saltos grandes en el tiempo
      if(dt > 1.0): dt = 0.0

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

