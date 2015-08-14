import pygame
from pygame.locals import *

import Box2D
from Box2D.b2 import *

import math

import numpy as np

TARGET_FPS=60
PPM=20.0
TIME_STEP=1.0/TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT=1200, 600

colors = {
    staticBody  : (255,255,255,255),
    dynamicBody : (127,127,127,255),
}

screen=pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Body')
clock=pygame.time.Clock()
world=world(gravity=(0,-9.81),doSleep=True)

ground=world.CreateStaticBody(position=(30,2))
box=ground.CreatePolygonFixture(box=(30,3), density=1, friction=0.5)
box.filterData.categoryBits = 1
box.filterData.maskBits = 14

ang1 = -pi/12
ang2 = pi/12
ang3 = -pi/12
ang4 = pi/12

hi = 12.6

##if (10.8-2.8*math.cos(ang1)-2.8*math.cos(ang3))<(10.8-2.8*math.cos(ang2)-2.8*math.cos(ang4)):
##    hi = 7.2+2.8*math.cos(ang1)+2.8*math.cos(ang3)
##else:
##    hi = 7.2+2.8*math.cos(ang2)+2.8*math.cos(ang4)


torso=world.CreateDynamicBody(position=(5,hi), angle=0)
box=torso.CreatePolygonFixture(box=(0.2,2), density=1, friction=0.5)
box.filterData.categoryBits = 2
box.filterData.maskBits = 13

l1 = 5+1.4*math.sin(ang1)
l2 = hi-2-1.4*math.cos(ang1)
left_leg=world.CreateDynamicBody(position=(l1,l2), angle = ang1)
box=left_leg.CreatePolygonFixture(box=(0.2,1.4), density=1, friction=0.5)
box.filterData.categoryBits = 4
box.filterData.maskBits = 7

l1 = 5+1.4*math.sin(ang2)
l2 = hi-2-1.4*math.cos(ang2)
right_leg=world.CreateDynamicBody(position=(l1,l2), angle = ang2)
box=right_leg.CreatePolygonFixture(box=(0.2,1.4), density=1, friction=0.5)
box.filterData.categoryBits = 8
box.filterData.maskBits = 11

l1 = 5+2.8*math.sin(ang1)+1.4*math.sin(ang3)
l2 = hi-2-2.8*math.cos(ang1)-1.4*math.cos(ang3)
left_underknee=world.CreateDynamicBody(position=(l1,l2), angle = ang3)
box=left_underknee.CreatePolygonFixture(box=(0.2,1.4), density=1, friction=0.5)
box.filterData.categoryBits = 4
box.filterData.maskBits = 7

l1 = 5+2.8*math.sin(ang2)+1.4*math.sin(ang4)
l2 = hi-2-2.8*math.cos(ang2)-1.4*math.cos(ang4)
right_underknee=world.CreateDynamicBody(position=(l1,l2), angle = ang4)
box=right_underknee.CreatePolygonFixture(box=(0.2,1.4), density=1, friction=0.5)
box.filterData.categoryBits = 8
box.filterData.maskBits = 11

##l1 = 5+2.8*math.sin(ang1)+2.8*math.sin(ang3)-0.2*math.cos(ang3)+0.72
##l2 = hi-2-2.8*math.cos(ang1)-2.8*math.cos(ang3)
##left_foot=world.CreateDynamicBody(position=(l1,l2), angle=0)
##box=left_foot.CreatePolygonFixture(box=(0.72,0.2), density=1, friction=0.5)
##box.filterData.categoryBits = 4
##box.filterData.maskBits = 7
##
##l1 = 5+2.8*math.sin(ang2)+2.8*math.sin(ang4)-0.2*math.cos(ang4)+0.72
##l2 = hi-2-2.8*math.cos(ang2)-2.8*math.cos(ang4)
##right_foot=world.CreateDynamicBody(position=(l1,l2), angle=0)
##box=right_foot.CreatePolygonFixture(box=(0.72,0.2), density=1, friction=0.5)
##box.filterData.categoryBits = 8
##box.filterData.maskBits = 11

#torso.ApplyLinearImpulse(impulse=(100,0), point=torso.worldCenter, wake = True)
#left_underknee.ApplyLinearImpulse(impulse=(100,0), point=left_underknee.worldCenter, wake = True)
#world.CreateDistanceJoint(bodyA=ground_body, bodyB=dynamic_body, anchorA=ground_body.worldCenter, anchorB=dynamic_body.worldCenter)

left_hip=world.CreateRevoluteJoint(
    bodyA=torso, 
    bodyB=left_leg, 
    anchor=(5,hi-2),
    lowerAngle = -ang1-pi*0.05,
    upperAngle = pi-ang1-pi*0.05,
    enableLimit = True,
    maxMotorTorque = 0.0,
    motorSpeed = 1e3,
    enableMotor = True,
    )

right_hip=world.CreateRevoluteJoint(
    bodyA=torso, 
    bodyB=right_leg, 
    anchor=(5,hi-2),
    lowerAngle = -ang2-pi*0.05,
    upperAngle = pi-ang2-pi*0.05,
    enableLimit = True,
    maxMotorTorque = 0.0,
    motorSpeed = 1e3,
    enableMotor = True,
    )

left_knee=world.CreateRevoluteJoint(
    bodyA=left_leg, 
    bodyB=left_underknee, 
    anchor=(5+2.8*math.sin(ang1),hi-2-2.8*math.cos(ang1)),
    lowerAngle = ang1-ang3-pi+0.05*pi,
    upperAngle = ang1-ang3,
    enableLimit = True,
    maxMotorTorque = 0.0,
    motorSpeed = 1e3,
    enableMotor = True,
    )

right_knee=world.CreateRevoluteJoint(
    bodyA=right_leg, 
    bodyB=right_underknee, 
    anchor=(5+2.8*math.sin(ang2),hi-2-2.8*math.cos(ang2)),
    lowerAngle = ang2-ang4-pi+0.05*pi,
    upperAngle = ang2-ang4,
    enableLimit = True,
    maxMotorTorque = 0.0,
    motorSpeed = 1e3,
    enableMotor = True,
    )

##left_ankle=world.CreateRevoluteJoint(
##    bodyA=left_underknee, 
##    bodyB=left_foot, 
##    anchor=(5+2.8*math.sin(ang1)+2.8*math.sin(ang3),hi-2-2.8*math.cos(ang1)-2.8*math.cos(ang3)),
##    lowerAngle = ang3-0.5*pi+0.05*pi,
##    upperAngle = ang3+0.5*pi-1,
##    enableLimit = True,
##    maxMotorTorque = 0.0,
##    motorSpeed = 0.0,
##    enableMotor = False,
##    )
##
##right_ankle=world.CreateRevoluteJoint(
##    bodyA=right_underknee, 
##    bodyB=right_foot, 
##    anchor=(5+2.8*math.sin(ang2)+2.8*math.sin(ang4),hi-2-2.8*math.cos(ang2)-2.8*math.cos(ang4)),
##    lowerAngle = ang4-0.5*pi+0.05*pi,
##    upperAngle = ang4+0.5*pi-1,
##    enableLimit = True,
##    maxMotorTorque = 0.0,
##    motorSpeed = 0.0,
##    enableMotor = False,
##    )

M = np.zeros((4,))       
W = np.array([[-3.37285237, -0.51124511, -0.1797195 ,  1.06501462],
 [-2.83073888,  4.17255821, -7.30725113, -0.60715168],
 [-3.05096041, -4.19664316, -6.28545913, -5.14521058],
 [-6.14613752, -2.47655846,  1.11249558,  4.98720933],
 [ 5.901776  , -1.83204293, -2.13293444,  3.19278592],
 [ 4.23462061,  4.53711484, -7.26080094, -3.90687478],
 [ 5.11061035, -4.08503382,  4.50419147, -1.68225486],
 [ 0.94715719, -4.98824461,  1.32635606, -0.8083649 ]])

running=True
cnt = 0
while running:
    for event in pygame.event.get():
        if event.type==QUIT or (event.type==KEYDOWN and event.key==K_ESCAPE):
            running=False
    if cnt==600:
        running=False
    cnt += 1
    screen.fill((20,40,60,0))

    polozaj = np.array([left_hip.angle, right_hip.angle, left_knee.angle, right_knee.angle, left_hip.speed, right_hip.speed, left_knee.speed, right_knee.speed])
     
    M = np.dot(polozaj, W)
    if M[0]>0:
        left_hip.maxMotorTorque = M[0]
        left_hip.motorspeed = 1e3
    else:
        left_hip.maxMotorTorque = -M[0]
        left_hip.motorspeed = -1e3
    if M[1]>0:
        right_hip.maxMotorTorque = M[1]
        right_hip.motorspeed = 1e3
    else:
        right_hip.maxMotorTorque = -M[1]
        right_hip.motorspeed = -1e3
    if M[2]>0:
        left_knee.maxMotorTorque = M[2]
        left_knee.motorspeed = 1e3
    else:
        left_knee.maxMotorTorque = -M[2]
        left_knee.motorspeed = -1e3
    if M[3]>0:
        right_knee.maxMotorTorque = M[3]
        right_knee.motorspeed = 1e3
    else:
        right_knee.maxMotorTorque = -M[3]
        right_knee.motorspeed = -1e3 

   # left_leg.ApplyTorque(500, wake=True)
   # right_leg.ApplyTorque(-200, wake=True)
   # torso.ApplyForce(force=(150,150), point=torso.worldCenter, wake=True)
    
    for body in (ground, torso, left_leg, right_leg, left_underknee, right_underknee):#, left_foot, right_foot):
        for fixture in body.fixtures:
    
            shape=fixture.shape
             
            vertices=[(body.transform*v)*PPM for v in shape.vertices]

            vertices=[(v[0], SCREEN_HEIGHT-v[1]) for v in vertices]

            pygame.draw.polygon(screen, colors[body.type], vertices)
            

    #pygame.draw.line(screen, (127,127,127,255), [ground_body.worldCenter.x * 20 , 600-ground_body.worldCenter.y * 20], [dynamic_body.worldCenter.x * 20 , 600-dynamic_body.worldCenter.y * 20])    
    #print (ground_body.worldCenter.y - dynamic_body.worldCenter.y)/(dynamic_body.worldCenter.x-ground_body.worldCenter.x)
    world.Step(TIME_STEP, 10, 10)

    #kicma tri dela, ruke dva dela, vrat i glava, neuronska, genet, realno vreme

    pygame.display.flip()
    clock.tick(TARGET_FPS) # 

pygame.quit()
