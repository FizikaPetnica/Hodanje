import pygame
from pygame.locals import *

import Box2D
from Box2D.b2 import *

import math

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
box=ground.CreatePolygonFixture(box=(30,3), density=1)
box.filterData.categoryBits = 1
box.filterData.maskBits = 14

torso=world.CreateDynamicBody(position=(5,9), angle=0)
box=torso.CreatePolygonFixture(box=(0.1,1), density=1)
box.filterData.categoryBits = 2
box.filterData.maskBits = 13

ang1 = 0
l1 = 5+0.7*math.sin(ang1)
l2 = 8-0.7*math.cos(ang1)
left_leg=world.CreateDynamicBody(position=(l1,l2), angle = ang1)
box=left_leg.CreatePolygonFixture(box=(0.1,0.7), density=1)
box.filterData.categoryBits = 4
box.filterData.maskBits = 7

ang2 = 0
l1 = 5+0.7*math.sin(ang2)
l2 = 8-0.7*math.cos(ang2)
right_leg=world.CreateDynamicBody(position=(l1,l2), angle = ang2)
box=right_leg.CreatePolygonFixture(box=(0.1,0.7), density=1)
box.filterData.categoryBits = 8
box.filterData.maskBits = 11

ang3 = 0
l1 = 5+1.4*math.sin(ang1)+0.7*math.sin(ang3)
l2 = 8-1.4*math.cos(ang1)-0.7*math.cos(ang3)
left_underknee=world.CreateDynamicBody(position=(l1,l2), angle = ang3)
box=left_underknee.CreatePolygonFixture(box=(0.1,0.7), density=1)
box.filterData.categoryBits = 4
box.filterData.maskBits = 7

ang4 = 0
l1 = 5+1.4*math.sin(ang2)+0.7*math.sin(ang4)
l2 = 8-1.4*math.cos(ang2)-0.7*math.cos(ang4)
right_underknee=world.CreateDynamicBody(position=(l1,l2), angle = ang4)
box=right_underknee.CreatePolygonFixture(box=(0.1,0.7), density=1)
box.filterData.categoryBits = 8
box.filterData.maskBits = 11

l1 = 5+1.4*math.sin(ang1)+1.4*math.sin(ang3)-0.1*math.cos(ang3)+0.36
l2 = 8-1.4*math.cos(ang1)-1.4*math.cos(ang3)
left_foot=world.CreateDynamicBody(position=(l1,l2), angle=0)
box=left_foot.CreatePolygonFixture(box=(0.36,0.1), density=1)
box.filterData.categoryBits = 4
box.filterData.maskBits = 7

l1 = 5+1.4*math.sin(ang2)+1.4*math.sin(ang4)-0.1*math.cos(ang4)+0.36
l2 = 8-1.4*math.cos(ang2)-1.4*math.cos(ang4)
right_foot=world.CreateDynamicBody(position=(l1,l2), angle=0)
box=right_foot.CreatePolygonFixture(box=(0.36,0.1), density=1)
box.filterData.categoryBits = 8
box.filterData.maskBits = 11

#centar.ApplyLinearImpulse(impulse=(-100,0), point=right_underknee.worldCenter, wake = True)
#centar.ApplyLinearImpulse(impulse=(-100,0), point=left_underknee.worldCenter, wake = True)
#world.CreateDistanceJoint(bodyA=ground_body, bodyB=dynamic_body, anchorA=ground_body.worldCenter, anchorB=dynamic_body.worldCenter)

left_hip=world.CreateRevoluteJoint(
    bodyA=torso, 
    bodyB=left_leg, 
    anchor=(5,8),
    lowerAngle = -ang1-pi*0.05,
    upperAngle = pi-ang1-pi*0.05,
    enableLimit = True,
    maxMotorTorque = 10.0,
    motorSpeed = -10.0,
    enableMotor = False,
    )

right_hip=world.CreateRevoluteJoint(
    bodyA=torso, 
    bodyB=right_leg, 
    anchor=(5,8),
    lowerAngle = -ang2-pi*0.05,
    upperAngle = pi-ang2-pi*0.05,
    enableLimit = True,
    maxMotorTorque = 10.0,
    motorSpeed = 10.0,
    enableMotor = False,
    )

left_knee=world.CreateRevoluteJoint(
    bodyA=left_leg, 
    bodyB=left_underknee, 
    anchor=(5+1.4*math.sin(ang1),8-1.4*math.cos(ang1)),
    lowerAngle = ang1-ang3-pi+0.05*pi,
    upperAngle = ang1-ang3,
    enableLimit = True,
    maxMotorTorque = 0.0,
    motorSpeed = 0.0,
    enableMotor = False,
    )

right_knee=world.CreateRevoluteJoint(
    bodyA=right_leg, 
    bodyB=right_underknee, 
    anchor=(5+1.4*math.sin(ang2),8-1.4*math.cos(ang2)),
    lowerAngle = ang2-ang4-pi+0.05*pi,
    upperAngle = ang2-ang4,
    enableLimit = True,
    maxMotorTorque = 0.0,
    motorSpeed = 0.0,
    enableMotor = False,
    )

left_ankle=world.CreateRevoluteJoint(
    bodyA=left_underknee, 
    bodyB=left_foot, 
    anchor=(5+1.4*math.sin(ang1)+1.4*math.sin(ang3),8-1.4*math.cos(ang1)-1.4*math.cos(ang3)),
    lowerAngle = ang3-0.5*pi+0.05*pi,
    upperAngle = ang3+0.5*pi-1,
    enableLimit = True,
    maxMotorTorque = 0.0,
    motorSpeed = 0.0,
    enableMotor = False,
    )

right_ankle=world.CreateRevoluteJoint(
    bodyA=right_underknee, 
    bodyB=right_foot, 
    anchor=(5+1.4*math.sin(ang2)+1.4*math.sin(ang4),8-1.4*math.cos(ang2)-1.4*math.cos(ang4)),
    lowerAngle = ang4-0.5*pi+0.05*pi,
    upperAngle = ang4+0.5*pi-1,
    enableLimit = True,
    maxMotorTorque = 0.0,
    motorSpeed = 0.0,
    enableMotor = False,
    )

running=True
while running:
    for event in pygame.event.get():
        if event.type==QUIT or (event.type==KEYDOWN and event.key==K_ESCAPE):
            running=False

    screen.fill((20,40,60,0))

   # ground_body.ApplyTorque(10, wake=True)
   # dynamic_body.ApplyTorque(-50, wake=True)
   # dynamic_body.ApplyForce(force=(50, 50), point=dynamic_body.worldCenter, wake=True)
    
    for body in (ground, torso, left_leg, right_leg, left_underknee, right_underknee, left_foot, right_foot):
        for fixture in body.fixtures:
    
            shape=fixture.shape
             
            vertices=[(body.transform*v)*PPM for v in shape.vertices]

            vertices=[(v[0], SCREEN_HEIGHT-v[1]) for v in vertices]

            pygame.draw.polygon(screen, colors[body.type], vertices)

    #pygame.draw.line(screen, (127,127,127,255), [ground_body.worldCenter.x * 20 , 600-ground_body.worldCenter.y * 20], [dynamic_body.worldCenter.x * 20 , 600-dynamic_body.worldCenter.y * 20])    
    #print (ground_body.worldCenter.y - dynamic_body.worldCenter.y)/(dynamic_body.worldCenter.x-ground_body.worldCenter.x)
    world.Step(TIME_STEP, 10, 10)

    #da ne visi, kicma tri dela, ruke dva dela, vrat i glava

    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
