"""unit controller."""

from controller import *
import struct

# create the Robot instance.
robot = Robot()
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

id_ = int(robot.getName()[-2])  # 依据机器人名字获得ID

if robot.getName()[-3]!='(':
    id_ = int(robot.getName()[-3:-1])

rev = robot.getReceiver('receiver')  #  初始化接收器
rev.enable(9)
rev.setChannel(id_)   #设置接收器ID 为机器人ID


def limit(val,min,max):
    '''简单的限制函数'''
    if val < min:
        val = min
    elif val > max:
        val = max 
    return val

def setspeed(speed,dir_):
    '''速度设置函数'''

    #简单的线性转换
    w = 1 * dir_
    r_speed_add = w
    l_speed_add = -w

    if id_ == 1:
        # print(speed,dir_)
        pass
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(limit(speed+l_speed_add,-6,6))
    rightMotor.setVelocity(limit(speed+r_speed_add,-6,6))


timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1:
    dataList = [0.,0.]

    if rev.getQueueLength() >0:  #判断数据收到队列 是否围殴0 
        message=rev.getData()   #获取数据
        dataList=struct.unpack("2f",message) # 转换
        rev.nextPacket()    #这里直观重要
    
        # if id_==1:
        #     print(dataList)

    setspeed(dataList[0],dataList[1])
    pass

