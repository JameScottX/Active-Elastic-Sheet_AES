from controller import *
import numpy as np
import struct

#定义机器人数量
ROBOT_NUM = 16
ROW = 4
COL = 4

class Driver (Supervisor):
    timeStep = 10
   
    def __init__(self):
        super(Driver, self).__init__()
        
        #此处获得世界格机器人定义
        robot = []
        for i in range(ROBOT_NUM):
            robot.append(self.getFromDef('ROBOT%d'%(i+1)))
        #建立发射器 默认id为1
        self.emitter = Emitter('emitter')
        self.emitter.setChannel(1)

        self.locf= []
        self.angf= []
        self.loc = [ np.array([0.,0.]) for i in range(ROBOT_NUM)]
        self.ang = [ 0. for i in range(ROBOT_NUM)]

        #获得机器人状态对象
        for i in range(ROBOT_NUM):
            self.locf.append( robot[i].getField('translation') )  
            self.angf.append( robot[i].getField('rotation'))

        group_ = np.array([i+1 for i in range(ROBOT_NUM)])
        
        self.group = np.reshape(group_,(ROW,COL))

        #单元距离常数项
        self.alph = 5
        #单元角速度常数项
        self.beta = 3
        #车子得前进速度
        self.des_Xv = 2
        #单元正向距离
        self.distance_keep = 0.2


        self.save_data = []
        self.timecount = 0

    def refresh(self):
        '''刷新单元机器人位置和姿态'''
        for i in range(ROBOT_NUM):
            temp = self.locf[i].getSFVec3f()
            self.loc[i] = np.array([-temp[2], -temp[0]])
            self.ang[i] = self.angf[i].getSFRotation()[3]

    def aes_init(self):
        '''设定单元机器人与周围单元链接关系'''
        self.choose_all = []
        self.choose_all2 = []

        #正的四个方向
        for i in range(ROW):
            for j in range(COL):
                left = j-1 
                right = j+1
                up = i-1
                down = i+1
                choose=[]
                choose.append(self.group[i,j])  #添加本地ID
                if left>=0:
                    choose.append(self.group[i,j-1])
                if right<COL:
                    choose.append(self.group[i,j+1])
                if up>=0:
                    choose.append(self.group[i-1,j])
                if down<ROW:
                    choose.append(self.group[i+1,j])

                self.choose_all.append(choose)
        #斜对角方向
        for i in range(ROW):
            for j in range(COL):
                left = j-1 
                right = j+1
                up = i-1
                down = i+1
                choose=[]
                choose.append(self.group[i,j])  #添加本地ID
                if left >=0 and up>=0:
                    choose.append(self.group[i-1,j-1])
                if left >=0 and down<ROW:
                    choose.append(self.group[i+1,j-1])
                if right<COL and up>=0:
                    choose.append(self.group[i-1,j+1])
                if right<COL and down<ROW:
                    choose.append(self.group[i+1,j+1])

                self.choose_all2.append(choose)

        #校验 单元链接关系
        print(self.choose_all)
        print(self.choose_all2)
        # self.choose_all2 = [ [1,5],[2,4,6],[3,5],[4,2,8],[5,1,3,7,9],[6,2,8],[7,5],[8,4,6],[9,5] ]
    
    def aes(self):
        '''AES模型算法'''
        force_all = []
        #先计算正对方向ID的力
        for i in self.choose_all:
            len_ = len(i)
            ori = i[0]-1
            force_add = np.array([0.,0.])
            for j in range(len_):
                if j !=0:
                    #计算目标向量
                    temp2 = self.loc[i[j]-1] - self.loc[ori]
                    #计算向量长度
                    temp = np.linalg.norm(temp2)
                    #叠加每个 f
                    force_add += self.alph * (temp -self.distance_keep) / temp * temp2
            
            force_all.append(force_add)
        force_all = np.array(force_all)

        #计算斜对方向ID的力
        force_all2 = []
        for i in self.choose_all2:
            len_ = len(i)
            ori = i[0]-1
            force_add = np.array([0.,0.])
            for j in range(len_):
                if j !=0:
                    #计算目标向量
                    temp2 = self.loc[i[j]-1] - self.loc[ori]
                    #计算向量长度
                    temp = np.linalg.norm(self.loc[i[j]-1] - self.loc[ori])
                    #叠加每个 f
                    force_add += self.alph * (temp - self.distance_keep * np.sqrt(2)) /temp * temp2
            
            force_all2.append(force_add)

        # print(force_all+force_all2)

        force_all2 = np.array(force_all2)
        force_all= force_all+force_all2  #将正向和斜向力叠加

        #获得单位航向
        ang_v = [ [0.,0.] for i in range(ROBOT_NUM)]
        ang_vp = [ [0.,0.] for i in range(ROBOT_NUM)]
        # print(self.ang)
        for i in range(ROBOT_NUM):
            #航向单位
            ang_v[i][0] = np.cos(self.ang[i])
            ang_v[i][1] = np.sin(self.ang[i])
            #航向单位正交方向
            ang_vp[i][0] = np.cos(self.ang[i]+np.pi/2)
            ang_vp[i][1] = np.sin(self.ang[i]+np.pi/2)

        ang_v = np.array(ang_v)
        ang_vp = np.array(ang_vp)

        #计算期望速度
        Xv = []
        for i in range(ROBOT_NUM):
            Xv.append( np.dot(force_all[i], ang_v[i] )  + self.des_Xv )
        Xv = np.array(Xv)

        #计算期望角速度
        Av = []
        for i in range(ROBOT_NUM):
            Av.append ( self.beta * np.dot(force_all[i], ang_vp[i]) )
        Av = np.array(Av)

        # print(Xv[0])
        return Xv,Av
  
    def send_msg(self,id_, speed,dir_):
        '''根据ID分别发送速度和角速度信息'''
        self.emitter.setChannel(id_)
        message = struct.pack("2f",speed,dir_)
        self.emitter.send(message)
        

    def main(self):
        '''主要函数'''
        self.aes_init()  #初始化单元链接关系
        while self.step(self.timeStep) != -1:

            self.refresh()  #刷新参数
            Xv,Av = self.aes()
            for i in range(ROBOT_NUM):
                # if i ==0:
                #     print(Xv[i],Av[i])
                self.send_msg(i+1, Xv[i],Av[i])

            self.timecount+=1
            self.save_data.append(np.hstack(self.loc))
            if self.timecount == 2* 1000/self.timeStep:  #这两 2s 后 保存数据
                np.savetxt('pos.txt',np.array(self.save_data))
                print('saved!')


                
    
           

controller = Driver()
controller.main()


