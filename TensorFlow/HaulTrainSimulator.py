
# coding: utf-8

# # 货运列车仿真器

# In[1]:


import numpy as np
import tensorflow as tf
from scipy.interpolate import interp1d
import time
import matplotlib.pyplot as plt


# ## 定义主类

# In[2]:


class TrainSimulator:
    '货运列车运行仿真器（并行探索）'
    
    # hHpeed_Limits = np.array([85, 65, 80])/3.6
    # speed_ChangePoint = np.array([0.4, 0.5])
    # TotalLength = 8e3
    def __init__(self, 
                 Ne, 
                 actionT, 
                 hHpeed_Limits, 
                 speed_ChangePoint, 
                 TotalLength, 
                 rampList,
                 percentage_posRange,
                 speed_pusnishement = -5,
                 is_enable_tensorflow = False,
                ): 
        self.Ne = Ne
        self.dt = 0.1    # 单次积分时间步长，列车状态每 dt 秒 更新一次
        self.nDt = int(np.round(actionT/self.dt))
        self.actionT = actionT     # 每 actionT 做一次操作
        
        self.TotalLength = TotalLength
        self.rampList = rampList
        self.initial_posRange = TotalLength*percentage_posRange    # 初始位置种子范围
        self.hHpeed_Limits = hHpeed_Limits
        self.lowSpeedLim = 15/3.6
        self.speed_ChangePoint = self.TotalLength*speed_ChangePoint
        
        self.is_enable_tensorflow = is_enable_tensorflow
        self.speed_pusnishement = speed_pusnishement
        
        self.Nl = int(2)
        self.Nw = int(200)
        self.Nt = self.Nl + self.Nw
        self.maxStep = int(1e3)
        
        Ad, Bd, TBcl, LenTrains, mTrainGroup, C0, Ca, KK, DD = initialize_locomotive_char(Ne)
        self.Ad = Ad
        self.Bd = Bd
        self.TBcl = TBcl
        self.LenTrains = LenTrains
        self.mTrainGroup = mTrainGroup
        self.C0 = C0
        self.Ca = Ca
        self.KK = KK
        self.DD = DD
        self.len_train = self.LenTrains[-1, 0]
        
        Nt = self.Nt
        self.X = np.matrix(np.zeros([Nt*2, Ne], dtype = float))
        self.U = np.matrix(np.zeros([Nt, Ne], dtype = float)) 
        self.countStepNe = np.zeros(Ne, dtype = int)
        self.bool_terminate = np.zeros(Ne, dtype = bool)
        self.bool_restart = np.zeros(Ne, dtype = bool)
        self.phase_buffer = np.zeros([self.maxStep, 2, Ne])*np.nan   # 记录轨迹
        
        self.reward = np.zeros([Ne, 1], dtype = float)
        
        self.tmp_Fl = np.zeros([2, Ne], dtype = float)
        self.R_tmp = np.zeros([2, Ne])
        self.F_c = np.zeros([self.nDt, Ne])
        self.F_l = np.zeros([self.nDt, Ne])
        
        self.carType = np.zeros(Nt, dtype = bool)
        self.carType[[0, -1]] = True
        
        self.MotorNotch = np.zeros([2, Ne], dtype = int)
        self.MotorNotch_ = self.MotorNotch.copy()
        
        init_bool = np.ones([1, Ne], dtype = bool) 
        self.init_simulation(init_bool)    # 初始化所有列车

        nSlim = self.speed_ChangePoint.size
        ind1122 = np.tile(np.arange(nSlim, dtype = int), (2, 1))
        ind112233 = np.tile(np.arange(nSlim+1, dtype = int), (2, 1))
        tmp = speed_ChangePoint[ind1122]*TotalLength
        tmp = np.concatenate((tmp, np.zeros([1, tmp.shape[1]])*np.nan), axis = 0)
        tmp_ = np.transpose(tmp[[0, 2, 1], :])
        tmp_ = np.reshape(tmp_, tmp_.size)
        self.x_high = np.append(np.append(0, tmp_), self.TotalLength) 
        tmp = self.hHpeed_Limits[ind112233]*3.6
        tmp = np.concatenate((tmp, np.zeros([1, tmp.shape[1]])*np.nan), axis = 0)
        y_high = np.transpose(tmp).reshape(tmp.size)
        self.y_high = np.delete(y_high, -1)

        self.x_low = np.array([0, self.TotalLength], dtype = float)
        self.y_low = np.array([self.lowSpeedLim, self.lowSpeedLim], dtype = float)*3.6
        
        if self.is_enable_tensorflow:   # 激活使用 tensorflow 加速矩阵运算
            self.sess = tf.Session()
            self.phX = tf.placeholder(tf.float32, [Nt*2, Ne], name='X')  # input X
            self.phU = tf.placeholder(tf.float32, [Nt, Ne], name='U')    # input U
            self.tf_Ad = tf.constant(self.Ad, dtype = tf.float32, name='Ad')
            self.tf_Bd = tf.constant(self.Bd, dtype = tf.float32, name='Bd')
            self.update_op = tf.matmul(self.tf_Ad, self.phX) + tf.matmul(self.tf_Bd, self.phU)

    def step(self, input_actions):
        
        self.MotorNotch_ = self.MotorNotch    # 更新 操作方式，并保存过去的记录
        self.MotorNotch = input_actions
        
        Nt = self.Nt
        
        self.R_tmp[:] = 0 
        for itr in range(self.nDt):
            # ========================================== 计算三个力（暂不考虑空气自动力）
            self.U[:] = 0 
            
            # ------------------------ 机车牵引力
            self.tmp_Fl = GetLocomotiveF(self.X[[0, Nt-1]]*3.6, self.MotorNotch, self.TBcl)
            #tmp_Fl *= (1 + np.random.rand(tmp_Fl.shape[0],tmp_Fl.shape[1])*0.02-0.01) # add noise
            self.U[self.carType, :] = self.tmp_Fl * 1e3
            
            # ------------------------ 附加阻力阻力
            pTrains = self.LenTrains + self.X[Nt:]
            addForce, rempTrains = GetAdditionalF(self.mTrainGroup, pTrains, self.rampList)
            self.U += addForce
            
            # ------------------------ 附加阻力阻力
            basicForce = GetBasicF(self.mTrainGroup, self.X[:Nt], self.C0, self.Ca)
            basicForce[self.X[:Nt] <= 0]= 0
            self.U = self.U + basicForce 
            
            # ========================================== 更新状态方程
            
            if not self.is_enable_tensorflow:   # 激活使用 tensorflow 加速矩阵运算
                self.X = self.UpdateStateEquationNp(self.X, self.U, self.Ad, self.Bd)
            else:
                self.X = self.UpdateStateEquationTf(self.X, self.U)
            
            # ========================================== 记录状态变量
            
            F_c_ = np.multiply(self.KK, (self.X[Nt:-1]-self.X[Nt+1:])) + np.multiply(self.DD, (self.X[:Nt-1]-self.X[1:Nt]))
            
            self.F_c[itr] = np.max(np.abs(F_c_), axis = 0)
            self.F_l[itr] = np.sum(self.tmp_Fl*self.tmp_Fl, axis = 0)
            
        # ========================================== 记录状态，并判断是否有终止发生
        
        self.countStepNe += 1      # 标识为多走了一步
        for i in range(self.Ne):
            self.phase_buffer[self.countStepNe[i], :, i] = self.X[[Nt, 0], i].reshape(2)
        
        self.R_tmp[0] = np.sum(self.F_c, axis = 0)
        self.R_tmp[1] = np.sum(self.F_l, axis = 0)
                
        train_pos = self.X[[Nt, -1]].getA()
        train_pos[1, :] += self.len_train
        Lspeed = obtain_speed_limit(train_pos, self.speed_ChangePoint, self.hHpeed_Limits) 
        
        # 每走一步均有一个成本 
        current_speed = self.X[0, :].getA()
        self.reward[:] = (current_speed.reshape([self.Ne, 1])-33) * 0.3
        
        self.reward += (self.higSpeed_r(current_speed - np.min(Lspeed, axis = 0)) +                         self.lowSpeed_r(current_speed - self.lowSpeedLim)).reshape([self.Ne, 1])
        
        bool_terminate = self.X[Nt, :].getA().reshape(self.Ne) >= self.TotalLength
        
        # 正常终末状态标记为正数
        self.reward[bool_terminate] *= -1
        
        # if np.any(bool_terminate):
        bool_restart = (bool_terminate + (self.countStepNe >= self.maxStep) 
                        + (current_speed < self.lowSpeedLim - 4) 
                        + (current_speed > np.min(Lspeed, axis = 0) + 4))
        
        state = self.get_observation()
        
        return state, self.reward, bool_restart
    
    # 返回第一个机车的速度和位移
    def get_observation(self):
        state = np.transpose(self.X[[self.Nt, 0], :].getA())
        state[:, 1] *= 3.6
        return state
    
    def get_phase_buffer(self):
        return self.phase_buffer
    
    def set_initial_pos_par(self, percentage_):
        self.initial_posRange = self.TotalLength*percentage_
            
    def init_simulation(self, bool_start):
        Nt = self.Nt
        inds = np.argwhere(np.squeeze(bool_start))
        inds = np.squeeze(inds)
        tmp_ind = np.sum(bool_start)
        # print(inds)
        
        initial_position = np.random.rand(tmp_ind)*self.initial_posRange
        
        for i in range(tmp_ind):          # 一定概率从0开始
            if np.random.rand() < 0.2:
                initial_position[i] /= 10
        
        if tmp_ind == 1:
            self.X[Nt:, inds] = initial_position
        else:
            for i in range(tmp_ind):
                self.X[Nt:, inds[i]] = initial_position[i]
        
        train_pos = self.X[[Nt, -1]].getA()
        train_pos = train_pos[:, inds]
        if inds.size == 1:
            train_pos = train_pos.reshape([2, 1])
        train_pos[1, :] += self.len_train
        Lspeed_ = obtain_speed_limit(train_pos, self.speed_ChangePoint, self.hHpeed_Limits)
        
        initial_speed = np.random.rand(tmp_ind)*(np.min(Lspeed_, axis = 0) - self.lowSpeedLim) + self.lowSpeedLim
        
        # tmp = np.mat(initial_speed).reshape(1, tmp_ind)
        if tmp_ind == 1:
            self.X[:Nt, inds] = initial_speed
        else:
            for i in range(tmp_ind):
                self.X[:Nt, inds[i]] = initial_speed[i]
        
        initial_notch = np.round((initial_speed*3.6)/10).astype(int)
        initial_notch[initial_notch > 12] = 12
        initial_notch[initial_notch < -12] = -12
        
        if tmp_ind == 1:
            self.MotorNotch[:, inds] = initial_notch      # 初始级位
        else:
            for i in range(tmp_ind):
                self.MotorNotch[:, inds[i]] = initial_notch[i]      # 初始级位
        
        self.MotorNotch_[:, inds] = self.MotorNotch[:, inds]
        
        self.countStepNe[inds] = 0      # 重置为1 
        self.phase_buffer[:, :, inds] = np.nan
        
        if tmp_ind > 1:
            for i in range(tmp_ind):
                self.phase_buffer[self.countStepNe[inds[i]], :, inds[i]] = self.X[[Nt, 0], inds[i]].reshape(2)
        else:
            self.phase_buffer[self.countStepNe[inds], :, inds] = self.X[[Nt, 0], inds].reshape(2)
        
    def show_phase_figure(self):
        fig = plt.figure(1)
        fig.clf()
        fig.set_size_inches(16, 5)

        for i in range(self.phase_buffer.shape[2]):
            plt.plot(self.phase_buffer[:, 0, i], self.phase_buffer[:, 1, i]*3.6, '.-')

            plt.plot(self.x_high, self.y_high, 'k', linewidth = 5)
            plt.plot(self.x_low, self.y_low, 'k', linewidth = 5)
        
        plt.xlim(self.x_low)
        plt.ylim([0, 120])
        plt.xlabel('Mileage (km)', fontsize = 20)
        plt.ylabel('Speed (km/h)', fontsize = 20)
        plt.show()
        
    def UpdateStateEquationNp(self, X, U, Ad, Bd):
        return Ad*X + Bd*U
        
    def UpdateStateEquationTf(self, X, U):
        return np.asmatrix(self.sess.run(self.update_op, 
                             feed_dict = {self.phX: X, self.phU: U}))
    
    def higSpeed_r(self, dv): 
        r = -0.05*np.power((np.abs(dv) + 5), 3) - 10
        r[dv < 0] = 0
        #return r
        #r = np.zeros(dv.shape, float)
        #r[dv > 0] = self.speed_pusnishement * 0.2
        return r

    def lowSpeed_r(self, dv):
        r = -0.05*np.power((np.abs(dv) + 5), 3) - 10
        r[dv > 0] = 0
        #return r
        #r = np.zeros(dv.shape, float)
        #r[dv < 0] = self.speed_pusnishement
        return r


# In[3]:



    
def obtain_speed_limit(v, speed_ChangePoint, hHpeed_Limits): 
    Lspeed = np.zeros(v.shape)
    itr = 0
    bo_tmp = v < speed_ChangePoint[itr]
    Lspeed[bo_tmp] = hHpeed_Limits[itr]
    for itr in range(1, speed_ChangePoint.size):
        bo_tmp = np.logical_and(v >= speed_ChangePoint[itr-1], v < speed_ChangePoint[itr])
        Lspeed[bo_tmp] = hHpeed_Limits[itr]
        
    bo_tmp = v >= speed_ChangePoint[itr]
    Lspeed[bo_tmp] = hHpeed_Limits[itr+1]
    
    return Lspeed


# In[4]:



def initialize_locomotive_char(Ne):
    # 初始化一些重要参数
    # -------------- 获取车辆牵引特性 
    TBcl = np.zeros([25, 12, 3])
    MatrixTBCL_force_tmp = np.loadtxt(open('./static/MatrixTBCL_force_0.csv',"rb"), delimiter=",")
    TBcl[:,:,0] = MatrixTBCL_force_tmp.copy()
    MatrixTBCL_force_tmp = np.loadtxt(open('./static/MatrixTBCL_force_1.csv',"rb"), delimiter=",")
    TBcl[:,:,1] = MatrixTBCL_force_tmp.copy()
    MatrixTBCL_force_tmp = np.loadtxt(open('./static/MatrixTBCL_force_2.csv',"rb"), delimiter=",")
    TBcl[:,:,2] = MatrixTBCL_force_tmp.copy()
    
    # -------------- 读入动力学方程离散化矩阵
    Ad = np.matrix(np.loadtxt(open('./static/Ad.csv',"rb"), delimiter=","))
    Bd = np.matrix(np.loadtxt(open('./static/Bd.csv',"rb"), delimiter=","))
    
    # -------------- 读入基本力，车沟力,基本力 计算参数
    locmotive_info = np.loadtxt(open('./static/locmotive_info.csv',"rb"), delimiter=",", skiprows = 1)
    basicF_par = np.loadtxt(open('./static/basicF_par.csv',"rb"), delimiter=",", skiprows = 1)
    coupler_par = np.loadtxt(open('./static/coupler_par.csv',"rb"), delimiter=",", skiprows = 1)
    # Ne = 10
    mTrainGroup = np.matrix(locmotive_info[:,0]).T*np.matrix(np.ones(Ne))
    LenTrains = np.matrix(locmotive_info[:,1]).T*np.matrix(np.ones(Ne))
    
    C0 = np.matrix(basicF_par[:,0]).T*np.matrix(np.ones(Ne))
    Ca = np.matrix(basicF_par[:,1]).T*np.matrix(np.ones(Ne))
    
    KK = np.matrix(coupler_par[:,0]).T*np.matrix(np.ones(Ne))
    DD = np.matrix(coupler_par[:,1]).T*np.matrix(np.ones(Ne))
    
    return Ad, Bd, TBcl, LenTrains, mTrainGroup, C0, Ca, KK, DD


# In[6]:


def GetBasicF(mTrainGroup, TrainVelocity, C0, Ca):
    # 基本阻力
    F = -np.multiply(mTrainGroup, C0)
    F[0] += -np.multiply(np.multiply(Ca[0], np.sum(mTrainGroup, axis = 0)),          np.power(TrainVelocity[0], 2))
    return F


# In[7]:


def GetAdditionalF(mTrainGroup, pTrains, rampList):
    # 附加阻力
    rempTrains = np.zeros(pTrains.shape)
    mm = [np.min(pTrains), np.max(pTrains)]
    
    indmm = np.zeros(2, dtype = int)
    indmm[0] = np.argmin(np.abs(rampList[:, 0] - mm[0])) - 1
    indmm[1] = np.argmin(np.abs(rampList[:, 0] - mm[1])) + 1
    
    indmm[indmm < 0] = 0
    
    for ptr in range(indmm[0], indmm[1]+1):        
        bo = (pTrains >= rampList[ptr,0]).getA() & (pTrains < rampList[ptr + 1,0]).getA()
        rempTrains[bo] = rampList[ptr, 1]
    
    addForce = -mTrainGroup.getA()*9.8*rempTrains
    
    return addForce, rempTrains


# In[8]:


def GetLocomotiveF(V, MotorNotch, TBcl):
    # 机车牵引力
    sp = V.shape
    n = V.size
    
    V_ = V.getA().reshape([n, 1])
    MotorNotch_ = MotorNotch.reshape(n) + 12
    
    speed_pos = np.sum(V_ - TBcl[MotorNotch_, :, 0] >= 0, axis = 1) - 1
    
    x = TBcl[MotorNotch_, speed_pos, 0]
    y = TBcl[MotorNotch_, speed_pos, 1]
    dy = TBcl[MotorNotch_, speed_pos, 2]
    
    Fvec = y + dy*(V_.T-x)
    
    return Fvec.reshape(sp)


# In[ ]:






