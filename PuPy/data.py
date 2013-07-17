'''
Created on Dec 8, 2012

@author: nuci
'''

import numpy as np
import pylab as pl
import h5py

_s_accel = 'accelerometer'
_s_gyro = 'gyro'
_s_compass = 'compass'
_s_gps = 'puppyGPS'

_s_target = ('trg0', 'trg1', 'trg2', 'trg3')
_s_hip = ('hip0', 'hip1', 'hip2', 'hip3')
_s_knee = ('knee0', 'knee1', 'knee2', 'knee3')
_s_touch = ('touch0', 'touch1', 'touch2', 'touch3')

datasets = ['trg0', 'trg1', 'trg2', 'trg3',
            'hip0', 'hip1', 'hip2', 'hip3',
            'knee0', 'knee1', 'knee2', 'knee3',
            'touch0', 'touch1', 'touch2', 'touch3',
            'accelerometer_x', 'accelerometer_y', 'accelerometer_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'compass_x', 'compass_y', 'compass_z',
            'puppyGPS_x', 'puppyGPS_y', 'puppyGPS_z',
            ]
# FL: front left, FR: front right, HL: hind left, HR: hind right
# X: front-back axis, Y: left-right axis, Y: top-bottom axis
idx_signals = {'M_FL':0, 'M_FR':1, 'M_HL':2, 'M_HR':3, # Motor target angles in rad
               'H_FL':4, 'H_FR':5, 'H_HL':6, 'H_HR':7, # hip joint angles in rad
               'K_FL':8, 'K_FR':9, 'K_HL':10, 'K_HR':11, # knee joint angles in rad
               'T_FL':12, 'T_FR':13, 'T_HL':14, 'T_HR':15, # Touch sensors at the feet some pressure unit
               'A_X':16, 'A_Y':17, 'A_Z':18, # acceleration sensors in m/s
               'G_X':19, 'G_Y':20, 'G_Z':21, # gyroscope (maybe in rad/s?)
               'M_X':22, 'M_Y':23, 'M_Z':24, # magnetometer
               'GPS_X':25, 'GPS_Y':26, 'GPS_Z':27, # Global Positioning Sensor in world coordinated
               'ground':28, # index of the current terrain Puppy's center of mass is at (numbering defined in individual experiment)
               'reset':29,
               'tumble':30, # 1 where tumbling is detected, 0 otherwise
               'gait':31} # 1 where Puppy was respawned at a new location, 0 otherwise

idx_motor = [idx_signals[k] for k in ['M_FL', 'M_FR', 'M_HL', 'M_HR']]
idx_sensor = [idx_signals[k] for k in ['H_FL', 'H_FR', 'H_HL', 'H_HR',
                                       'K_FL', 'K_FR', 'K_HL', 'K_HR',
                                       'T_FL', 'T_FR', 'T_HL', 'T_HR',
                                       'A_X', 'A_Y', 'A_Z',
                                       'G_X', 'G_Y', 'G_Z',
                                       'M_X', 'M_Y', 'M_Z',
                                       'GPS_X', 'GPS_Y', 'GPS_Z']]
idx_gps = [idx_signals[k] for k in ['GPS_X', 'GPS_Y', 'GPS_Z',]]
idx_terrain = idx_signals['ground']
idx_reset = idx_signals['reset']
idx_tumble = idx_signals['tumble']
idx_gait = idx_signals['gait']

idx_modalities = [[0,1,2,3], [4,5,6,7], [8,9,10,11], [12,13,14,15],
                  [16,17,18], [19,20,21], [22,23,24], [25,26,27]]

names_modalities = ['Motor commands', 'Hip angles', 'Knee angles', 'Touch pressure', 'Acceleration', 'Gyroscope', 'Compass', 'Position']
units_modalities = ['rad', 'rad', 'rad', 'N', 'm/s^2', 'rad/s', 'north vector', 'm']

class PuppyDataSim():
    '''
    This class contains the data from Puppy experiments including relevant meta information.
    '''

    def __init__(self, path='', fs=50., resampling=1, min_chunk_size=1000, 
                  remove_tumbling=True, pre_delete_tumble=20, post_delete_tumble=20,
                  remove_reset=True, pre_delete_reset=0, post_delete_reset=20):
        '''
        Load data from path and clean it from artifacts as desired.
        '''
        # copy parameters:
        self.fs = fs
        self.path = path
        self.min_chunk_size = min_chunk_size
        self._resampling = resampling
        self._remove_tumbling = remove_tumbling
        self._remove_reset = remove_reset
        self._pre_delete_tumble = pre_delete_tumble
        self._post_delete_tumble = post_delete_tumble
        self._pre_delete_reset = pre_delete_reset
        self._post_delete_reset = post_delete_reset
        self._x = np.array([])
        self.i_chunk = np.array([])
        self.shape = (0,)
        
        if path:
            self.loadData()
    
    def loadData(self):
        # load the data:
        self._x = read_sim_self_3_with_gait_index(self.path)
        self._x = np.concatenate([d for d in self._x], axis=0)
        self.nSamples = self[:].shape[0]
        self.t = np.arange(self.nSamples, dtype=int)
        
        # find tumbles and resets:
        idx_keep = np.ones(self.nSamples, dtype=bool)
        if self._remove_tumbling:
            idx_keep = np.bitwise_and(idx_keep, remove_artifacts(self.tumble, None, self._pre_delete_tumble, self._post_delete_tumble))
        if self._remove_reset:
            idx_keep = np.bitwise_and(idx_keep, remove_artifacts(self.reset, None, self._pre_delete_reset, self._post_delete_reset))
        
        # check for chunk size:
        i_start = pl.find(np.diff(idx_keep.astype(int))>0)+1
        i_end = pl.find(np.diff(idx_keep.astype(int))<0)+1
        if len(i_start)!=0 and len(i_end)!=0:
            if i_start[0]>i_end[0]:
                i_start = np.concatenate([[0], i_start])
            if i_start[-1]>i_end[-1]:
                i_end = np.concatenate([i_end, [self.nSamples]])
            for start, end in zip(i_start,i_end):
                if end-start<=self.min_chunk_size:
                    idx_keep[start:end] = False
                
        # subsample:    
        self._x = self._x[::self._resampling]
        self.t = self.t[::self._resampling]
        idx_keep = idx_keep[::self._resampling]
        self.fs /= self._resampling
        
        # find chunks:
        self.i_chunk = np.zeros(self.nSamples, dtype=bool)
        self.i_chunk[pl.find(np.diff(idx_keep.astype(int))>0)+1] = True
        self.i_chunk[0] = idx_keep[0]
        
        # clean from tumbles, reset, and too short chunks:
        self._x = self._x[idx_keep]
        self.t = self.t[idx_keep]
        self.i_chunk = self.i_chunk[idx_keep]
        self.i_chunk = pl.find(self.i_chunk)
        self.nChunks = len(self.i_chunk)
        self.nSamples = idx_keep.sum()
        self.shape = (self.nSamples,len(idx_motor+idx_sensor))
    
    def copy(self, idx=None):
        '''
        deep copy of the data object.
        '''
        cpy = PuppyDataSim(path='',
                           fs=self.fs,
                           resampling=self._resampling,
                           min_chunk_size=self.min_chunk_size, 
                           remove_tumbling=self._remove_tumbling,
                           pre_delete_tumble=self._pre_delete_tumble,
                           post_delete_tumble=self._post_delete_tumble,
                           remove_reset=self._remove_reset,
                           pre_delete_reset=self._pre_delete_reset,
                           post_delete_reset=self._post_delete_reset)
        # copy data:
        cpy.path = self.path
        if idx is None:
            cpy._x = self._x.copy()
            cpy.i_chunk = self.i_chunk.copy()
        else:
            if isinstance(idx, int):
                idx = np.arange(idx)
            cpy._x = self._x[idx]
            cpy.t = self.t[idx]
            # this works only if idx is monotonically increasing with step size 1!
            cpy.i_chunk = np.unique(np.hstack([idx[:1], self.i_chunk[np.bitwise_and(self.i_chunk>=idx[0],self.i_chunk<=idx[-1])]]))-idx[0]
#            cpy.i_chunk = np.concatenate([[0], self.i_chunk[1:][np.bitwise_and(self.i_chunk[1:]>=idx[0], self.i_chunk[1:]<idx[-1])]-idx[0]])                
        cpy.nSamples = cpy._x.shape[0]
        cpy.nChunks = len(cpy.i_chunk)
        cpy.shape = cpy._x.shape
        return cpy
    
    def __getitem__(self, idx):
        return self._x[idx] # like this it is compatible with array representation of the data
    
    def __setitem__(self, idx, val):
        self._x[idx] = val # like this it is compatible with array representation of the data
    
    def __iter__(self):
        self._current_iter = -1
        return self
    
    def next(self):
        self._current_iter += 1
        if self._current_iter >= self.nSamples:
            raise StopIteration
        else:
            return self._x[self._current_iter]
    
    # !depricated: dat.x[idx] is much slower than dat[idx]
    def _get_x(self):
        return self._x[:,idx_motor+idx_sensor]
    def _set_x(self, value):
        self._x[:,idx_motor+idx_sensor] = value
    x = property(_get_x, _set_x, doc="sensorimotor data of the robot")
    
    def _get_gait(self):
        return self._x[:,idx_gait]
    def _set_gait(self, value):
        self._x[:,idx_gait] = value
    gait = property(_get_gait, _set_gait, doc="Gait number as indicated by the 'index' field of the used gait")
    
    def _get_ground(self):
        return self._x[:,idx_terrain]
    def _set_ground(self, value):
        self._x[:,idx_terrain] = value
    ground = property(_get_ground, _set_ground, doc="Ground number as indicated by the terrain info file (zero by default)")
    
    def _get_gps(self):
        return self._x[:,idx_gps]
    def _set_gps(self, value):
        self._x[:,idx_gps] = value
    gps = property(_get_gps, _set_gps, doc="Current position of the robot from global positioning sensor (_x,y,z)")
    
    def _get_tumble(self):
        return self._x[:,idx_tumble]
    def _set_tumble(self, value):
        self._x[:,idx_tumble] = value
    tumble = property(_get_tumble, _set_tumble, doc="has a 1.0 where Puppy tumbled and 0.0 elsewhere")
    
    def _get_reset(self):
        return self._x[:,idx_reset]
    def _set_reset(self, value):
        self._x[:,idx_reset] = value
    reset = property(_get_reset, _set_reset, doc="has a 1.0 where Puppy was reset to a new position and 0.0 elsewhere")
    
    def __str__(self):
        return "<PuppyDataSim from folder '%s' with %d samples and %d chunks>" % (self.path, self.nSamples, self.nChunks)
    
    def __repr__(self):
        return "PuppyDataSim(path='%s',fs=%.f,resampling=%d,min_chunk_size=%d," % \
                             (self.path, self.fs, self._resampling, self.min_chunk_size) + \
                             "remove_tumbling=%r,pre_delete_tumble=%d,post_delete_tumble=%d," % \
                             (self._remove_tumbling, self._pre_delete_tumble, self._post_delete_tumble) + \
                             "remove_reset=%r,pre_delete_reset=%d,post_delete_reset=%d)" % \
                              (self._remove_reset, self._pre_delete_reset, self._post_delete_reset)
    
    


def remove_artifacts(i_artifact, data=None, pre_remove=0, post_remove=0):
    '''
    look into index data and find indices to keep. 
    '''
    nSamples = i_artifact.size
    i_artifact = pl.find(i_artifact)
    idx_keep = np.ones(nSamples, dtype=bool)
    for i in range(len(i_artifact)):
        idx_keep[max(0,i_artifact[i]-pre_remove):min(nSamples,i_artifact[i]+1+post_remove)] = False
    
    if data==None:
        return idx_keep
    else:
        return data[idx_keep]



def cleanData(idx_tumble, idx_reset, pre_delete_tumble=20, post_delete_tumble=20, pre_delete_reset=0, post_delete_reset=20, min_chunk_size=1000, plot=False):
    '''
    remove regions where Puppy tumbled and where it was restarted (burn-in).
    remove also too short regions.
    '''
    idx_keep_t = remove_artifacts(idx_tumble, None, pre_delete_tumble, post_delete_tumble)
    idx_keep_r = remove_artifacts(idx_reset, None, pre_delete_reset, post_delete_reset)
    idx_keep = np.bitwise_and(idx_keep_t, idx_keep_r)
    on = pl.find(np.diff(idx_keep.astype(int))>0)+1
    off = pl.find(np.diff(idx_keep.astype(int))<0)+1
    if on[0]>off[0]:
        on = np.concatenate([[0], on])
    if on[-1]>off[-1]:
        off = np.concatenate([off, [len(idx_tumble)]])
    
    for start, end in zip(on,off):
        if end-start<=min_chunk_size:
            #print start,end
            idx_keep[start:end] = False
    
    if plot: # plot must be the acceleration data in order to work
        idx_keep1 = np.bitwise_and(idx_keep_t, idx_keep_r)
        accZ = plot
        accZ_smoothed = np.convolve(accZ, np.ones(50)/50., 'same')
        t = np.arange(1,accZ.shape[0]+1)# / 50.
        pl.plot(t, accZ, 'b')
        pl.plot(t, accZ_smoothed, 'g')
        pl.plot([t[1],t[-1]], [accZ.mean(), accZ.mean()], '-k')
        pl.plot([t[1],t[-1]], [accZ.mean()-accZ.std(), accZ.mean()-accZ.std()], '--k')
        pl.plot([t[1],t[-1]], [accZ.mean()+accZ.std(), accZ.mean()+accZ.std()], '--k')
        pl.plot(t[idx_keep], np.ones(idx_keep.sum())*(-7.), '*g')
        pl.plot(t[idx_keep1], np.ones(idx_keep1.sum())*(-5.), '*y')
        pl.plot(t[idx_tumble==1.], np.ones(idx_tumble.sum())*(-9.), '*r')
        pl.plot(t[idx_reset==1.], np.ones(idx_reset.sum())*(-10.), '*r')
        pl.plot(t[on], np.ones(on.shape[0])*(-11.), 'db')
        pl.plot(t[off-1], np.ones(off.shape[0])*(-11.2), 'sb')
#        idx_gps = [25,26]
#        pl.plot(t, dat[:,idx_gps], '-.g')
    
    return idx_keep


def loadData(path, resampling=1, min_chunk_size=1000, gait_list=None):
    dat = read_sim_self_3_with_gait_index(path, gait_list) # are the resets correct when using gait_list and it breaks the time series???
    dat = np.concatenate([d for d in dat], axis=0)
    
    # find tumbling and reset regions:
    idx_keep = cleanData(dat[:,idx_tumble], dat[:,idx_reset], min_chunk_size=min_chunk_size*resampling)
    
    # subsample:    
    dat = dat[::resampling]
    idx_keep = idx_keep[::resampling]
    
    # find chunks:
    i_on = np.zeros(dat.shape[0], dtype=bool)
    i_on[pl.find(np.diff(idx_keep.astype(int))>0)+1] = True
    i_on[0] = idx_keep[0]
    
    # remove tumbling and reset regions:
    dat = dat[idx_keep]
    i_on = i_on[idx_keep]
    
    return dat, i_on
