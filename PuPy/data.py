'''
Created on Dec 8, 2012

@author: nuci
'''

import numpy as np
import pylab as pl
import h5py


class RobotData(object):
    '''
    This class contains the data from Robot experiments including relevant meta information.
    '''
    def __init__(self, filename='', experiment=0, datasets=[], fs=50., resampling=1, load_data=True):
        self.filename = filename
        self.experiment = experiment
        self.datasets = datasets
        self.fs = fs
        self._resampling = resampling
        self._x = np.array([])
        self.t = np.array([])
        self._loaded = False
        
        if self.filename and load_data:
            self.loadData()
    
    def loadData(self, experiment=None, datasets=None):
        """
        Load the data from the file into the memory.
        ``experiment``
            An :py:keyword:int: specifying which group will be read from the file (which experiment will be loaded).
        
        ``datasets``
            A :py:keyword:list: containing the names of the datasets that will be loaded and form the columns of the data matrix.
        """
        # override experiment and datasets if desired:
        if experiment is not None:
            self.experiment = experiment
        if datasets is None:
            datasets = self.datasets
        
        # open file and load data:
        with h5py.File(self.filename, 'r') as fid:
            if len(datasets)==0: # load all datasets if none is given
                # TODO: this is unsafe when the experiment includes datasets of different length
                datasets = sorted(fid['/%d'%self.experiment].keys(), key=unicode.lower)
            
            # create data matrix:
            self.datasets = []
            self._x = np.empty([0, fid['/%d'%self.experiment].values()[0].shape[0]])
            for d in datasets:
                if d in fid['/%d'%self.experiment].keys():
                    self._x = np.vstack([self._x, fid['/%d'%self.experiment][d]])
                    self.datasets.append(d)
                else:
                    print "dataset %s not in file. skipping..." % d
            self._x = self._x.T
            self.t = np.arange(self.nSamples, dtype=int)
                
            # subsample if requested:    
            self._x = self._x[::self._resampling]
            self.t = self.t[::self._resampling]
            self.fs /= self._resampling
        
            self._loaded = True
        
    
    def copy(self, idx=None):
        '''
        deep copy of the data object.
        '''
        # create RobotData instance:
        cpy = RobotData(filename='',
                        experiment=self.experiment,
                        datasets=self.datasets,
                        fs=self.fs,
                        resampling=self._resampling)
        # copy data and fields:
        if idx is None:
            cpy._x = self._x.copy()
            cpy.t = self.t.copy()  
        else:
            if isinstance(idx, int):
                idx = np.arange(idx)
            cpy._x = self._x[idx]
            cpy.t = self.t[idx]
        cpy.filename = self.filename
        cpy._loaded = self._loaded
        return cpy
    
    def __getitem__(self, idx):
        return self._x.__getitem__(idx) # like this it is compatible with array representation of the data
    
    def __setitem__(self, idx, val):
        self._x.__setitem__(idx, val) # like this it is compatible with array representation of the data
    
    def __iter__(self):
        self._current_iter = -1
        return self
    
    def next(self):
        self._current_iter += 1
        if self._current_iter >= self.nSamples:
            raise StopIteration
        else:
            return self._x[self._current_iter]
    
    def _shape(self):
        return self._x.shape
    shape = property(_shape)
    
    def _nSamples(self):
        return self._x.shape[0]
    nSamples = property(_nSamples)
    
    def _get_idx_signals(self):
        idx_signals = {}
        for i,d in enumerate(self.datasets):
            idx_signals[d] = i
        return idx_signals
    def _set_idx_signals(self, idx_signals):
        self.datasets = []
        for i in range(max(idx_signals.values())):
            self.datasets.append(filter(lambda x:idx_signals[x]==i, idx_signals)[0])
    idx_signals = property(_get_idx_signals, _set_idx_signals, doc="dict containing the indexes to the datasets")
    
    def __str__(self):
        return "<%s from file '%s'>" % (str(self.__class__).split('.')[-1][:-2], self.filename)
    
    def __repr__(self):
        return "%s(filename='%s',experiment=%d,datasets=%s,fs=%.f,resampling=%d)" % \
            (str(self.__class__).split('.')[-1][:-2], self.filename, self.experiment, str(self.datasets), self.fs, self._resampling)        

class PuppyData(RobotData):
    '''
    This class contains the data from Puppy experiments including relevant meta information.
    '''
     
    # default signals:
    # FL: front left, FR: front right, HL: hind left, HR: hind right
    # X: front-back axis, Y: left-right axis, Y: top-bottom axis
    idx_signals = {'trg0':0, 'trg1':1, 'trg2':2, 'trg3':3, # Motor target angles in rad
                   'hip0':4, 'hip1':5, 'hip2':6, 'hip3':7, # hip joint angles in rad
                   'knee0':8, 'knee1':9, 'knee2':10, 'knee3':11, # knee joint angles in rad
                   'touch0':12, 'touch1':13, 'touch2':14, 'touch3':15, # Touch sensors at the feet some pressure unit
                   'accelerometer_x':16, 'accelerometer_y':17, 'accelerometer_z':18, # acceleration sensors in m/s
                   'gyro_x':19, 'gyro_y':20, 'gyro_z':21, # gyroscope (maybe in rad/s?)
                   'compass_x':22, 'compass_y':23, 'compass_z':24, # magnetometer
                   'puppyGPS_x':25, 'puppyGPS_y':26, 'puppyGPS_z':27} # Global Positioning Sensor in world coordinated
#                   'gait':28, # index of the current gait 
#                   'ground':29, # index of the current terrain Puppy's center of mass is at (numbering defined in individual experiment)
#                   'reset':30, # 1 where Puppy was respawned at a new location, 0 otherwise
#                   'tumble':31 # 1 where Puppy fell over, 0 otherwise

    # TODO: !!!!!!!!!!!!!!!!!!!! solve issue with optional gait,ground,tumble,reset !!!!!!!!!!!!!!!!!!!!!!!!!

    idx_motor = [idx_signals[k] for k in ['trg0', 'trg1', 'trg2', 'trg3']]
    idx_sensor = [idx_signals[k] for k in ['hip0', 'hip1', 'hip2', 'hip3',
                                           'knee0', 'knee1', 'knee2', 'knee3',
                                           'touch0', 'touch1', 'touch2', 'touch3',
                                           'accelerometer_x', 'accelerometer_y', 'accelerometer_z',
                                           'gyro_x', 'gyro_y', 'gyro_z',
                                           'compass_x', 'compass_y', 'compass_z',
                                           'puppyGPS_x', 'puppyGPS_y', 'puppyGPS_z']]
    idx_gps = [idx_signals[k] for k in ['puppyGPS_x', 'puppyGPS_y', 'puppyGPS_z',]]
#    idx_terrain = idx_signals['ground']
#    idx_reset = idx_signals['reset']
#    idx_tumble = idx_signals['tumble']
#    idx_gait = idx_signals['gait']
    
    idx_modalities = [[0,1,2,3], [4,5,6,7], [8,9,10,11], [12,13,14,15],
                      [16,17,18], [19,20,21], [22,23,24], [25,26,27]]
    
    names_modalities = ['Motor commands', 'Hip angles', 'Knee angles', 'Touch pressure', 'Acceleration', 'Gyroscope', 'Compass', 'Position']
    units_modalities = ['rad', 'rad', 'rad', 'N', 'm/s^2', 'rad/s', 'north vector', 'm']
    
    def __init__(self, *args, **kwargs):
        '''
        Load data from filename and clean it from artifacts as desired.
        '''
        super(PuppyData, self).__init__(*args, **kwargs)
        self._remove_tumbling = False
        self._remove_reset = False
        self.min_chunk_size = 1
        self._pre_delete_tumble = 0
        self._post_delete_tumble = 0
        self._pre_delete_reset = 0
        self._post_delete_reset = 0
        self.i_chunk = np.array([0])
        self.nChunks = 1
        
    def loadData(self, experiment=None, datasets=None):
        if datasets is None or len(datasets)==0:
            datasets = [filter(lambda x:self.idx_signals[x]==i, self.idx_signals)[0] for i in range(len(self.idx_signals))]
        super(PuppyData, self).loadData(experiment, datasets)
        
    def cleanData(self, min_chunk_size=1000,
                  remove_tumbling=True, pre_delete_tumble=20, post_delete_tumble=20,
                  remove_reset=True, pre_delete_reset=0, post_delete_reset=20):
        
        if not self._loaded:
            self.loadData()
        
        self.min_chunk_size = min_chunk_size
        self._remove_tumbling = remove_tumbling
        self._remove_reset = remove_reset
        self._pre_delete_tumble = pre_delete_tumble
        self._post_delete_tumble = post_delete_tumble
        self._pre_delete_reset = pre_delete_reset
        self._post_delete_reset = post_delete_reset
        self.i_chunk = np.array([0])
        self.nChunks = 1
        
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
    
    
    def copy(self, idx=None):
        cpy = super(PuppyData, self).copy(idx)
        
        if idx is None:
            cpy.i_chunk = self.i_chunk.copy()
        else:
            if isinstance(idx, int):
                idx = np.arange(idx)
            # this works only if idx is monotonically increasing with step size 1!
            cpy.i_chunk = np.unique(np.hstack([idx[:1], self.i_chunk[np.bitwise_and(self.i_chunk>=idx[0],self.i_chunk<=idx[-1])]]))-idx[0]
        cpy.nChunks = len(cpy.i_chunk)
        cpy._remove_tumbling = self._remove_tumbling
        cpy._remove_reset = self._remove_reset
        cpy.min_chunk_size = self.min_chunk_size
        cpy._pre_delete_tumble = self._pre_delete_tumble
        cpy._post_delete_tumble = self._post_delete_tumble
        cpy._pre_delete_reset = self._pre_delete_reset0
        cpy._post_delete_reset = self._post_delete_reset0
        return cpy
    
    # TODO check index lists (self.idx_gait etc...)
    def _get_gait(self):
        return self._x[:,self.idx_gait]
    def _set_gait(self, value):
        self._x[:,self.idx_gait] = value
    gait = property(_get_gait, _set_gait, doc="Gait number as indicated by the 'index' field of the used gait")
    
    def _get_ground(self):
        return self._x[:,self.idx_terrain]
    def _set_ground(self, value):
        self._x[:,self.idx_terrain] = value
    ground = property(_get_ground, _set_ground, doc="Ground number as indicated by the terrain info file (zero by default)")
    
    def _get_gps(self):
        return self._x[:,self.idx_gps]
    def _set_gps(self, value):
        self._x[:,self.idx_gps] = value
    gps = property(_get_gps, _set_gps, doc="Current position of the robot from global positioning sensor (_x,y,z)")
    
    def _get_tumble(self):
        return self._x[:,self.idx_tumble]
    def _set_tumble(self, value):
        self._x[:,self.idx_tumble] = value
    tumble = property(_get_tumble, _set_tumble, doc="has a 1.0 where Puppy tumbled and 0.0 elsewhere")
    
    def _get_reset(self):
        return self._x[:,self.idx_reset]
    def _set_reset(self, value):
        self._x[:,self.idx_reset] = value
    reset = property(_get_reset, _set_reset, doc="has a 1.0 where Puppy was reset to a new position and 0.0 elsewhere")
    
    


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
