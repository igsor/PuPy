"""

Code for input/output handling

"""

def clear_empty_groups(pth):
    """Remove HDF5 groups with no datasets from ``pth``. Return a list
    of removed groups."""
    import h5py
    f = h5py.File(pth,'a')
    deleted = []
    for k in f.keys():
        if len(f[k].keys()) == 0:
            deleted.append(k)
            del f[k]
    
    f.close()
    return deleted

class Normalization(object):
    """Supply normalization routines for sensor data. The normalization
    parameters (per sensor) are loaded from the ``pth`` file. See
    `py:meth:`load_normalization` for the file syntax.
    """
    def __init__(self, pth=None):
        if pth is not None:
            self.load_normalization(pth)

    def set_normalization(self, sensor, offset, scale):
        """Set the normalization parameters ``offset`` and ``scale`` for
        a specific ``sensor``.
        
        The normalization computes the following:
            
            x' = ( x - ``offset`` ) / ``scale``
        
        """
        self._sensor_mapping[sensor] = (offset, scale)
    
    def load_normalization(self, pth):
        """Load normalization parameters from a *JSON* file at ``pth``.
        
        The file structure must be like so:
        {
            "<sensor name>" : [<offset>, <scale>],
        }
        
        For example:
        {
            "hip0"   : [0.9678, 3.141],
            "touch0" : [-0.543, 1e3],
            "trg0"   : [1e-2, 8]
        }
        
        .. note::
            This routine is only available, if the *json* module is
            installed.
        
        """
        import json
        f = open(pth,'r')
        self._sensor_mapping = json.load(f)
        f.close()
    
    def normalize_epoch(self, epoch):
        """Normalize all values in the :py:keyword:`dict` ``epoch``,
        where the key is regarded as sensor name."""
        if len(self._sensor_mapping) > 0:
            new_epoch = {}
            for lbl in epoch:
                new_epoch[lbl] = self.normalize_value(lbl, epoch[lbl])
        else:
            new_epoch = epoch
        
        return epoch
    
    def denormalize_epoch(self, epoch):
        """Denormalize all values in the :py:keyword:`dict` ``epoch``,
        where the key is regarded as sensor name."""
        if len(self._sensor_mapping) > 0:
            new_epoch = {}
            for lbl in epoch:
                new_epoch[lbl] = self.denormalize_value(lbl, epoch[lbl])
        else:
            new_epoch = epoch
            
        return epoch
    
    def normalize_value(self, sensor, value):
        """Return the normalized ``value``, with respect to ``sensor``."""
        if sensor not in self._sensor_mapping:
            return value
        
        offset, scale = self._sensor_mapping[sensor]
        return (value - offset) / scale
    
    def denormalize_value(self, sensor, value):
        """Return the denormalized ``value``, with respect to
        ``sensor``."""
        if sensor not in self._sensor_mapping:
            return value
        
        offset, scale = self._sensor_mapping[sensor]
        return scale * value + offset

