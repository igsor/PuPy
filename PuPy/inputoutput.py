"""

Code for input/output handling

"""

import warnings

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
    
    .. todo::
        Doesn't work with zero-mean and unit variance data. (There was
        a bug...).
    
    The normalization takes two parameters, an *offset* ($o$) and
    *scale* ($s$).
    Data is normalized according to
    
    .. math::
        x' = \\frac{ x - o }{ s }
    
    and denormalized
    
    .. math::
        x = s * x' + o
    
    The normalization parameters *offset* and *scale* must be set such
    that the desired mapping is achieved.
    
    """
    def __init__(self, pth=None):
        self._sensor_mapping = {}
        if pth is not None:
            self.load(pth)

    def set(self, sensor, offset, scale):
        """Set the normalization parameters ``offset`` and ``scale`` for
        a specific ``sensor``.
        
        The normalization computes the following:
            
            x' = ( x - ``offset`` ) / ``scale``
        
        """
        self._sensor_mapping[sensor] = (offset, scale)
    
    def get(self, sensor):
        """Return the normalization parameters of ``sensor``. The order
        is (offset, scale). A valid result is returned in any case.
        If the sensor was not configured, the identity mapping is
        returned, i.e.
        
        >>> nrm = Normalization()
        >>> nrm.set('unknown_sensor', *nrm.get('unknown_sensor'))
        >>> nrm.normalize_value('unknown_sensor', value) == value
        True
        
        However, a warning will be issued.
        
        """
        if sensor in self._sensor_mapping:
            return self._sensor_mapping[sensor]
        else:
            warnings.warn('Tried to get normalization parameters for an unknown sensor (%s)'%sensor)
            return (0.0, 1.0)
    
    def load(self, pth):
        """Load normalization parameters from a *JSON* file at ``pth``.
        
        The file structure must be like so:

        .. code-block:: javascript
        
            {
                "<sensor name>" : [<offset>, <scale>],
            }
        
        For example:
        
        
        .. code-block:: javascript
        
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
    
    def save(self, pth):
        """Store the normalization parameters in a *JSON* file at
        ``pth``.
        """
        import json
        f = open(pth,'w')
        json.dump(self._sensor_mapping, f)
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
            warnings.warn('Tried to normalize unknown sensor (%s)'%sensor)
            return value
        
        offset, scale = self._sensor_mapping[sensor]
        return (value - offset) / scale
    
    def denormalize_value(self, sensor, value):
        """Return the denormalized ``value``, with respect to
        ``sensor``."""
        if sensor not in self._sensor_mapping:
            warnings.warn('Tried to denormalize unknown sensor (%s)'%sensor)
            return value
        
        offset, scale = self._sensor_mapping[sensor]
        return scale * value + offset
    
    def params_unit(self, data, sensor=None):
        """Find parameters *offset* and *scale* of ``data``, such that
        ``data`` is mapped to [-1.0, 1.0].
        
        If not :py:const:`None`, the parameters will be attached to
        ``sensor``.
        
        With :math:`o = \min \\text{data}` and
        :math:`s = \max \\text{data} - \min \\text{data}`,
        the normalization to [-1, 1] becomes
        
        .. math::
            x' = 2 \\frac{x - o }{ s } - 1
        
        Since the normalization is is done differently
        (see :py:class:`Normalization`), this has to be reformulated:
        
        .. math::
            :nowrap:
            
            \\begin{eqnarray*}
            x' &=& 2 \\frac{ x - o }{ s } - 1 \\\\
               &=& \\frac{2}{s} (x - o) - \\frac{ 2/s }{ 2/s } \\\\
               &=& \\frac{2}{s} \\left[ x - o - \\frac{ 1 }{ 2/s } \\right] \\\\
               &=& \\frac{2}{s} \\left[ x - \\left( o + \\frac{ s }{ 2 } \\right) \\right] \\\\
            \\end{eqnarray*}
        
        So, the effective normalization parameters are
        
        .. math::
            :nowrap:
            
            \\begin{eqnarray*}
            \\text{scale} &=& \\frac{s}{2} \\\\
            \\text{offset} &=& o + \\frac{s}{2}
            \\end{eqnarray*}
        
        """
        scale = data.ptp() / 2.0
        offset = data.min() + scale
        
        if sensor is not None:
            self._sensor_mapping[sensor] = (offset, scale)
        
        return offset, scale
    
    def params_stat(self, data, sensor=None):
        """Find parameters *offset* and *scale* of ``data``, such that
        normalized ``data`` has zero mean and unit variance.
        
        If not :py:const:`None`, the parameters will be attached to
        ``sensor``.
        
        The normalization parameters can be computed straight-forward:
        The *offset* is the mean, the *scale* the standard deviance.
        """
        offset, scale = data.mean(), data.std()
        
        if sensor is not None:
            self._sensor_mapping[sensor] = (offset, scale)
        return offset, scale
