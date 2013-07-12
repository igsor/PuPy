"""

Code for input/output handling

"""

def clear_empty_groups(pth):
    """Remove HDF5 groups with no datasets from ``pth``."""
    import h5py
    f = h5py.File(pth,'a')
    for k in f.keys():
        if len(f[k].keys()) == 0:
            del f[k]
    
    f.close()
