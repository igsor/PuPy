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

