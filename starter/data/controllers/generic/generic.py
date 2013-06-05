CMD=''


if CMD == '': raise NotImplementedError()

import os
python = '/usr/bin/python2.7'
os.system(python + ' '  + CMD)
