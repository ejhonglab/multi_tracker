#!/usr/bin/env python

import datetime
import os
import errno

directory = datetime.datetime.today().strftime('%Y%m%d')

try:
    os.makedirs(directory)
except OSError as e:
    if e.errno != errno.EEXIST:
        raise

os.chdir(directory)
import copy_configs
