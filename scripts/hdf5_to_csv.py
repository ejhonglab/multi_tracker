#!/usr/bin/env python

from __future__ import print_function

from argparse import ArgumentParser
from os.path import join, splitext
import glob

import pandas as pd
#import matplotlib.pyplot as plt

import multi_tracker_analysis as mta


def main():
    additional_cols = ['measurement_x', 'measurement_y', 'objid', 'area']

    parser = ArgumentParser()
    # TODO where is 'area' column actually coming from? not obvious in
    # data_association.py...
    parser.add_argument('-a', '--all', action='store_true', help='saves '
        'additional columns that might be useful (%s)' % additional_cols
    )
    args = parser.parse_args()

    columns = ['time_epoch', 'position_x', 'position_y']
    if args.all:
        columns += additional_cols

    experiment_dir = '.'

    def find_file_via_suffix(suffix):
        files = glob.glob(join(experiment_dir, '*' + suffix))
        assert len(files) == 1
        return files[0]

    csv_fname = find_file_via_suffix('_stimuli.csv')
    sdf = pd.read_csv(csv_fname)

    for hdf5_fname in glob.glob(join(experiment_dir, '*_trackedobjects.hdf5')):
        print(hdf5_fname)
        # TODO suppress load message about config files we aren't using.
        # seems the latter is None. but what else could it be?
        df, _ = mta.read_hdf5_file_to_pandas.load_and_preprocess_data(
            hdf5_fname
        )
        csv_fname = splitext(hdf5_fname)[0] + '.csv'

        # TODO TODO TODO are measurement_[x|y] columns nan where appropriate?
        # (seems no...) otherwise, what happens if there isn't truly a contour
        # updating the filter?

        # TODO also add speed / velocity columns after checking they are correct
        # / fixing if need be
        df.to_csv(csv_fname, columns=columns, index=False)


if __name__ == '__main__':
    main()

