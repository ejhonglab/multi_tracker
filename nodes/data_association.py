#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import rospy
import copy
import numpy as np
import os, sys

from std_msgs.msg import Float32, Header, String
from geometry_msgs.msg import Point, Vector3
from multi_tracker.msg import Contourinfo, Contourlist
from multi_tracker.msg import Trackedobject, Trackedobjectlist

import matplotlib.pyplot as plt
import multi_tracker_analysis.Kalman as Kalman
import imp

import threading


class DataAssociator(object):
    def __init__(self):
        kalman_parameter_py_file = os.path.expanduser( rospy.get_param('multi_tracker/data_association/kalman_parameters_py_file') )
        self.lockBuffer = threading.Lock()

        try:
            self.kalman_parameters = imp.load_source('kalman_parameters', kalman_parameter_py_file)
            print 'Kalman py file: ', kalman_parameter_py_file
        except: # look in home directory for kalman parameter file
            home_directory = os.path.expanduser( rospy.get_param('multi_tracker/home_directory') )
            kalman_parameter_py_file = os.path.join(home_directory, kalman_parameter_py_file)
            self.kalman_parameters = imp.load_source('kalman_parameters', kalman_parameter_py_file)
            print 'Kalman py file: ', kalman_parameter_py_file

        self.association_matrix = self.kalman_parameters.association_matrix
        self.association_matrix /= np.linalg.norm(self.association_matrix)
        self.max_covariance = self.kalman_parameters.max_covariance
        self.max_velocity = self.kalman_parameters.max_velocity

        self.tracked_objects = {}
        self.current_objid = 0
        
        #self.min_size = rospy.get_param('multi_tracker/data_association/min_size')
        #self.max_size = rospy.get_param('multi_tracker/data_association/max_size')
        self.max_tracked_objects = rospy.get_param('multi_tracker/data_association/max_tracked_objects')
        self.n_covariances_to_reject_data = rospy.get_param('multi_tracker/data_association/n_covariances_to_reject_data')

        self.contour_buffer = []

        # initialize the node
        rospy.init_node('data_associator')
        self.time_start = rospy.Time.now()
        
        # Publishers.
        self.pubTrackedObjects = rospy.Publisher('multi_tracker/tracked_objects', Trackedobjectlist, queue_size=300)
        
        # Subscriptions.
        self.subImage = rospy.Subscriber('multi_tracker/contours', Contourlist, self.contour_callback, queue_size=300)

    def new_tracked_object(self, contour):
        obj_state = np.matrix([contour.x, 0, contour.y, 0, 0, 0, contour.area, 0, contour.angle, 0]).T # pretending 3-d tracking (z and zvel) for now
        obj_measurement = np.matrix([contour.x, contour.y, 0, contour.area, contour.angle]).T
        new_obj = { 'objid':        self.current_objid,
                'statenames':   {'position': [0, 2, 4],
                                 'velocity': [1, 3, 5],
                                 'size': 6,
                                 'd_size': 7,
                                 'angle': 8,
                                 'd_angle': 9,
                                },
                'state':        obj_state,
                'measurement':  obj_measurement,
                'timestamp':    [contour.header.stamp],
                'frames':       [int(contour.header.frame_id)],
                'kalmanfilter': Kalman.DiscreteKalmanFilter(x0      = obj_state,
                                                            P0      = self.kalman_parameters.P0,
                                                            phi     = self.kalman_parameters.phi,
                                                            gamma   = self.kalman_parameters.gamma,
                                                            H       = self.kalman_parameters.H,
                                                            Q       = self.kalman_parameters.Q,
                                                            R       = self.kalman_parameters.R,
                                                            gammaW  = self.kalman_parameters.gammaW,
                                                            ),
                'nframes':      0,
              }
        self.tracked_objects.setdefault(new_obj['objid'], new_obj)
        self.current_objid += 1

    # TODO set max cost as node param? redundant w/ ncov? which to keep?
    # TODO change back to costs
    def greedy_association(self, contour_to_object_error, max_cost):
        """
        costs: a numpy array of dimension (# contours in this frame x # targets in last frame).
               entries are the distance between centers.
        """
        # TODO compare floris / ctrax versions for correctness
        '''
        best_observations = costs.argmin(axis=0)
        lowest_costs = costs[best_observations]
        best_observations[lowest_costs > max_cost] = -1
        unmatched_contours = []
        unmatched_targets = []
        n_targets = costs.shape[1]
        for i in range(n_targets):
            if observation_for_target[i] < 0:
                unmatched_targets.append(i)
                contours_accounted_for.append(
        '''
        # TODO multiple assignments to same / suboptimal / dissapearing?
        # TODO TODO not currently equipped to deal with splits. fix.
        if contour_to_object_error is None or len(contour_to_object_error) == 0:
            raise ValueError('greedy_association not expecting None or empty list')
        #    return []

        contour_to_object_error = np.array(contour_to_object_error)
        #print 'unsorted costs\n', contour_to_object_error
        sorted_indices = np.argsort(contour_to_object_error[:,0])
        contour_to_object_error = contour_to_object_error[sorted_indices,:]
        #print 'sorted costs\n', contour_to_object_error

        # TODO make this a dict?
        objects2contours = []
        all_objects = set()
        all_contours = set()
        objects_accounted_for = set()
        contours_accounted_for = set()
        for data in contour_to_object_error:
            c = int(data[2])
            objid = int(data[1])
            if objid not in objects_accounted_for:
                if c not in contours_accounted_for:
                    print 'assigning c=' + str(c) + ' to o=' + str(objid) + \
                        ' w/ cost=' + str(data[0])
                    '''
                    contour = contourlist.contours[c]
                    measurement = np.matrix([contour.x, contour.y, 0, contour.area, contour.angle]).T
                    tracked_object = self.tracked_objects[objid]
                    update_tracked_object(tracked_object, measurement, contourlist)
                    '''
                    objects2contours.append( (objid, c) )
                    objects_accounted_for.add(objid)
                    contours_accounted_for.add(c)
            all_objects.add(objid)
            all_contours.add(c)

        unmatched_objects = []
        for o in all_objects:
            if not o in objects_accounted_for:
                unmatched_objects.append(o)

        unmatched_contours = []
        for c in all_contours:
            if not c in contours_accounted_for:
                unmatched_contours.append(c)
        
        return objects2contours, unmatched_objects, unmatched_contours

    def contour_callback(self, contourlist):
        with self.lockBuffer:
            self.contour_buffer.append(contourlist)

    def contour_identifier(self, contourlist):
        print 'starting contour_identifier'
        print 'contours from', contourlist.header.stamp
        # keep track of which new objects have been "taken"
        update_dict = {}

        def update_tracked_object(tracked_object, measurement, contourlist):
            print 'updating objid', tracked_object['objid']
            if measurement is None:
                print 'measurement is none'
                m = np.matrix([np.nan for i in range( tracked_object['measurement'].shape[0] ) ]).T
                xhat, P, K = tracked_object['kalmanfilter'].update( None ) # run kalman filter
            else:
                tracked_object['measurement'] = np.hstack( (tracked_object['measurement'], measurement) ) # add object's data to the tracked object
                xhat, P, K = tracked_object['kalmanfilter'].update( tracked_object['measurement'][:,-1] ) # run kalman filter
            tracked_object['frames'].append(int(contourlist.header.frame_id))
            tracked_object['frames'].pop(0)
            tracked_object['nframes'] += 1
            tracked_object['timestamp'].append(contourlist.header.stamp)
            tracked_object['timestamp'].pop(0)
            tracked_object['state'] = np.hstack( (tracked_object['state'][:,-1], xhat) )
            #print xhat
            # TODO why is more than most recent xhat stored? (in state)

        # iterate through objects first
        # get order of persistence
        objid_in_order_of_persistance = []
        if len(self.tracked_objects.keys()) > 0:
            persistance = []
            objids = []
            for objid, tracked_object in self.tracked_objects.items():
                 persistance.append(tracked_object['nframes'])
                 objids.append(objid)
            order = np.argsort(persistance)[::-1]
            objid_in_order_of_persistance = [objids[o] for o in order]


        # loop through contours and find errors to all tracked objects (if less than allowed error)
        # then loop through the errors in order of increasing error and assign contours to objects
        tracked_object_state_estimates = None
        tracked_object_covariances = None
        tracked_object_ids = []
        for objid, tracked_object in self.tracked_objects.items():
            # TODO why these two coordinates? when is the est updated?
            tose = np.array([[tracked_object['kalmanfilter'].xhat_apriori[0,0], tracked_object['kalmanfilter'].xhat_apriori[2,0]]])
            # TODO is this line right? matmul? which wiki eqs does this correspond to?
            # TODO what are the dimensions? it bears on the dims of the section below
            cov = np.array([np.linalg.norm( (tracked_object['kalmanfilter'].H * tracked_object['kalmanfilter'].P).T * self.association_matrix )])
            tracked_object_ids.append(objid)
            if tracked_object_state_estimates is None:
                tracked_object_state_estimates = tose
                tracked_object_covariances = cov
            else:
                tracked_object_state_estimates = np.vstack((tracked_object_state_estimates,  tose))
                tracked_object_covariances = np.vstack((tracked_object_covariances,  cov))

        # build a matrix of costs (distances between detected contours and trajectories)
        if tracked_object_state_estimates is not None:
            contour_to_object_error = []
            costs = np.empty([len(contourlist.contours), len(self.tracked_objects)])
            max_cost = 100 # TODO replace me with correct value / rosparam
            for c, contour in enumerate(contourlist.contours):
                m = np.array([[contour.x, contour.y]])
                costs[c,:] = np.array([np.linalg.norm(m-e) for e in tracked_object_state_estimates])
                # TODO covariance the right term? (cause sqrt...?)
                # check dimensionality of this and above
                ncov = self.n_covariances_to_reject_data * np.sqrt(tracked_object_covariances)
                indices = np.where(costs[c,:] < ncov)[0]
                if len(indices) == 0:
                    print 'rejected all matches that might have been made to contour', c, \
                        'because of covariances'
                    print 'cutoffs\n', ncov
                    print 'data\n', costs[c,:]
                    
                # TODO reasons this would be innappropriate?
                # uncomment me. just temporarily using floris's
                #indices = np.where(costs[c,:] >= ncov)[0]
                #costs[c,indices] = max_cost
                new_contour_object_errors = [[costs[c,i], tracked_object_ids[i], c] for i in indices]
                contour_to_object_error.extend(new_contour_object_errors)
            
            if len(contour_to_object_error) > 0:
                # TODO use float INFINITY constant to get rid of max_cost?
                object2contour, unmatched_objects, unmatched_contours = \
                    self.greedy_association(contour_to_object_error, max_cost)
            else:
                contour_to_object_error = None
        else:
            contour_to_object_error = None

        if contour_to_object_error is None:
            object2contour = []
            unmatched_objects = list(self.tracked_objects.keys())
            unmatched_contours = range(len(contourlist.contours))

        #print 'object2contour', object2contour
        #print 'unmatched_objects', unmatched_objects

        #object2contour, unmatched_objects, unmatched_contours = greedy_association(costs, max_cost)
        #object2contour, unmatched_objects, unmatched_contours = optimal_association(costs)

        # TODO adaptive splitting based on estimates of blob areas?
        # could initiate service request with image processor if association alg
        # assigns two trajectories to one contour? do i need info from contour? what to send?
        # something else to update here? additional terms in cost? model of areas?

        # update state estimates for tracked objects matched to contours
        print('updating matched objects')
        for row in object2contour:
            object_id = row[0]
            contour_id = row[1]
            tracked_object = self.tracked_objects[object_id]
            contour = contourlist.contours[contour_id]
            measurement = np.matrix([contour.x, contour.y, 0, contour.area, contour.angle]).T
            # TODO this seems to only use contourlist for metadata. maybe pass explicitly.
            update_tracked_object(tracked_object, measurement, contourlist)
        print('done updating matched objects')

        # update predictions for unmatched objects
        # TODO so something was in both this and obj2cont?
        print('updating objects (that are supposed to be) not matched to contours')
        for object_id in unmatched_objects:
            update_tracked_object(self.tracked_objects[object_id], None, contourlist)
        print('done updating unmatched objects')
        '''
        # TODO so what causes unmatched objects to finally get dropped? after how many frames?
        # is it just a byproduct of the covariance getting large (for some reason?, if it does?)
        for objid, tracked_object in self.tracked_objects.items():
            # TODO was this information stored in tracked object just to check
            # whether it was matched? if so, maybe change?
            if tracked_object['frames'][-1] != int(contourlist.header.frame_id):
                update_tracked_object(tracked_object, None, contourlist)
        '''

        # any unnaccounted contours should spawn new objects
        for contour_id in unmatched_contours:
            #print('contour not accounted for')
            self.new_tracked_object(contourlist.contours[contour_id])

        # make sure we don't get too many objects - delete the oldest ones, and ones with high covariances
        # TODO optionally maintain estimate of # of objects, and then aim for that target,
        # rather than track too few objects?
        objects_to_destroy = []
        if len(objid_in_order_of_persistance) > self.max_tracked_objects:
            for objid in objid_in_order_of_persistance[self.max_tracked_objects:]:
                print 'will destroy', objid, 'because it is one of least persistent'
                print 'and we are over max_tracked_objects'
                objects_to_destroy.append(objid)

        # TODO double check none of this filtering steps neighbor the bad frames
        # check covariance, and velocity
        for objid, tracked_object in self.tracked_objects.items():
            tracked_object_covariance = np.linalg.norm( (tracked_object['kalmanfilter'].H*tracked_object['kalmanfilter'].P).T * self.association_matrix )
            if tracked_object_covariance > self.max_covariance:
                print 'destroying object', objid, 'for high covariance', tracked_object_covariance
                if objid not in objects_to_destroy:
                    objects_to_destroy.append(objid)
            
            v = np.linalg.norm( np.array( tracked_object['state'][tracked_object['statenames']['velocity'],-1] ).flatten().tolist() )
            if v > self.max_velocity:
                print 'destroying object', objid, 'for high velocity', v
                if objid not in objects_to_destroy:
                    objects_to_destroy.append(objid)

        for objid in objects_to_destroy:
            del(self.tracked_objects[objid])

        # recalculate persistance (not necessary, but convenient)
        # TODO why?
        objid_in_order_of_persistance = []
        if len(self.tracked_objects.keys()) > 0:
            persistance = []
            for objid, tracked_object in self.tracked_objects.items():
                 persistance.append(len(tracked_object['frames']))
                 objid_in_order_of_persistance.append(objid)
            order = np.argsort(persistance)[::-1]
            objid_in_order_of_persistance = [objid_in_order_of_persistance[o] for o in order]

        # publish tracked objects
        object_info_to_publish = []
        t = contourlist.header.stamp
        for objid in objid_in_order_of_persistance:
            # TODO just remove from this above to remove need for this check?
            if objid not in objects_to_destroy:
                tracked_object = self.tracked_objects[objid]
                data = Trackedobject()
                data.header  = Header(stamp=t, frame_id=contourlist.header.frame_id)
                p = np.array( tracked_object['state'][tracked_object['statenames']['position'],-1] ).flatten().tolist()
                v = np.array( tracked_object['state'][tracked_object['statenames']['velocity'],-1] ).flatten().tolist()
                data.position       = Point( p[0], p[1], p[2] )
                data.velocity       = Vector3( v[0], v[1], v[2] )
                data.angle          = tracked_object['state'][tracked_object['statenames']['angle'],-1]
                data.size           = tracked_object['state'][tracked_object['statenames']['size'],-1]#np.linalg.norm(tracked_object['kalmanfilter'].P.diagonal())
                data.measurement    = Point( tracked_object['measurement'][0, -1], tracked_object['measurement'][1, -1], 0)
                # TODO shouldn't we be maintaining a covariance matrix? for debuggin?
                tracked_object_covariance = np.linalg.norm( (tracked_object['kalmanfilter'].H*tracked_object['kalmanfilter'].P).T * self.association_matrix )
                data.covariance     = tracked_object_covariance # position covariance only
                data.objid          = tracked_object['objid']
                data.persistence    = tracked_object['nframes']
                object_info_to_publish.append(data)
            #print tracked_object['state']
        header = Header(stamp=t)
        self.pubTrackedObjects.publish( Trackedobjectlist(header=header, tracked_objects=object_info_to_publish) )
        print('finishing contour_identifier\n\n')

    def main(self):
        while not rospy.is_shutdown():
            t = rospy.Time.now() - self.time_start
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.contour_buffer) > 0:
                    print 'LENGTH OF CONTOUR_BUFFER', len(self.contour_buffer)
                    self.contour_identifier(self.contour_buffer.pop(0))
                
                if len(self.contour_buffer) > 9:
                    pt = (rospy.Time.now() - time_now).to_sec()
                    rospy.logwarn("Data association processing time exceeds acquisition rate. Processing time: %f, Buffer: %d", pt, len(self.contour_buffer))



if __name__ == '__main__':
    data_associator = DataAssociator()
    data_associator.main()
