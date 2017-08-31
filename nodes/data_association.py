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
        kalman_parameter_py_file = rospy.get_param('multi_tracker/data_association/kalman_parameters_py_file')
        
        self.lockBuffer = threading.Lock()

        try:
            self.kalman_parameters = imp.load_source('kalman_parameters', kalman_parameter_py_file)
            print 'Kalman py file: ', kalman_parameter_py_file
        except:
            if rospy.get_param('multi_tracker/explicit_directories', False):
                # look in home directory for kalman parameter file
                home_directory = os.path.expanduser(rospy.get_param('multi_tracker/home_directory'))
                kalman_parameter_py_file = os.path.join(home_directory, kalman_parameter_py_file)
                self.kalman_parameters = imp.load_source('kalman_parameters', \
                    kalman_parameter_py_file)
                print 'Kalman py file: ', kalman_parameter_py_file
            else:
	        # TODO make one long string so error is all printed
                raise IOError(kalman_parameter_py_file + ' not found. Try launching tracking' + \
                    ' from a directory with all required configuration files with ROS_HOME=`pwd`' + \
                    ', or set the multi_tracker/explicit_directories parameter in ' + \
                    'tracker_parameters.yaml to true.')

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

        self.debug = rospy.get_param('multi_tracker/data_association/debug', False)

        # initialize the node
        rospy.init_node('data_associator')
        self.time_start = rospy.Time.now()

        node_name = rospy.get_name()
        last_name_component = node_name.split('_')[-1]
        try:
            self.pipeline_num = int(last_name_component)
            remap_topics = True
        except ValueError:
            remap_topics = False

        tracked_object_topic = 'multi_tracker/tracked_objects'
        contour_topic = 'multi_tracker/contours'
        
        if remap_topics:
            tracked_object_topic = tracked_object_topic + '_' + str(self.pipeline_num)
            contour_topic = contour_topic + '_' + str(self.pipeline_num)
        
        # Publishers.
        self.pubTrackedObjects = rospy.Publisher(tracked_object_topic, Trackedobjectlist, queue_size=300)
        
        # Subscriptions.
        self.subImage = rospy.Subscriber(contour_topic, Contourlist, self.contour_callback, queue_size=300)
 

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

    def hungarian_association(self, costs, max_cost=None, split_cost=None):
        raise NotImplementedError()
        return objects2contours, unmatched_objects, unmatched_contours

    def ctrax_mixed_association(self, costs, max_cost=None, split_cost=None):
        raise NotImplementedError()
        return objects2contours, unmatched_objects, unmatched_contours

    def ctrax_greedy_association(self, costs, max_cost=None, split_cost=None):
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
        raise NotImplementedError()
        return objects2contours, unmatched_objects, unmatched_contours

    # TODO set max cost as node param? redundant w/ ncov? which to keep?
    # TODO change back to costs
    def floris_greedy_association(self, costs, max_cost=None, split_cost=None):
        """
        costs: a numpy array of dimension (# contours in this frame x # targets in last frame).
               entries are the distance between centers.
        """
        def drop_max_cost(coord_val):
            """
            """
            return coord_val[coord_val[:,2] < max_cost,:]
            
        #indices = np.where(costs[c,:] < ncov)[0]
        #new_contour_object_errors = [[costs[c,i], tracked_object_ids[i], c] for i in indices]
        #contour_to_object_error.extend(new_contour_object_errors)
        contour_to_object_error = cost_matrix_to_coord_val_format(costs)
        #print 'costs converted to coord-val format'
        #print contour_to_object_error

        # values over threshold # covariances are set to max cost before this function
        contour_to_object_error = drop_max_cost(contour_to_object_error)
        #print 'after dropping max_cost entries'
        #print contour_to_object_error
        
        # TODO compare floris / ctrax versions for correctness
        # TODO multiple assignments to same / suboptimal / dissapearing?
        # TODO TODO not currently equipped to deal with splits. fix.
        
        #print 'unsorted by costs\n', contour_to_object_error
        sorted_indices = np.argsort(contour_to_object_error[:,2])
        contour_to_object_error = contour_to_object_error[sorted_indices,:]
        #print 'sorted by costs\n', contour_to_object_error

        objects2contours = {}
        all_objects = set()
        all_contours = set()
        objects_accounted_for = set()
        contours_accounted_for = set()
        for data in contour_to_object_error:
            # TODO check these are the correct indexes
            c = int(data[1])
            objid = int(data[0])
            if objid not in objects_accounted_for:
                if c not in contours_accounted_for:
                    if self.debug:
                        print 'assigning c=' + str(c) + ' to o=' + str(objid) + \
                            ' w/ cost=' + str(data[2])
                    
                    objects2contours[objid] = c
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
        
        # will have to convert to current object id after returning
        # here, objects are just the indices in the cost matrix, which is the order in
        # tracked_object_ids
        return objects2contours, unmatched_objects, unmatched_contours

    def contour_callback(self, contourlist):
        with self.lockBuffer:
            self.contour_buffer.append(contourlist)

    def contour_identifier(self, contourlist):
        if self.debug:
            print 'starting contour_identifier'
            print 'contours from', contourlist.header.stamp
        # keep track of which new objects have been "taken"
        update_dict = {}

        def update_tracked_object(tracked_object, measurement, contourlist):
            if self.debug:
                print 'updating objid', tracked_object['objid']
            if measurement is None:
                if self.debug:
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

        # TODO break this out into a validation_gate function?
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
    
        '''
        if not tracked_object_covariances is None:
            print 'tracked_object_covariances.shape', tracked_object_covariances.shape
            print 'np.sqrt(tracked_object_covariances).shape', \
                np.sqrt(tracked_object_covariances).shape
        '''

        # build a matrix of costs (distances between detected contours and trajectories)
        if tracked_object_state_estimates is not None:
            costs = np.empty([len(contourlist.contours), len(self.tracked_objects)])
            # TODO warn if incurring max_cost (in debugging)
            max_cost = 50 # TODO replace me with correct value / rosparam
            do_assignment = False
            
            for c, contour in enumerate(contourlist.contours):
                m = np.array([[contour.x, contour.y]])
                costs[c,:] = np.array([np.linalg.norm(m-e) for e in tracked_object_state_estimates])
                # TODO covariance the right term? (cause sqrt...?)
                # check dimensionality of this and above
                ncov = self.n_covariances_to_reject_data * np.sqrt(tracked_object_covariances)
                    
                # TODO reasons this would be inappropriate?
                # uncomment me. just temporarily using floris's
                # TODO shouldn't this threshold be wrt the current estimates (centered)?
                mask = costs[c,:] >= ncov.flatten()
                # TODO why are tracked_object_covariances not a 1d array? werent they scalars?
                # if this is a history thing, maybe get rid of it?
                costs[c, mask] = max_cost
                objects_rejected = np.sum(mask)
                if objects_rejected < len(mask):
                    do_assignment = True
                
                if self.debug:
                    if objects_rejected == len(mask):
                        print 'rejected all matches that might have been made to contour', c, \
                            'because of covariances'
                        print 'cutoffs\n', ncov
                        print 'data\n', costs[c,:]
                    elif objects_rejected  > 0:
                        print objects_rejected , 'pairs above covariance threshold (set to max_cost)'

            if do_assignment:
                # TODO use float INFINITY constant to get rid of max_cost?
                # TODO just pass floris' the object covariances too, rather than acting
                # indirectly through max_cost?
                object2contour, unmatched_objects, unmatched_contours = \
                    self.floris_greedy_association(costs, max_cost)
                
                # above function uses indices of cost matrix to refer to objects
                # point those indices back to the current objects they represent
                # TODO break info function?
                object2contour = {tracked_object_ids[o]: c for o, c in object2contour.iteritems()}
                unmatched_objects = [tracked_object_ids[o] for o in unmatched_objects]
                
            else:
                costs = None
        else:
            costs = None

        if costs is None:
            object2contour = {}
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
        # TODO make room for this kind of stuff in association algs + returns

        # update state estimates for tracked objects matched to contours
        if self.debug:
            print('updating matched objects')
        for object_id, contour_id in object2contour.iteritems():
            tracked_object = self.tracked_objects[object_id]
            contour = contourlist.contours[contour_id]
            measurement = np.matrix([contour.x, contour.y, 0, contour.area, contour.angle]).T
            # TODO this seems to only use contourlist for metadata. maybe pass explicitly.
            update_tracked_object(tracked_object, measurement, contourlist)
        if self.debug:
            print('done updating matched objects')

        # update predictions for unmatched objects
        # TODO so something was in both this and obj2cont?
        if self.debug:
            print('updating objects (that are supposed to be) not matched to contours')
        for object_id in unmatched_objects:
            update_tracked_object(self.tracked_objects[object_id], None, contourlist)
        if self.debug:
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

        # moved after deletes for other reasons to not go below # of objects we want
        '''
        if len(objid_in_order_of_persistance) > self.max_tracked_objects:
            for objid in objid_in_order_of_persistance[self.max_tracked_objects:]:
                if self.debug:
                    print 'will destroy', objid, 'because it is one of least persistent'
                    print 'and we are over max_tracked_objects'
                objects_to_destroy.append(objid)
        '''

        # TODO double check none of this filtering steps neighbor the bad frames
        # check covariance, and velocity
        objects_to_destroy = []
        for objid, tracked_object in self.tracked_objects.items():
            # TODO isn't this calculated above? just save?
            tracked_object_covariance = np.linalg.norm( (tracked_object['kalmanfilter'].H*tracked_object['kalmanfilter'].P).T * self.association_matrix )
            if tracked_object_covariance > self.max_covariance:
                if self.debug:
                    print 'destroying object', objid, 'for high covariance', \
                        tracked_object_covariance
                
                if objid not in objects_to_destroy:
                    objects_to_destroy.append(objid)
            
            v = np.linalg.norm( np.array( tracked_object['state'][tracked_object['statenames']['velocity'],-1] ).flatten().tolist() )
            if v > self.max_velocity:
                if self.debug:
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
            # TODO some reverse arg to sort to save some steps?
            order = np.argsort(persistance)[::-1]
            objid_in_order_of_persistance = [objid_in_order_of_persistance[o] for o in order]

        # publish tracked objects
        object_info_to_publish = []
        t = contourlist.header.stamp
        num_published = 0
        for objid in objid_in_order_of_persistance:
            # make sure we don't get too many objects - delete the oldest ones, 
            # and ones with high covariances
            # TODO optionally maintain estimate of # of objects, and then aim for that target,
            # rather than track too few objects?
            if num_published > self.max_tracked_objects:
                del(self.tracked_objects[objid])
                continue
                
            # TODO just remove from this above to remove need for this check?
            if objid not in objects_to_destroy:
                # TODO break into func
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
                num_published += 1
            #print tracked_object['state']
        
        header = Header(stamp=t)
        self.pubTrackedObjects.publish( Trackedobjectlist(header=header, tracked_objects=object_info_to_publish) )
        if self.debug:
            print('finishing contour_identifier\n\n')

    def main(self):
        while not rospy.is_shutdown():
            t = rospy.Time.now() - self.time_start
            with self.lockBuffer:
                time_now = rospy.Time.now()
                if len(self.contour_buffer) > 0:
                    if self.debug:
                        rospy.loginfo('LENGTH OF CONTOUR_BUFFER' + str(len(self.contour_buffer)))
                    
                    self.contour_identifier(self.contour_buffer.pop(0))
                
                if len(self.contour_buffer) > 9:
                    pt = (rospy.Time.now() - time_now).to_sec()
                    rospy.logwarn("Data association processing time exceeds acquisition rate. Processing time: %f, Buffer: %d", pt, len(self.contour_buffer))


def cost_matrix_to_coord_val_format(cost_matrix):
    """
    """
    # TODO TODO TODO why does this seem to be necessary... it can't be. think...
    dims = np.flipud(cost_matrix.shape)
    contour_id_range = np.arange(dims[0])
    object_id_range = np.arange(dims[1])
    contour_idx, object_idx = np.meshgrid(contour_id_range, object_id_range)
    coord_val = np.vstack((contour_idx.flatten(), object_idx.flatten(), cost_matrix.flatten())).T
    return coord_val

def coord_val_to_cost_matrix(coord_val, shape=None):
    if shape is None:
        ndims = coord_val.shape[1] - 1
        shape = np.empty(ndims, dtype=np.int32)
        
        # the last column is the value, so we don't care about that here
        for i in range(ndims):
            shape[i] = np.max(coord_val[:,i]) + 1
    # TODO stop flipping shape here if i can avoid flipping in inverse
    return coord_val[:,-1].reshape(np.flipud(shape))

# TODO include me in separate file for testing w/ other tests
def test_cost_matrix_to_coord_val_format():
    """
    """
    A = np.random.rand(9,10)
    A_coord_val = cost_matrix_to_coord_val_format(A)
    A_prime = coord_val_to_cost_matrix(A_coord_val)
    assert np.all(A.shape == A_prime.shape)
    assert np.all(A == A_prime)

if __name__ == '__main__':
    data_associator = DataAssociator()
    data_associator.main()
