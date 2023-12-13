#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 30 15:43:19 2018

@author: matteomacchini
"""


########################################################


import os

import time
import numpy as np
import pandas as pd
import struct
import datetime
import shutil

import utilities.HRI as HRI
import HRI_mapping
import dataset_handling.user_data as user_data
import dataset_handling.CalibrationDataset as CalibrationDataset

#import remote_handler
import communication.UDP_handler as udp
import utilities.utils as utils

from settings.settings import get_settings
from settings.settings import get_feat_names

import logging


########################################################

settings = get_settings()
feat_names = get_feat_names()

logging.basicConfig(level=settings['logging_level'])

MANY_DATA = 10000000

N_FLOATS_UDP = 10

_DEBUG = {}

# calib_maneuver_dict = {0 : 'straight',
#                        1 : 'just_left',
#                        2 : 'just_right',
#                        3 : 'just_up',
#                        4 : 'just_down',
#                        5 : 'up_right',
#                        6 : 'up_left',
#                        7 : 'down_right',
#                        8 : 'down_left'}

                       
calib_maneuver_dict = {0 : 'forward',
                        1 : 'backward',
                        2 : 'yaw_right',
                        3 : 'yaw_left',
                        4 : 'up',
                        5 : 'down',
                        6 : 'left_tilt',
                        7 : 'right_tilt',
                        8 : 'just_rest',
                        9 : 'right',
                        10 : 'left',
                        11 : 'roll_up',
                        12 : 'roll_down',
                        13 : 'pitch_up',
                        14 : 'pitch_down',
                        15 : 'yaw_up',
                        16 : 'yaw_down',
                        17 : 'no_input',
                        18 : 'just_right',
                        19 : 'just_left',
                        20 : 'just_up',
                        21 : 'just_down',
                        22 : 'up_right',
                        23 : 'up_left',
                        24 : 'down_right',
                        25 : 'down_left',
                        26 : 'fast',
                        27 : 'slow',
                        28 : 'straight'}

class HRI_communication():


    """"""""""""""""""""""""
    """ CLASS  FUNCTIONS """
    """"""""""""""""""""""""


    ####################################################

    def __init__(self):

        self.settings = settings

        # define data structure and settings['headers']

        self._DEBUG = _DEBUG

        self.calib_maneuver_dict = calib_maneuver_dict


    ####################################################


    """"""""""""""""""""""""""""""
    """"" PRIVATE  FUNCTIONS """""
    """"""""""""""""""""""""""""""


    ####################################################


    def _import_dummy(self):

        self.mapp.import_data(which_user = 'test', clean = False)
        self.dummy_data = HRI.merge_data_df(self.mapp.motion_data_umprocessed['test'])[self.mapp.settings.init_values_to_remove:]


    ####################################################


    def _run_avatar(self):

        count = 0

        while count<settings['n_readings']:

            count += 1

            # update skeleton
            (skel) = self._read_motive_skeleton()

            flag = ''

            # check if unity flag
            unity_flag = self._acquire_unity_flag()

            # if flag : send skeleton
            if unity_flag=='r':

                if settings['debug']:
                    logging.debug('sending skeleton to UNITY')

                # send skeleton
                write_sk_to_unity(Write_unity_sk, unity_sk_client, skel)

            elif unity_flag=='q':

                # close unity write socket
                Read_unity_flag.socket.close()

                break


    ####################################################


    def _write_sk_to_unity(self, skel):

        skel_msg = np.reshape(skel[: , :-3], 21 * 8)
        arr = skel_msg.tolist()

        arr = arr + [float(count)]

        strs = ""
        # one int and 7 floats, '21' times

        for i in range(0, len(arr) // 4):
            if i % 8 == 0:
                strs += "i"
            else:
                strs += "f"

        logging.debug(arr)
        # logging.debug(len(arr))
        message = struct.pack('%sf' % len(arr), *arr)

        # logging.debug(message)
        self._udp_write(message)


    ####################################################


    """"""""""""""""""""""""
    """ PUBLIC FUNCTIONS """
    """"""""""""""""""""""""


    ####################################################


    def run(self, mode = None):
        """ runs acquisition/control depending on mode """

        run(mode)


########################################################
########################################################
########################################################

""""""""""""""""""""""""""""""
""""" PRIVATE  FUNCTIONS """""
""""""""""""""""""""""""""""""


def _acquire_input_data():
    """ acquires data from the input device (MoCap, remote, IMU...) specified in settings """
    
    logging.debug('collecting input data')

    if settings['input_device'] == 'motive':                                                    

        if settings['dummy_read']:                      # if you want a dummy msg
            # generates dummy skeleton
            input_data = []

            for i in range(settings['n_rigid_bodies_in_skeleton']):                     # one dummy rigid body per each rigid body in skeleton 
                # generates dummy rigid body
                one_rb = bytearray(struct.pack("ifffffff", i+1, 0, 0, 0, 0, 0, 0, 1))   # 1 int x ID, 3 float x pos, 4 float x quaternion, total length is 32
                input_data = input_data + one_rb if len(input_data) else one_rb         # concatenate

            logging.debug('acquired dummy skeleton')

        else:
            input_data = _read_motive_skeleton()

    elif settings['input_device'] == 'remote':

        if settings['dummy_read']:                      # if you want a dummy msg
            input_data = [0, 0, 0, 0]                   # generates dummy input (all zeros)
            logging.debug('acquired dummy remote')
        else:
            input_data = remote_handler.data

    elif settings['input_device'] == 'imu':

        if settings['dummy_read']:                      # if you want a dummy msg
            input_data = [0, 0, 0]                      # generates dummy input (all zeros)
            logging.debug('acquired dummy remote')
        else:
            input_data = _read_imu()

    if settings['input_device'] == 'imus':                                                    

        if settings['dummy_read']:                      # if you want a dummy msg
            # generates dummy imus
            input_data = []

            for i in range(len(settings['used_body_parts'])):                          # simulate two imus
                # generates dummy rigid body (works up to i=10)
                one_rb = bytearray(struct.pack("qqccccccccdddddddddd", 0, 0, '0'.encode(), '0'.encode(), '0'.encode(), '0'.encode(), '0'.encode(), '0'.encode(), '0'.encode(), str(i).encode(), 0, 0, 0, 0, 0, 0, 0, 0, 0, 1))   # 104 values
                input_data = input_data + one_rb if len(input_data) else one_rb         # concatenate

            logging.debug('acquired dummy imu set')

        else:
            input_data = _read_imus()

    if _timeout(input_data):
        logging.debug(settings['input_device'] + ' acquisition timeout')

    logging.debug('input data = ' + str(input_data))

    return input_data


########################################################


def _control_routine(input_data_num, mapp, control_history_raw_num, control_history_num, count):
    """ processes input and sends command data to unity/hardware """

    if input_data_num is 't':
            raise NameError('No data from input device!')

    # import dummy data (if required)
    if not settings['control_from_dummy_data']:
        _DEBUG['input_data_num_unproc'] = input_data_num    ### TODO : store in list using count
        # skeleton data from binary to list
        if settings['input_device'] == 'motive':
            input_data_num = _process_motive_skeleton(input_data_num)
            # input_data = 
        elif settings['input_device'] == 'imus':
            input_data_num = _process_imus(input_data_num)

    
    ### TODO :  and make this part a function of the InputData class (simply 'skeleton.process')

    _DEBUG['input_data_num'] = input_data_num

    if settings['input_device'] == 'motive':
        if settings['control_from_dummy_data']: ### TOFIX
            # input_data = user_data.skeleton(np.reshape(input_data_num[settings['headers']['motive']].values, (settings['n_rigid_bodies_in_skeleton'],-1)))
            input_data = dict(zip(settings['headers']['motive'], input_data_num))
        else:
            # input is a motive skeleton
            input_data = user_data.skeleton(input_data_num)

        # store in history input data array
        control_history_raw_num[count] = input_data.values

    if settings['input_device'] == 'imus':
        if settings['control_from_dummy_data']: ### TOFIX
            pass
            # input_data = user_data.skeleton(np.reshape(input_data_num[settings['headers']['motive']].values, (settings['n_rigid_bodies_in_skeleton'],-1)))
        else:
            # input is a imu set
            input_data = user_data.imus(np.resize(input_data_num, [len(settings['used_body_parts']), settings['n_elements_per_imu']]))

        # store in history input data array
        control_history_raw_num[count] = input_data.values
        
    elif settings['input_device'] == 'remote':

        # input is a remote read
        input_data = user_data.remote(input_data_num)
        input_data = input_data.values if input_data.values is not None else np.array([128, 128, 128, 128]) # if input is None read a dummy
        # store in history input data array
        control_history_raw_num[count] = input_data

    elif settings['input_device'] == 'imu':

        # input is an imu read
        input_data = user_data.imu(input_data_num)
        input_data = input_data.values if input_data.values is not None else np.array([0, 0, 0]) # if input is None read a dummy
        # store in history input data array
        control_history_raw_num[count] = input_data

    if settings['input_device'] == 'motive':

        # first skeleton preprocessing
        skel = _skeleton_preprocessing(input_data)

        if settings['control_style'] == 'simple':

            # get only torso pitch and roll
            angles_scaled = - skel.values[0, 8:11]/np.pi * 2 # minus sign because angles are reversed
            # scaling values (coming from Miehlbradt's paper)
            y_score_scaled = np.array([angles_scaled[0] - 2*angles_scaled[2], angles_scaled[1]])

            logging.debug('torso pitch and roll = ' + str(skel.values[0, 8:11]))
        else:
            # second skeleton processing
            commands_tofit = _skeleton_preprocessing_II(skel, mapp)

    if settings['input_device'] == 'imus':

        # first skeleton preprocessing
        skel = _skeleton_preprocessing(input_data)

        if settings['control_style'] == 'simple':

            # get only torso pitch and roll
            angles_scaled = - skel.values[0, 8:11]/np.pi * 2 # minus sign because angles are reversed
            # scaling values (coming from Miehlbradt's paper)
            y_score_scaled = np.array([angles_scaled[0] - 2*angles_scaled[2], angles_scaled[1]])

            logging.debug('torso pitch and roll = ' + str(skel.values[0, 8:11]))
        else:
            # second skeleton processing
            commands_tofit = _skeleton_preprocessing_II(skel, mapp)

    elif settings['input_device'] == 'remote':

        _DEBUG['input_raw'] = input_data

        if settings['control_style'] == 'simple':
            
            # reading inputs 0 and 1
            controls_raw = np.array(input_data[-1:-3:-1])   
            # scaling factors for [### TODO : check] remote
            controls_av = np.array([120, 124])
            controls_range = np.array([107, 114])
            # scaling values based on [### TODO : check] remote
            y_score_scaled = (controls_raw - controls_av)/(controls_range/settings['remote_gain'])

        else:

            # remote processing
            commands_tofit = _remote_preprocessing_II(input_data, mapp)

    elif settings['input_device'] == 'imu':

        _DEBUG['input_raw'] = input_data

        if settings['control_style'] == 'simple':
            
            # reading inputs 0 and 1
            controls_raw = np.array(input_data[-1:-3:-1])      
            # scaling factors for imu
            controls_av = np.array([0, 0, 0])
            controls_range = np.array([360, 360, 360])
            # scaling values based imu factors
            y_score_scaled = (controls_raw - controls_av)/(controls_range/settings['imu_gain'])

        else:

            # imu processing
            commands_tofit = _imu_preprocessing_II(input_data, mapp)

    if not settings['control_style'] == 'simple':

        if settings['control_style'] == 'maxmin':

            # process using linear regression
            y_score_scaled = _maxmin(commands_tofit, mapp)
        
        elif settings['control_style'] == 'new':

            # process using nonlinear regression
            y_score_scaled = _new(commands_tofit, mapp)
    
    controls = np.zeros(N_FLOATS_UDP) # longer than longer input

    for i, out in enumerate(settings['regression_outputs']):
    # add zero values for unity (expecting 5 floats)
        controls[i] = y_score_scaled[out]
    controls = controls.tolist() # comment for HW ### TODO : make option
    # store in history input data array ### TODO : do outside of function
    control_history_num[count] = controls
    # send commands to unity
    _write_commands_to_unity(controls)

    return control_history_raw_num, control_history_num


########################################################


def _create_hri_folders():
    """ create all the needed folders to store data """

    HRI.create_dir_safe(settings['data_folder'])
    HRI.create_dir_safe(settings['subject_folder'])
    HRI.create_dir_safe(settings['control_folder'])
        
        
########################################################


def _acquire_unity_flag():
    """ read and process flag from unity """

    # read and process flag
    flag = udp.udp_read(udp.sockets['read_unity_flag'], keep_last = False)
    flag = _process_unity_flag(flag)

    logging.debug('UNITY flag = ' + flag)

    return flag


########################################################


def _import_mapping(is_struct = False):
    """ imports results from mapping procedure """

    def extract_parameters(normalization_values, used_body_parts, outputs, outputs_no_pll):
        """ processes parameters from the given mapping variable """

        param = {}

        if settings['input_device'] == 'motive' or settings['input_device'] == 'imus':
            # gets the normalization values from [normalization_values] based on the given features
            feats = HRI.select_motive_features(settings, feat_names)
            param['normalization_values'] = normalization_values.iloc[:,:-2][feats]
            # reshape on the correct form based on the acquisition hardware
            parameters_val = param['normalization_values'].values

            param['norm_av'] = parameters_val[0,:].reshape(len(settings['used_body_parts']),-1)
            param['norm_std'] = parameters_val[1,:].reshape(len(settings['used_body_parts']),-1)

        elif settings['input_device'] == 'remote':
            # gets the normalization values from [normalization_values] based on the given features
            param['normalization_values'] = normalization_values.drop(outputs + outputs_no_pll, axis=1)
            # reshape on the correct form based on the acquisition hardware
            parameters_val = param['normalization_values'].values

            param['norm_av'] = parameters_val[0,:].reshape(4,-1)
            param['norm_std'] = parameters_val[1,:].reshape(4,-1)     # 4 inputs from remote

        elif settings['input_device'] == 'imu':
            # gets the normalization values from [normalization_values] based on the given features
            param['normalization_values'] = normalization_values.drop(outputs + outputs_no_pll, axis=1)
            # reshape on the correct form based on the acquisition hardware
            parameters_val = param['normalization_values'].values

            param['norm_av'] = parameters_val[0,:].reshape(3,-1)
            param['norm_std'] = parameters_val[1,:].reshape(3,-1)     # 3 inputs from imu

        return param


    if is_struct:

        mapp = {}

        # load mapping
        mapp_temp = HRI_mapping.HRI_mapping()           # in this case, mapp_temp is a class HRI_mapping.HRI_mapping()
        mapp_temp = HRI.load_obj(os.path.join(settings['subject_folder'], settings['control_style']))
        # puts data in dict
        mapp['parameters'] = extract_parameters(mapp_temp.param.normalization_values, mapp_temp.settings.used_body_parts, mapp_temp.settings.outputs, mapp_temp.settings.outputs_no_pll)    # processes parameters from class HRI_mapping.HRI_mapping()
        
        mapp['features'] = mapp_temp.settings.feats_reduced
        mapp['test_info'] = mapp_temp.test_info
        mapp['test_results'] = mapp_temp.test_results

    else:

        # load mapping
        mapp =  HRI.load_obj(os.path.join(settings['subject_folder'], '{}_{}'.format(settings['input_device'], settings['control_style'])))
        # puts data in dict
        mapp['parameters'] = extract_parameters(mapp['parameters'], mapp['settings']['used_body_parts'], mapp['settings']['outputs'], mapp['settings']['outputs_no_pll'])

    return mapp


########################################################


def _imu_preprocessing_II(input_data, mapp):
    """ normalizes and applies dimensionality reduction to imu data """

    # get input values in np array
    controls_raw = np.array(input_data)      # reading inputs 0 and 1

    # normalize
    [controls_norm, _] = utils.normalize(controls_raw, [mapp['parameters']['norm_av'], mapp['parameters']['norm_std']])

    logging.debug(controls_norm)

    # store in a dictonary
    controls_dict = {'roll_imu' : controls_norm[0],
                     'pitch_imu' : controls_norm[1],
                     'yaw_imu' : controls_norm[2]}

    # get dim_reduced data
    remote_tofit = np.array([controls_dict[x] for x in mapp['features']])
    remote_tofit = remote_tofit.reshape(1, -1)

    return remote_tofit


########################################################


def _maxmin(skel_tofit, mapp):
    """ performs linear regression on input data """
    
    y_score_scaled = {}
    skel_tofit_red = {}

    for out in settings['regression_outputs']:
        
        # dimensionality reduction
        dim_red = mapp['dimred']
        skel_tofit_red[out] = HRI_mapping.transform_cca(skel_tofit[out], dim_red[out])

        # linear regression 
        # minmax_map = mapp['test_info']['maxmin_map']
        # y_score = HRI_mapping._predict_maxmin(skel_tofit_red, minmax_map)   # fit
        best_mapping = mapp['test_info'][out]['results'][mapp['test_info'][out]['best']]['reg']
        y_score = best_mapping.predict(skel_tofit_red[out])

        # scale data
        y_score_scaled[out] = y_score.flatten()

    return y_score_scaled


########################################################


def _new(skel_tofit, mapp):
    """ performs nonlinear regression on input data """


    # dimensionality reduction
    dim_red = mapp['test_info']['dim_red']
    skel_tofit_red = HRI_mapping.transform_cca(skel_tofit, dim_red)

    logging.debug('')
    logging.debug('')
    logging.debug('full = ', skel_tofit_red)

    # clamp between min and max of calibration
    max_inputs = mapp['test_info']['max_values']
    min_inputs= mapp['test_info']['min_values']

    for count, i in enumerate(skel_tofit_red[0]):

        logging.debug(i)

        if skel_tofit_red[0][count] > max_inputs[count]:
            skel_tofit_red[0][count] = max_inputs[count]
        elif skel_tofit_red[0][count] < min_inputs[count]:
            skel_tofit_red[0][count] = min_inputs[count]

    logging.debug('clipped = ', skel_tofit_red)
    logging.debug('')
    logging.debug('')

    # nonlinear regression 
    best_mapping = mapp['test_results'][mapp['test_info']['best']]['reg']
    y_score = best_mapping.predict(skel_tofit_red)

    # scale data
    y_score_scaled = y_score.flatten()/90.0

    return y_score_scaled


########################################################

previous_imu_data = None
THRESHOLD = 180
MINI_THRESHOLD = 1
DEG_CORRECTION = 360

def _process_imu(data):

    if _timeout(data):
        return data

    # print("Byte Length of Message :", len(data), "\n")

    how_many = int(len(data)/64)

    strs = "dddd"#*how_many

    data_ump = struct.unpack(strs, data)[1:4]


    ### check if they changed "too much"
    ### MAKE SURE THAT THIS IS NOT DUE TO A RESET
    global previous_imu_data
    if previous_imu_data is not None:
        for idx, i in enumerate(unity_control):
            if abs(i-previous_imu_data[idx]) > THRESHOLD: #check treshold
                tmp = list(unity_control)
                if abs(i-0) > MINI_THRESHOLD: #enter this statement only if not reset
                    DEG_CORRECTION = abs(i-previous_imu_data[idx])
                    if i-previous_imu_data[idx] > 0:
                        tmp[idx] = -DEG_CORRECTION + unity_control[idx]
                    else:
                        tmp[idx] = DEG_CORRECTION + unity_control[idx]
                unity_control = tmp

    previous_imu_data = unity_control
    prev = struct.unpack(strs,Read_struct.previous_state)[1:4]
    unity_control = struct.unpack(strs, data)[1:4]

    return unity_control


########################################################


def _process_motive_skeleton(data):

    # print(data)

    if _timeout(data):
        return data

    strs = ""

    # one int and 7 floats, '21' times

    for i in range(0, len(data) // 4):
        if i % 8 == 0:
            strs += "i"
        else:
            strs += "f"

    data_ump = struct.unpack(strs, data)

    # Q_ORDER = [3, 0, 1, 2]
    Q_ORDER = [0, 1, 2, 3] # apparently natnet is now streaming [qx, qy, qy, qw]

    for i in range(0, len(data_ump) // settings['n_data_per_rigid_body']):
        bone = list(data_ump[i*settings['n_data_per_rigid_body'] : (1+i)*settings['n_data_per_rigid_body']])

        # print bone

        if i == 0:
            ID = [(int(bin(bone[0])[-8:], 2))]
            position = np.array(bone[1:4])
            quaternion_t = np.array(bone[4:])
            quaternion = np.array([quaternion_t[j] for j in Q_ORDER])
        else:
            ID = ID + [(int(bin(bone[0])[-8:], 2))]
            position = np.vstack((position, bone[1:4]))
            quaternion_t = bone[4:]
            quaternion = np.vstack((quaternion, [quaternion_t[j] for j in Q_ORDER]))

    ID = np.array(ID)

    data = np.c_[ID, position, quaternion]

    # sort by ID
    data = data[data[:, 0].argsort()]

    # print(data)

    return data


########################################################


def _process_imus(data):

    if _timeout(data):
        return data

    str_base = settings['types_in_imus']
    str_l = len(str_base)

    how_many = int(len(data)/settings['n_bytes_per_imu'])
    strs = str_base*how_many
    data_ump = struct.unpack(strs, data)
    imus = []

    for i in range(how_many):
        imus.append({})

        idx = str_l*i

        imus[-1]['ts'] = data_ump[idx+0]
        imus[-1]['ID'] = settings['imu_ID'][(data_ump[idx+2].decode("utf-8") + data_ump[idx+3].decode("utf-8") + data_ump[idx+4].decode("utf-8") + data_ump[idx+5].decode("utf-8") + data_ump[idx+6].decode("utf-8") +  data_ump[idx+7].decode("utf-8") + data_ump[idx+8].decode("utf-8") + data_ump[idx+9].decode("utf-8")).lower()]
        imus[-1]['Euler'] = data_ump[idx+13:idx+16]
        imus[-1]['Quat'] = data_ump[idx+16:idx+20]

        

    ids = [x['ID'] for x in imus]

    data = []

    for id in ids:
        data = data + [imus[id-1]['ID']] + list(imus[id-1]['Euler']) + list(imus[id-1]['Quat'])

    # sort by ID
    data = np.array(data)

    return data


########################################################


def _process_input_data(input_data, unity_num):
    """ postprocessing of acquired data (received as binary) """

    ### HANDLING INPUT DATA ###

    # timer for debugging
    start = time.clock()

    # delete empty rows
    input_data = [x for x in input_data if x is not None]

    # if acquired from motive
    if settings['input_device'] == 'motive':

        # create empty numpy array to store data
        input_data_num = np.empty([len(input_data), settings['n_rigid_bodies_in_skeleton']*settings['n_elements_in_rigid_body']])   # 1 skel = [n_rigid_bodies_in_skeleton] * [n_elements_in_rigid_body] (see header for details)

        for i in range(0, len(input_data)):

            # process list of binaries into numpy array
            skel_np_t = _process_motive_skeleton(input_data[i])

            skel_np_t.resize(1, skel_np_t.size)

            input_data_num[i] = skel_np_t

            logging.debug('input processed ' + str(i) + ' of ' + str(len(input_data)))

    # if acquired from remote
    if settings['input_device'] == 'remote':

        # create empty numpy array to store data
        input_data_num = np.empty([len(input_data), 4]) # 1 remote = 4 values (see header for details)
        # store list of values into numpy array
        input_data_num =  np.array(input_data)

    # if acquired from imu
    if settings['input_device'] == 'imu':

        # create empty numpy array to store data
        input_data_num = np.empty([len(input_data), 3]) # 1 imu = 3 values (see header for details)
        # store list of values into numpy array
        input_data_num =  np.array(input_data)

    logging.debug('input processed in ' +  str(time.clock() - start))

    # if acquired from imus
    if settings['input_device'] == 'imus':

        # create empty numpy array to store data
        n_imus = int(len(input_data[0])/settings['n_bytes_per_imu'])
        n_data = int(n_imus * settings['n_elements_per_imu'])
        input_data_num = np.empty([len(input_data), n_data])   # 1 skel = [n_rigid_bodies_in_skeleton] * [n_elements_in_rigid_body] (see header for details)

        global _DEBUG
        _DEBUG['n_imus'] = n_imus
        
        for i in range(0, len(input_data)):

            # process list of binaries into numpy array
            imus_np_t = _process_imus(input_data[i])
            imus_np_t.resize(1, imus_np_t.size)

            input_data_num[i] = imus_np_t

            logging.debug('input processed ' + str(i) + ' of ' + str(len(input_data)))

    ### HANDLING UNITY DATA ###

    # delete empty rows
    unity_data = [x for x in unity_num if x is not None]

    # create empty numpy array to store data
    unity_num = np.empty([len(unity_data), len(unity_data[0])])      # normally 32 values (see header for details)

    # timer for debugging
    start = time.clock()
    # store list of values into numpy array
    unity_num =  np.array(unity_data)

    logging.debug('unity data processed in ' +  str(time.clock() - start))

    return input_data, input_data_num, unity_data, unity_num


########################################################


def _process_unity_calib(data):

    if _timeout(data):
        return data

    # print("Byte Length of Message :", len(data), "\n")
    strs = ""
    for i in range(0, len(data)//4):
        strs += "f"

    # print(strs)
    # print(len(data))

    unity_control = struct.unpack(strs, data)
    # print("Message Data :", unity_control, "\n")
    return unity_control


########################################################


def _process_unity_flag(data):

    if _timeout(data):
        return data

    # we receive a char
    flag = data.decode("utf-8")

    return flag


########################################################


def _read_imu():
    """ acquires data from imu """

    # acquire imu data
    imu_data_temp = udp.udp_read(udp.sockets['read_imu'], keep_last = True)

    # process online if not timeout
    if not _timeout(imu_data_temp):
        imu_data_temp = _process_imu(imu_data_temp)

    return imu_data_temp


########################################################


def _read_imus():
    """ acquires data from imu set """

    # acquire imu data
    imu_data_temp = udp.udp_read(udp.sockets['read_imus'], keep_last = True)

    # # process online if not timeout
    # if not _timeout(imu_data_temp):
    #     imu_data_temp = _process_imu(imu_data_temp)

    return imu_data_temp


########################################################


def _read_motive_skeleton():
    """ reads skeleton data from motive """
    return udp.udp_read(udp.sockets['read_motive_sk'])


########################################################


def _acquire_unity_data(unity_num, count):
    """ acquires data from the unity simulator """

    logging.debug('collecting unity data')
    # read and process unity calibration data
    unity_calib_data = udp.udp_read(udp.sockets['read_unity_control'])  
    unity_calib = np.array(_process_unity_calib(unity_calib_data))

    # if you want a dummy msg
    if settings['dummy_unity']:                   
        # generates dummy input (all zeros)                      
        unity_calib = np.array([0] * settings['headers']['unity'].size)    

        logging.debug('acquired dummy unity data')

    if _timeout(unity_calib):
        logging.debug('unity calib timeout')

    logging.debug('received unity calibration data')

    # store value in array
    if unity_calib.size == settings['headers']['unity'].size:       # if data match header size
        unity_num[count] = unity_calib                  # saves data
    else:
        unity_num[count] = unity_num[count - 1]         # saves the previous one
        logging.warning('header not matching data size')    # warning

    return unity_calib, unity_num


########################################################


def _remote_preprocessing_II(input_data, mapp):
    """ normalizes and applies dimensionality reduction to remote data """

    # get input values in np array
    controls_raw = np.array(input_data)      # reading inputs 0 and 1

    # normalize
    [controls_norm, _] = utils.normalize(controls_raw, [mapp['parameters']['norm_av'], mapp['parameters']['norm_std']])

    logging.debug(controls_norm)

    # store in a dictonary
    controls_dict = {'remote1' : controls_norm[0],
                     'remote2' : controls_norm[1],
                     'remote3' : controls_norm[2],
                     'remote4' : controls_norm[3]}

    # get dim_reduced data
    remote_tofit = np.array([controls_dict[x] for x in mapp['features']])
    remote_tofit = remote_tofit.reshape(1, -1)

    return remote_tofit


########################################################


def _run_acquisition(count = 0):
    """ run the acquisition routine until a 'q' (quit flag) is received from unity or count == settings['n_readings'] """

    last = time.clock()
    idle = 1    #stop reading if idle for 1 sec

    # initialize unity and input data arrays
    unity_num = [None] * MANY_DATA
    input_data = [None] * MANY_DATA

    logging.info('started acquisition')
    # acquisition stop flag
    stop = False

    while not stop:

        # acquire input data
        input_data_temp = _acquire_input_data()
        # check unity flag
        unity_flag = _acquire_unity_flag()

        # flag simulation
        if settings['simulate_flag']:   # simulate ready flag
            unity_flag = 'a'    
            time.sleep(0.1)
            
        now = time.clock()
        elapsed = now - last         

        # stop if acquiring limited number of data (useful for simulation/test)
        if count>=settings['n_readings']:
            logging.info('started acquisition (flag)')
            unity_flag='q'                             # simulate quit flag

        logging.info(elapsed)
        if (count>0 and elapsed>idle):
            logging.info('started acquisition (timeout)')
            unity_flag='q'                             # simulate quit flag

        logging.info(unity_flag)
        # if ready : read unity and skeleton
        if unity_flag=='a':      

            # check if idle
            last = time.clock()                      

            # save input_data in list
            input_data[count] = input_data_temp
            # read unity data
            unity_temp, unity_num = _acquire_unity_data(unity_num, count)
            # increment counter
            count += 1

            logging.debug('acquired one datapoint')

        # if ready : read unity and skeleton
        elif unity_flag=='q':

            # postprocessing of acquired data (received as binary)
            [input_data, input_data_num, _, unity_num] = _process_input_data(input_data, unity_num)

            # store acquire data to file
            [data, calib_data] = _store_acquisition_to_file(input_data, input_data_num, unity_num)
            # cose sockets
            close_sockets()

            logging.debug('finished acquisition')
            # stop loop
            stop = True

    return input_data, unity_num, data, calib_data


########################################################


def _run_control(mapp):
    """ run the control routine until a 'q' (quit flag) is received from unity or count == settings['n_readings'] """

    # initialize data lists for logging to file
    control_history_num = [None] * MANY_DATA
    control_history_raw_num = [None] * MANY_DATA
    unity_num = [None] * MANY_DATA

    # import dummy data (if required)
    if settings['control_from_dummy_data']: ### TOFIX
        files = os.listdir(settings['subject_folder'])
    
        files_new = [x for x in files if 'inst' in x]
        files_unprocessed = [x for x in files_new if 'CLEAN' not in x]

        dummy_data = pd.read_csv(os.path.join(settings['subject_folder'], files_unprocessed[0]))
        # l_desired = 2000
        # dummy_data = HRI_mapping.df_resample(dummy_data, l_desired)

        settings['n_readings'] = len(dummy_data)

    if settings['n_readings'] == np.inf:
        pass
        # settings['n_readings'] = len(self.dummy_data-1)
        

    # initialize counter
    count = 200
    # acquisition stop flag
    stop = False

    while not stop:

        if settings['control_from_dummy_data']:

            input_data = dummy_data.iloc[count]
            input_data = input_data[settings['headers']['motive']].values
        else:
            # acquire input data
            input_data = _acquire_input_data()
        
        # check unity flag if not simulated
        unity_flag = 'c' if settings['simulate_flag'] else _acquire_unity_flag()

        # if flag=='c' : read unity and input data, send commands to unity
        if unity_flag=='c':

            # acquire unity data
            _, unity_num = _acquire_unity_data(unity_num, count)
            # process input data and send commands to unity
            control_history_raw_num, control_history_num = _control_routine(input_data, mapp, control_history_raw_num, control_history_num, count)
            # increment counter
            count += 1

        # if flag=='q' : save to csv
        if unity_flag=='q' or count>=settings['n_readings']:

            # store data history to file
            _store_control_to_file(control_history_raw_num, control_history_num, unity_num)
            # closes UDP sockets
            close_sockets()

            HRI.save_obj(_DEBUG, os.path.join(settings['subject_folder'], 'control_debug'))

            # stop acquisition
            stop = True

        if count==settings['n_readings']:
            # stop acquisition
            stop = True


########################################################


def _select_features():
    """ returns the chosen subset of features for motive acquisitions """

    in_data = settings['features_used']

    if in_data == 'full':
        # returns everything
        feats = [col for col in list(settings['headers']['motive']) if '_' in col]
    elif in_data == 'angles':
        # only quaternions and euler angles
        feats = [col for col in list(settings['headers']['motive']) if 'quat' in col or 'roll_' in col or 'pitch_' in col or 'yaw_' in col]
    elif in_data == 'euler':
        # only euler angles
        feats = [col for col in list(settings['headers']['motive']) if 'roll_' in col or 'pitch_' in col or 'yaw_' in col]
    elif in_data == 'quaternions':
        # only quaternions
        feats = [col for col in list(settings['headers']['motive']) if 'quat' in col]

    return feats


########################################################

first_skel = None

def _skeleton_preprocessing(input_data):
    """ selects used body parts, relativizes, unbiases, and computes euler angles for skeleton  """

    skel_base  = input_data.copy()
    l = list(range(0, len(settings['used_body_parts'])))
    l.reverse()     # need to reverse to respect kinematic chain

    for i in l:
        input_data = HRI_mapping.relativize_angles_dict(settings['used_body_parts'][i], settings['kinematic_chain'][i], input_data)

    skel_rel = input_data.copy()

    # uses first skeleon to unbias current skeleton (removes first pose)
    global first_skel

    if first_skel is None:
        # if no first skeleton so far, saves current skeleton
        first_skel = input_data.copy()
    
    for i in settings['used_body_parts']:
        input_data = HRI_mapping._unbias_dict(i, input_data, first_skel)
    skel_unb = input_data.copy()

    for i in settings['used_body_parts']:
        input_data = HRI_mapping._compute_euler_angles_dict(i, input_data)
    skel_eul = input_data.copy()
    

    # used for debug
    _DEBUG['skel_base'] = skel_base
    _DEBUG['skel'] = np.copy(input_data)
    _DEBUG['skel_rel'] = skel_rel
    _DEBUG['skel_first'] = first_skel
    _DEBUG['skel_unb'] = skel_unb
    _DEBUG['skel_eul'] = skel_eul
    
    return input_data


########################################################


def acquire_store_init_pose():

    timeout = True

    while timeout:
        data = _read_imus()
        timeout = _timeout(data)

    data_proc = _process_imus(data)

    HRI.save_obj(data_proc, os.path.join(settings['subject_folder'],'init_pose'))


########################################################


def get_init_pose():

    return HRI.load_obj(os.path.join(settings['subject_folder'],'init_pose'))


########################################################

first_skel = None
init_skel = None

def _imus_preprocessing(skel):
    """ selects used body parts, relativizes, unbiases, and computes euler angles for skeleton  """

    # uses first skeleon to unbias current skeleton (removes first pose)
    global init_skel
    global first_skel

    if init_skel is None:
        # if no first skeleton so far, saves current skeleton
        init_skel = user_data.imus(np.resize(get_init_pose(), [len(settings['used_body_parts']), settings['n_elements_per_imu']]))
        # init_skel = user_data.imus(np.resize(get_init_pose(), [len(settings['used_body_parts']), settings['n_elements_per_imu']]))
        skel.assign_init_skel(init_skel)
    else:
        # else uses global value, precedently saved
        skel.assign_init_skel(init_skel)

    # initialize angles
    skel.init_pose()

    # compute relative angles
    skel.relativize()

    if first_skel is None:
        # if no first skeleton so far, saves current skeleton
        first_skel = user_data.skeleton(np.copy(skel.values))
        skel.assign_first_skel(first_skel)
    else:
        # else uses global value, precedently saved
        skel.assign_first_skel(first_skel)

    # unbias angles
    skel.unbias()
    
    # compute euler angles
    skel.compute_ea()
    
    return skel


########################################################


def _skeleton_preprocessing_II(skel, mapp):
    """ normalizes and applies dimensionality reduction to skeleton """

    used_feats = ['{}_{}'.format(x,y) for y in settings['used_body_parts'] for x in ['roll', 'pitch', 'yaw']]

    skel_t = [skel[x] for x in used_feats]

    skel_subselect = np.array(skel_t)

    skel = (skel_subselect-mapp['parameters']['norm_av'].reshape(-1))/mapp['parameters']['norm_std'].reshape(-1)

    skel = dict(zip(used_feats, skel))
    skel_norm = skel.copy()

    # dimensionality reduction
    if settings['dim_reduction']:

        if settings['dim_reduction_type'] == 'bones':
            # # keeps only used body parts
            # skel.keep_used_body_parts(mapp['features'])
            # skel_norm = np.copy(skel.values)
            # # subselects features from body parts
            # skel.keep_features(settings['features_used'])
            # skel_tofit = np.copy(skel.values.reshape(1, -1))
            pass

        elif settings['dim_reduction_type'] == 'signals':
            # keeps only used features

            skel_reg = {}
            skel_tofit = {}
            
            for out in settings['regression_outputs']:
                
                skel_reg[out] = np.array([skel[x] for x in mapp['features'][out]])

                skel_tofit[out] = skel_reg[out].reshape(1, -1)

    _DEBUG['skel_norm'] = skel_norm
    _DEBUG['skel_tofit'] = skel_tofit

    return skel_tofit


########################################################


def _start_remote_acquisition():
    """ start acquisition of remote data in separate thread """

    # instanciate class [remote_handler]
    remote_hand = remote_handler.remote_handler()

    # select the right remote
    if settings['remote_ID'] == 'Logitech':
        remote_hand.desired_device = remote_handler.LOGITECH_510
    elif  settings['remote_ID'] == 'HobbyKing':
        remote_hand.desired_device = remote_handler.HOBBYKING

    # read data in background
    remote_hand.connect()
    remote_hand.read_background()

    return remote_hand


########################################################


def _stop_remote_acquisition(remote_hand):
    """ stop acquisition of remote data in separate thread """

    # stop reading data in background
    remote_hand.stop_read_background
    remote_hand.close_device()

    return remote_hand


########################################################


def _store_acquisition_to_file(input_data, input_data_num, unity_num):
    """ postprocess data, define filename and store the acquired data locally """

    # bugfix different length (not very elegant)
    while len(input_data_num) > len(unity_num):
        input_data_num = input_data_num[:-1]
    while len(input_data_num) < len(unity_num):
        unity_num = unity_num[:-1]

    # pick right header for data
    data = settings['headers']['calib']

    # contactenate header and data
    calib_data = np.c_[input_data_num, unity_num]
    acquired_data = np.vstack([data, calib_data])

    # list maneuvers and names file accordingly
    maneuvers_list = np.unique(acquired_data[1:, -5])

    if len(maneuvers_list)==1:
        info_maneuver =  calib_maneuver_dict[int(calib_data[-1, -5])]
    else:
        info_maneuver = 'mixed'

    # get info from file to define new filename
    info_period = str(int(calib_data[-1, -4]))
    info_amplitude = str(int(calib_data[-1, -3]*100))
    instance = str(int(calib_data[-1, -2]))
    
    end_of_filename = info_maneuver + '_period_' + info_period + '_amplitude_' + info_amplitude

    # create folders in case they don't exist
    _create_hri_folders()
    # create new folder for this subject in case it doesn't exist
    # folder = '[subject_name]_[input_device]'
    folder_save = os.path.join(settings['data_folder'], HRI.file_name(settings))
    HRI.create_dir_safe(folder_save)

    # save data to txt file
    # title = '[maneuver]_period_[period]_amplitude_[amplitude]_[timestamp]'
    if calib_data.shape[0]>1 and calib_data.shape[1]>3:
        filename = end_of_filename + '_inst_' + instance + '_' + datetime.datetime.now().strftime("%Y_%b_%d_%I_%M_%S%p")
    elif data.shape[0]==1:
        logging.warning('no data acquired')
    else:
        logging.warning('problem with data size')

    np.savetxt(os.path.join(folder_save, filename + '.txt'), (acquired_data), delimiter=",", fmt="%s")

    return data, calib_data


########################################################


def _store_control_to_file(control_history_raw_num, control_history_num, unity_num):
    """ stores unity and input control history to file """

    # timer for debug
    t = datetime.datetime.now().strftime("%Y_%b_%d_%I_%M_%S%p")

    # remove empty rows
    control_history_raw_num = [np.array(x).flatten() for x in control_history_raw_num if x is not None]
    control_history_num = [x for x in control_history_num if x is not None]
    unity_num = [x for x in unity_num if x is not None]

    # add settings['headers']
    # control_history_raw = np.vstack([settings['headers']['history'], np.array(control_history_raw_num)])
    control_history = np.vstack([settings['headers']['control_history'], control_history_num])
    # unity_history = np.vstack([settings['headers']['unity'], np.array(unity_num)])

    # store to file
    # np.savetxt(os.path.join(settings['control_folder'], settings['subject_name'] + '_' + settings['input_device'] + '_control_history_raw' + '_' + str(t) + '.txt'), (control_history_raw), delimiter=",", fmt="%s")
    np.savetxt(os.path.join(settings['control_folder'], settings['subject_name'] + '_' + settings['input_device'] + '_control_history' + '_' + str(t) + '.txt'), (control_history), delimiter=",", fmt="%s")
    # np.savetxt(os.path.join(settings['control_folder'], settings['subject_name'] + '_' + settings['input_device'] + '_unity_history' + str(t) + '.txt'), (unity_history), delimiter=",", fmt="%s")

    logging.debug('stored data in ' + str(time.time() - t) +  ' s')


########################################################


def _timeout(data):
    """ check if communication timed out """
    return data is 't'


########################################################


def _write_commands_to_unity(commands):
    """ send commands to unity through UDP """

    # list to np array
    arr = np.array(commands)

    logging.debug(type(arr))
    logging.info('sending ' + str(arr.tolist()))
    logging.debug(arr)
    logging.debug(len(arr))

    # initializes packing string
    strs = ""

    # updates packing string (all floats = one 'f' every 4 bytes)
    for i in range(0, len(arr) // 4):
            strs += "f"

    # packs array into binary string
    message = struct.pack('%sf' % len(arr), *arr)
    # sends to unity
    udp.udp_write(udp.sockets, message)


########################################################


""""""""""""""""""""""""""""""
"""""  PUBLIC FUNCTIONS  """""
""""""""""""""""""""""""""""""


########################################################


def close_sockets():
    """ closing sockets for reuse """
    udp.close_sockets()


########################################################


def run(mode, old_mapping = False):
    """ run the routine defined by [mode] returns 0 if no errors """

    # setup sockets
    setup_sockets()


    # start remote acquisition, if required
    if settings['input_device'] == 'remote':
        remote_hand = _start_remote_acquisition()

    if mode == 'avatar': ### TODO

        # run avatar routine
        _run_avatar()

    if mode == 'acquisition':

        if HRI.hri_state(settings) != 'NO DATA':
            logging.error('Data already present for this subject/input/instance! Delete existing data or change instance.')
            close_sockets()
            return 1

        # run acquisition routine
        _run_acquisition()

    if mode == 'control':

        mapp = _import_mapping(is_struct = old_mapping)

        # run contol routine
        _run_control(mapp)

    # stop remote acquisition, if required
    if settings['input_device'] == 'remote':
        remote_hand = _stop_remote_acquisition(remote_hand)
            
    close_sockets()
    return 0


########################################################


def setup_sockets():
    """ setup of used sockets """
    udp.setup_sockets()


########################################################


def update_settings(settings_in):
    """ update settings """
    global settings
    settings = settings_in


########################################################


def delete_existing_data():
    """" Deletes folder with existing previous data """

    fol = os.path.join(settings['data_folder'], HRI.file_name(settings))
    if os.path.exists(fol):
        shutil.rmtree(fol, ignore_errors=True)
        logging.info('Deleted folder {0}'.format(fol))


########################################################


def _extract_from_mixed():
    """" Deletes folder with existing previous data """

    if settings['input_device']=='motive':
        cal = CalibrationDataset.MotiveDataset()
    elif settings['input_device']=='remote':
        cal = CalibrationDataset.RemoteDataset()
    elif settings['input_device']=='imu':
        cal = CalibrationDataset.ImuDataset()

    cal.update_settings(get_settings())
        
    cal.import_data()

    cal.extract_from_mixed()


if __name__ == '__main__':

    if 'comm' in locals():
        comm.close_sockets()

    import HRI_communication as comm

    # run(mode = 'control')
    run(mode = 'acquisition')