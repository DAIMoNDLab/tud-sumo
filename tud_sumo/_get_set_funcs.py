import traci
from copy import copy
from .utils import *

# --- NETWORK ---

def _get_interval_network_data(self, data_keys: list | tuple, n_steps: int, interval_end: int = 0, get_avg: bool=False) -> float | dict:
    """
    Returns network-wide vehicle data in the simulation during range (`curr step - n_step - interval_end -> curr_step - interval_end`).
    Valid data keys are; '_tts_', '_twt_', '_delay_', '_no_vehicles_', '_no_waiting_' and '_to_depart_'. By default, all values are
    totalled throughout the interval unless `get_avg == True`. If multiple data keys are given, the resulting data is returned in a
    dictionary by each key.
    
    Args:
        `data_keys` (str, list): Data key or list of keys
        `n_steps` (int): Interval length in steps (max at number of steps the simulation has run)
        `interval_end` (int): Steps since end of interval (`0 = current step`)
        `get_avg` (bool): Denotes whether to return the step average delay instead of the total

    Returns:
        float: Total delay in seconds
    """
    
    if self._all_data == None:
        desc = "No data to return (simulation likely has not been run, or data has been reset)."
        raise_error(SimulationError, desc, self.curr_step)
    elif n_steps + interval_end > self._all_data["end"] - self._all_data["start"]:
        desc = "Not enough data (n_steps '{0}' + interval_end '{1}' > '{2}').".format(n_steps, interval_end, self._all_data["end"] - self._all_data["start"])
        raise_error(ValueError, desc, self.curr_step)
    elif not isinstance(n_steps, int):
        desc = "Invalid n_steps '{0}' (must be int, not '{1}').".format(n_steps, type(n_steps).__name__)
        raise_error(TypeError, desc, self.curr_step)
    elif n_steps <= 1:
        desc = "Invalid n_steps '{0}' (must be >1).".format(n_steps)
        raise_error(ValueError, desc, self.curr_step)
    
    if not isinstance(data_keys, (list, tuple)): data_keys = [data_keys]

    all_data = {}
    for data_key in data_keys:

        valid_keys = ["tts", "twt", "delay", "no_vehicles", "no_waiting", "to_depart"]
        error, desc = test_valid_string(data_key, valid_keys, "data key")
        if error != None: raise_error(error, desc)

        if interval_end <= 0: data_val = self._all_data["data"]["vehicles"][data_key][-n_steps:]
        else: data_val = self._all_data["data"]["vehicles"][data_key][-(n_steps + interval_end):-interval_end]
        
        data_val = sum(data_val)
        if get_avg: data_val /= len(data_val)

        all_data[data_key] = data_val
    
    if len(data_keys) == 1: return all_data[data_keys[0]]
    else: return all_data


# --- DETECTORS ---

def _get_last_step_detector_vehicles(self, detector_ids: str | list | tuple, vehicle_types: list | None = None, flatten: bool = False) -> dict | list:
    """
    Get the IDs of vehicles that passed over the specified detectors.
    
    Args:
        `detector_ids` (str, list, tuple): Detector ID or list of IDs (defaults to all)
        `vehicle_types` (list, optional): Included vehicle types
        `flatten` (bool): If true, all IDs are returned in a 1D array, else a dict with vehicles for each detector
    
    Returns:
        (dict, list): Dict or list containing all vehicle IDs
    """

    detector_ids = [detector_ids] if not isinstance(detector_ids, (list, tuple)) else detector_ids
    if len(detector_ids) == 1: flatten = True
    vehicle_types = [vehicle_types] if vehicle_types != None and not isinstance(vehicle_types, (list, tuple)) else vehicle_types

    vehicle_ids = [] if flatten else {}
    for detector_id in detector_ids:
        
        if detector_id not in self.available_detectors.keys():
            desc = "Detector ID '{0}' not found.".format(detector_id)
            raise_error(KeyError, desc, self.curr_step)
        
        detected_vehicles = self.get_detector_vals(detector_id, "vehicle_ids")
        
        if vehicle_types != None:
            detected_vehicles = [vehicle_id for vehicle_id in detected_vehicles if self.get_vehicle_vals(vehicle_id, "type") in vehicle_types]

        if flatten: vehicle_ids += detected_vehicles
        else: vehicle_ids[detector_id] = detected_vehicles

    if flatten: vehicle_ids = list(set(vehicle_ids))

    return vehicle_ids

def _get_detector_vals(self, detector_ids: list | tuple | str, data_keys: str | list) -> int | float | dict:
    """
    Get data values from a specific detector (_Multi-Entry-Exit (MEE)_ or _Induction Loop (IL)_) using a list
    of data keys. Valid data keys are; '_type_', '_position_', '_vehicle_count_', '_vehicle_ids_', '_lsm_speed_',
    '_halting_no (MEE only)_', '_lsm_occupancy (IL only)_', '_last_detection (IL only)_', '_avg_vehicle_length_' (IL only).

    Args:
        `detector_ids` (list, tuple, str): Detector ID or list of IDs
        `data_keys` (str, list): Data key or list of keys
    
    Returns:
        (dict, str, list, int, float): Values by `data_key` (or single value)
    """

    all_data_vals = {}
    if isinstance(detector_ids, str): detector_ids = [detector_ids]
    detector_ids = validate_list_types(detector_ids, str, param_name="detector_ids", curr_sim_step=self.curr_step)

    for detector_id in detector_ids:
        if detector_id not in self.available_detectors.keys():
            desc = "Detector with ID '{0}' not found.".format(detector_id)
            raise_error(KeyError, desc, self.curr_step)
        else:
            detector_type = self.available_detectors[detector_id]["type"]

            match detector_type:
                case "multientryexit": d_class = traci.multientryexit
                case "inductionloop": d_class = traci.inductionloop
                case "_":
                    desc = "Only 'multientryexit' and 'inductionloop' detectors are currently supported (not '{0}').".format(detector_type)
                    raise_error(ValueError, desc, self.curr_step)

        detector_data, subscribed_data = {}, d_class.getSubscriptionResults(detector_id)
        if not isinstance(data_keys, (list, tuple)): data_keys = [data_keys]
        for data_key in data_keys:

            error, desc = test_valid_string(data_key, valid_detector_val_keys, "data key")
            if error != None: raise_error(error, desc)

            subscription_key = traci_constants["detector"][data_key] if data_key in traci_constants["detector"] else None

            match data_key:
                case "type":
                    detector_data[data_key] = detector_type

                case "position":
                    detector_data[data_key] = self.available_detectors[detector_id][data_key]

                case "vehicle_count":
                    if "vehicle_ids" in detector_data: vehicle_count = len(detector_data["vehicle_ids"])
                    elif traci_constants["detector"]["vehicle_ids"] in subscribed_data: vehicle_count = len(list(subscribed_data[traci_constants["detector"]["vehicle_ids"]]))
                    elif subscription_key in subscribed_data: vehicle_count = subscribed_data[subscription_key]
                    else: vehicle_count = d_class.getLastStepVehicleNumber(detector_id)
                    detector_data[data_key] = vehicle_count

                case "vehicle_ids":
                    if subscription_key in subscribed_data: vehicle_ids = subscribed_data[subscription_key]
                    else: vehicle_ids = d_class.getLastStepVehicleIDs(detector_id)
                    detector_data[data_key] = list(vehicle_ids)

                case "lsm_speed":
                    if self.get_detector_vals(detector_id, "vehicle_count") > 0:
                        if subscription_key in subscribed_data: speed = subscribed_data[subscription_key]
                        else: speed = d_class.getLastStepMeanSpeed(detector_id)
                        
                        if speed > 0: speed = convert_units(speed, "m/s", self._speed_unit)
                        detector_data[data_key] = speed
                    else: detector_data[data_key] = -1

                case _:
                    valid_keys = {"multientryexit": ["halting_no"], "inductionloop": ["last_detection", "lsm_occupancy", "avg_vehicle_length"]}
                    
                    if data_key not in valid_keys[detector_type]:
                        desc = "Invalid data key '{0}' for detector type '{1}'.".format(data_key, detector_type)
                        raise_error(KeyError, desc, self.curr_step)

                    match detector_type:
                        case "multientryexit":
                            if subscription_key in subscribed_data: halting_no = subscribed_data[subscription_key]
                            else: halting_no = d_class.getLastStepHaltingNumber(detector_id)
                            detector_data[data_key] = halting_no
                            
                        case "inductionloop":
                            if data_key == "lsm_occupancy":
                                if subscription_key in subscribed_data: occupancy = subscribed_data[subscription_key]
                                else: occupancy = d_class.getLastStepOccupancy(detector_id)
                                detector_data[data_key] = occupancy / 100
                            elif data_key == "last_detection":
                                if subscription_key in subscribed_data: last_detection = subscribed_data[subscription_key]
                                else: last_detection = d_class.getTimeSinceDetection(detector_id)
                                detector_data[data_key] = last_detection
                            elif data_key == "avg_vehicle_length":
                                vehicle_len = d_class.getLastStepMeanLength(detector_id)
                                vehicle_len = convert_units(vehicle_len, "metres", self._s_dist_unit)
                                detector_data[data_key] = vehicle_len
        
        if len(detector_ids) == 1:
            if len(data_keys) == 1: return detector_data[data_keys[0]]
            else: return detector_data
        else:
            if len(data_keys) == 1: all_data_vals[detector_id] = detector_data[data_keys[0]]
            else: all_data_vals[detector_id] = detector_data
    
    return all_data_vals

def _get_interval_detector_data(self,
                                detector_ids: str | list | tuple,
                                data_keys: str | list,
                                n_steps: int,
                                interval_end: int = 0,
                                avg_step_vals: bool = True,
                                avg_det_vals: bool = True,
                                sum_counts: bool = True
                                ) -> float | list | dict:
    """
    Get data previously collected by a detector over range (`curr step - n_step - interval_end -> curr_step - interval_end`).
    Valid data keys are; '_flow_', '_density_', '_speeds_', '_no_vehicles_', '_no_unique_vehicles_', '_occupancies_' (induction loop only).
    
    Args:
        `detector_ids` (str, list, tuple):  Detector ID or list of IDs
        `data_keys` (str, list): Data key or list of keys
        `n_steps` (int): Interval length in steps (max at number of steps the simulation has run)
        `interval_end` (int): Steps since end of interval (`0 = current step`)
        `avg_step_vals` (bool): Bool denoting whether to return an average value across the interval ('_flow_' or '_density_' always returns average value for interval)
        `avg_det_vals` (bool): Bool denoting whether to return values averaged for all detectors
        `sum_counts` (bool): Bool denoting whether to return total count values ('_no_vehicles_' or '_no_unique_vehicles_', overrides `avg_step_vals`)
    
    Returns:
        (int, float, dict): Either single value or dict containing values by `data_key` and/or detectors
    """

    if self._all_data == None:
        desc = "No detector data as the simulation has not been run or data has been reset."
        raise_error(SimulationError, desc, self.curr_step)
    elif n_steps + interval_end > self._all_data["end"] - self._all_data["start"]:
        desc = "Not enough data (n_steps '{0}' + interval_end '{1}' > '{2}').".format(n_steps, interval_end, self._all_data["end"] - self._all_data["start"])
        raise_error(ValueError, desc, self.curr_step)
    elif not isinstance(n_steps, int):
        desc = "Invalid n_steps '{0}' (must be int, not '{1}').".format(n_steps, type(n_steps).__name__)
        raise_error(TypeError, desc, self.curr_step)
    elif n_steps < 1:
        desc = "Invalid n_steps '{0}' (must be >=1).".format(n_steps)
        raise_error(ValueError, desc, self.curr_step)

    all_data_vals = {}
    if isinstance(detector_ids, str): detector_ids = [detector_ids]
    detector_ids = validate_list_types(detector_ids, str, param_name="detector_ids", curr_sim_step=self.curr_step)

    if isinstance(data_keys, str): data_keys = [data_keys]
    data_keys = validate_list_types(data_keys, str, param_name="data_keys", curr_sim_step=self.curr_step)

    for data_key in data_keys:

        valid_keys = ["speeds", "occupancies", "no_vehicles", "flow", "density", "no_unique_vehicles"]
        error, desc = test_valid_string(data_key, valid_keys, "data key")
        if error != None: raise_error(error, desc)

        # Store all data values in a matrix of size (n_steps x len(detector_ids))
        all_data_vals[data_key] = []

        for detector_id in detector_ids:
            if detector_id in self._all_data["data"]["detectors"].keys():

                # Data for speeds, occupancies and no_vehicles can be
                # directly read from sim_data
                if data_key in ["speeds", "occupancies", "no_vehicles"]:

                    key = "vehicle_counts" if data_key == "no_vehicles" else data_key
                    data = self._all_data["data"]["detectors"][detector_id][key]

                    if interval_end <= 0: values = data[-n_steps:]
                    else: values = data[-(n_steps + interval_end):-interval_end]                    
                
                # Data for flow, density and no_unique_vehicles are calculated
                # from vehicle ids (as we need to count unique vehicles)
                elif data_key in ["flow", "density", "no_unique_vehicles"]:

                    veh_ids = self._all_data["data"]["detectors"][detector_id]["vehicle_ids"]

                    if interval_end <= 0: interval_ids = veh_ids[-n_steps:]
                    else: interval_ids = veh_ids[-(n_steps + interval_end):-interval_end]

                    step_counts, known_ids = [], set([])
                    for step_data in interval_ids:
                        step_ids = set(step_data)
                        step_counts.append(len(step_ids - known_ids))
                        known_ids = known_ids.union(step_ids)

                    if data_key == "no_unique_vehicles": values = step_counts
                    else:
                        
                        if sum(step_counts) > 0:
                            # average flow (vehicles per hour) = no. unique vehicles / interval duration (in hours)
                            values = sum(step_counts) / (convert_units(n_steps, "steps", "hours", self.step_length))

                            # calculate density w/ flow & speed
                            if data_key == "density":
                                speed_data = self._all_data["data"]["detectors"][detector_id]["speeds"]

                                if interval_end <= 0: speed_vals = speed_data[-n_steps:]
                                else: speed_vals = speed_data[-(n_steps + interval_end):-interval_end]

                                speed_vals = [val for val in speed_vals if val != -1]

                                if len(speed_vals) > 0:

                                    if self.units.name == "UK": speed_vals = convert_units(speed_vals, "mph", "kmph")
                                    avg_speed = sum(speed_vals) / len(speed_vals)
                                    values /= avg_speed
                                
                                else: values = 0
                        
                        # if there are no vehicles detected, flow & density = 0
                        else: values = 0

                all_data_vals[data_key].append(values)

            else:
                desc = "Detector with ID '{0}' not found.".format(detector_id)
                raise_error(KeyError, desc, self.curr_step)

        # if averaging / summing values, flatten the matrix on the
        # x axis, from (n_steps x len(detector_ids)) to (1 x len(detector_ids))
        if (avg_step_vals or sum_counts) and data_key not in ["flow", "density"]:
            for idx, det_vals in enumerate(all_data_vals[data_key]):
                vals = [val for val in det_vals if val != -1]
                
                if sum_counts and data_key in ["no_vehicles", "no_unique_vehicles"]:
                    all_data_vals[data_key][idx] = sum(vals) if len(vals) > 0 else 0

                elif avg_step_vals: all_data_vals[data_key][idx] = sum(vals) / len(vals) if len(vals) > 0 else -1

        # if averaging detector values (return average for all detectors), flatten
        # the matrix on the y axis, from ([n_steps | 1] x len(detector_ids)) to ([n_steps | 1] x 1)
        if avg_det_vals and data_key not in ["flow", "density"]:
            
            if avg_step_vals:
                vals = [val for val in all_data_vals[data_key] if val != -1]
                all_data_vals[data_key] = sum(vals) / len(vals) if len(vals) > 0 else 0

            else:
                vals = []
                for det_vals in zip(*all_data_vals[data_key]):
                    _det_vals = [val for val in det_vals if val != -1]
                    vals.append(sum(_det_vals) / len(_det_vals) if len(_det_vals) > 0 else 0)
                all_data_vals[data_key] = vals

        elif avg_det_vals and data_key in ["flow", "density"]:
            all_data_vals[data_key] = sum(all_data_vals[data_key]) / len(all_data_vals[data_key])

        else:
            # if not averaging detector values (and len(detector_ids) > 1), unpack the matrix into
            # a dictionary containing all individual detector datasets by their ID
            if len(detector_ids) == 1: all_data_vals[data_key] = all_data_vals[data_key][0]
            else: all_data_vals[data_key] = {det_id: det_vals for det_id, det_vals in zip(detector_ids, all_data_vals[data_key])}

    if len(all_data_vals) == 1: return all_data_vals[data_keys[0]]
    else: return all_data_vals


# --- VEHICLES ---

def _get_vehicle_vals(self, vehicle_ids: str | list | tuple, data_keys: str | list) -> dict | str | int | float | list:
    """
    Get data values for specific vehicle using a list of data keys. Valid data keys are;
    
    **Vehicle Characteristics**:
    '_type_', '_length_', 

    **Vehicle Status**:
    '_speed_', '_is_stopped_', '_max_speed_', '_allowed_speed_', '_speed_factor_', '_headway_', '_imperfection_',
    '_acceleration_', '_max_acceleration_', '_max_deceleration_', '_position_', '_altitude_', '_heading_',
    '_edge_id_', '_lane_id_', '_lane_idx_', '_next_edge_id_', '_leader_id_', '_leader_dist_'

    **Trip Data**:
    '_route_id_', '_route_idx_', '_route_edges_', '_departure_', '_origin_', '_destination_'
    
    Args:
        `vehicle_ids` (str, list, tuple): Vehicle ID or list of IDs
        `data_keys` (str, list): Data key or list of keys
    
    Returns:
        (dict, str, int, float, list): Values by `data_key` (or single value)
    """

    all_data_vals = {}
    if isinstance(vehicle_ids, str): vehicle_ids = [vehicle_ids]
    vehicle_ids = validate_list_types(vehicle_ids, str, param_name="vehicle_ids", curr_sim_step=self.curr_step)

    for vehicle_id in vehicle_ids:
        if not self.vehicle_exists(vehicle_id):
            desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)

        return_val = False
        if isinstance(data_keys, str):
            data_keys = [data_keys]
            return_val = True
        elif isinstance(data_keys, (list, tuple)):
            return_val = len(data_keys) == 1
        else:
            desc = "Invalid data_keys given '{0}' (must be [str | (str)], not '{1}').".format(data_keys, type(data_keys).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        data_vals, vehicle_known = {}, vehicle_id in self._known_vehicles.keys()
        if not vehicle_known: self._known_vehicles[vehicle_id] = {}

        subscribed_data = traci.vehicle.getSubscriptionResults(vehicle_id)
        for data_key in data_keys:

            error, desc = test_valid_string(data_key, valid_get_vehicle_val_keys, "data key")
            if error != None: raise_error(error, desc, self.curr_step)

            subscription_key = traci_constants["vehicle"][data_key] if data_key in traci_constants["vehicle"] else None
        
            match data_key:
                case "type":
                    new_request = not (vehicle_known and data_key in self._known_vehicles[vehicle_id].keys())
                    if new_request: self._known_vehicles[vehicle_id][data_key] = traci.vehicle.getTypeID(vehicle_id)
                    data_vals[data_key] = self._known_vehicles[vehicle_id][data_key]

                case "length":
                    new_request = not (vehicle_known and data_key in self._known_vehicles[vehicle_id].keys())
                    if new_request:
                        length = traci.vehicle.getLength(vehicle_id)
                        length = convert_units(length, "metres", self._s_dist_unit)
                        self._known_vehicles[vehicle_id][data_key] = length
                    data_vals[data_key] = self._known_vehicles[vehicle_id][data_key]

                case "speed":
                    if subscription_key in subscribed_data: speed = subscribed_data[subscription_key]
                    else: speed = traci.vehicle.getSpeed(vehicle_id)
                    
                    data_vals[data_key] = convert_units(speed, "m/s", self._speed_unit)

                case "is_stopped":
                    if "speed" in data_vals: speed = data_vals["speed"]
                    elif subscription_key in subscribed_data: speed = subscribed_data[subscription_key]
                    else: speed = traci.vehicle.getSpeed(vehicle_id)
                    data_vals[data_key] = speed < 0.1

                case "max_speed":
                    if subscription_key in subscribed_data: max_speed = subscribed_data[subscription_key]
                    else: max_speed = traci.vehicle.getMaxSpeed(vehicle_id)
                    
                    data_vals[data_key] = convert_units(max_speed, "m/s", self._speed_unit)

                case "allowed_speed":
                    if subscription_key in subscribed_data: allowed_speed = subscribed_data[subscription_key]
                    else: allowed_speed = traci.vehicle.getAllowedSpeed(vehicle_id)
                    
                    data_vals[data_key] = convert_units(allowed_speed, "m/s", self._speed_unit)

                case "speed_factor":
                    if subscription_key in subscribed_data: speed_factor = subscribed_data[subscription_key]
                    else: speed_factor = traci.vehicle.getSpeedFactor(vehicle_id)

                    data_vals[data_key] = speed_factor

                case "headway":
                    if subscription_key in subscribed_data: headway = subscribed_data[subscription_key]
                    else: headway = traci.vehicle.getTau(vehicle_id)

                    data_vals[data_key] = headway

                case "imperfection":
                    if subscription_key in subscribed_data: imperfection = subscribed_data[subscription_key]
                    else: imperfection = traci.vehicle.getImperfection(vehicle_id)

                    data_vals[data_key] = imperfection

                case "acceleration":
                    if subscription_key in subscribed_data: acceleration = subscribed_data[subscription_key]
                    else: acceleration = traci.vehicle.getAcceleration(vehicle_id)
                    data_vals[data_key] = acceleration

                case "max_acceleration":
                    if subscription_key in subscribed_data: max_acceleration = subscribed_data[subscription_key]
                    else: max_acceleration = traci.vehicle.getAccel(vehicle_id)
                    data_vals[data_key] = max_acceleration

                case "max_deceleration":
                    if subscription_key in subscribed_data: max_deceleration = subscribed_data[subscription_key]
                    else: max_deceleration = traci.vehicle.getDecel(vehicle_id)
                    data_vals[data_key] = max_deceleration

                case "position":
                    if subscription_key in subscribed_data: position = list(subscribed_data[subscription_key])
                    elif traci_constants["vehicle"]["altitude"] in subscribed_data: position = list(subscribed_data[traci_constants["vehicle"]["altitude"]])[:2]
                    else: position = list(traci.vehicle.getPosition3D(vehicle_id))[:2]
                    data_vals[data_key] = tuple(position)

                case "altitude":
                    if subscription_key in subscribed_data: altitude = list(subscribed_data[subscription_key])[-1]
                    else: altitude = list(traci.vehicle.getPosition3D(vehicle_id))[-1]
                    data_vals[data_key] = altitude

                case "heading":
                    if subscription_key in subscribed_data: heading = subscribed_data[subscription_key]
                    else: heading = traci.vehicle.getAngle(vehicle_id)
                    data_vals[data_key] = heading

                case "departure":
                    new_request = not (vehicle_known and data_key in self._known_vehicles[vehicle_id].keys())
                    if new_request: self._known_vehicles[vehicle_id][data_key] = int(traci.vehicle.getDeparture(vehicle_id) / self.step_length)
                    data_vals[data_key] = self._known_vehicles[vehicle_id][data_key]

                case "edge_id":
                    if subscription_key in subscribed_data: edge_id = subscribed_data[subscription_key]
                    else: edge_id = traci.vehicle.getRoadID(vehicle_id)
                    data_vals[data_key] = edge_id

                case "lane_id":
                    if subscription_key in subscribed_data: lane_id = subscribed_data[subscription_key]
                    else: lane_id = traci.vehicle.getLaneID(vehicle_id)
                    data_vals[data_key] = lane_id

                case "lane_idx":
                    if subscription_key in subscribed_data: lane_idx = subscribed_data[subscription_key]
                    else: lane_idx = traci.vehicle.getLaneIndex(vehicle_id)
                    data_vals[data_key] = lane_idx

                case "origin":
                    new_request = not (vehicle_known and data_key in self._known_vehicles[vehicle_id].keys())
                    if new_request: self._known_vehicles[vehicle_id][data_key] = list(traci.vehicle.getRoute(vehicle_id))[0]
                    data_vals[data_key] = self._known_vehicles[vehicle_id][data_key]

                case "destination":
                    if subscription_key in subscribed_data: route = subscribed_data[subscription_key]
                    else: route = list(traci.vehicle.getRoute(vehicle_id))
                    data_vals[data_key] = route[-1]

                case "route_id":
                    if subscription_key in subscribed_data: route_id = subscribed_data[subscription_key]
                    else: route_id = traci.vehicle.getRouteID(vehicle_id)
                    data_vals[data_key] = route_id

                case "route_idx":
                    if subscription_key in subscribed_data: route_idx = subscribed_data[subscription_key]
                    else: route_idx = list(traci.vehicle.getRouteIndex(vehicle_id))
                    data_vals[data_key] = route_idx

                case "route_edges":
                    route = list(traci.vehicle.getRoute(vehicle_id))
                    data_vals[data_key] = route

                case "next_edge_id":
                    if "route_edges" in data_vals: route_edges = data_vals["route_edges"]
                    else: route_edges = list(traci.vehicle.getRoute(vehicle_id))

                    if "edge_id" in data_vals: edge_id = data_vals["edge_id"]
                    else: edge_id = traci.vehicle.getRoadID(vehicle_id)

                    edge_idx = route_edges.index(edge_id)
                    if edge_idx == len(route_edges) - 1: data_vals[data_key] = None
                    else:
                        next_edges = route_edges[route_edges.index(edge_id) + 1:]
                        filtered = [item for item in next_edges if not item.startswith(':')]
                        data_vals[data_key] = filtered[0] if filtered else None

                case "leader_id":
                    if subscription_key in subscribed_data: leader_data = subscribed_data[subscription_key]
                    else: leader_data = traci.vehicle.getLeader(vehicle_id)

                    if leader_data == None: data_vals[data_key] = None
                    else: data_vals[data_key] = leader_data[0]

                case "leader_dist":
                    if subscription_key in subscribed_data: leader_data = subscribed_data[subscription_key]
                    else: leader_data = traci.vehicle.getLeader(vehicle_id)

                    if leader_data == None: leader_dist = None
                    else: leader_dist = convert_units(leader_data[1], "metres", self._s_dist_unit)

                    data_vals[data_key] = leader_dist

        if len(vehicle_ids) == 1:
            if return_val: return list(data_vals.values())[0]
            else: return data_vals
        else:
            if return_val: all_data_vals[vehicle_id] = list(data_vals.values())[0]
            else: all_data_vals[vehicle_id] = data_vals

    return all_data_vals

def _set_vehicle_vals(self, vehicle_ids: list | tuple | str, **kwargs) -> None:
    """
    Changes vehicle characteristics.
    
    Args:
        `vehicle_ids` (list, tuple, str): Vehicle ID or list of IDs
        `type` (str, optional): Vehicle type ID
        `colour` (str, list, tuple, optional): Vehicle colour, either hex code, list of rgb/rgba values or valid SUMO colour string
        `highlight` (bool, optional): Highlights the vehicle with a circle (bool)
        `speed` (int, float, optional): Set new speed value
        `max_speed` (int, float, optional): Set new max speed value
        `speed_factor` (int, float, optional): Set new speed factor value
        `headway` (int, float): Desired minimum time headway in seconds
        `imperfection` (int, float): Driver imperfection (0 denotes perfect driving)
        `acceleration` ((int, float, optional), (int, float)): Set acceleration for a given duration 
        `max_acceleration` (int, float, optional): Set maximum allowed acceleration
        `max_deceleration` (int, float, optional): Set maximum allowed deceleration
        `lane_idx` (int, (int, float), optional): Try and change lane for a given duration
        `destination` (str, optional): Set vehicle destination edge ID
        `route_id` (str, optional): Set vehicle route by route ID or list of edges
        `route_edges` (list, optional): Set vehicle route by list of edges
        `speed_safety_checks` (bool, optional): (**Indefinitely**) set whether speed/acceleration safety constraints are followed when setting speed
        `lc_safety_checks` (bool, optional): (**Indefinitely**) set whether lane changing safety constraints are followed when changing lane
        `stop` (bool, optional): Stop the vehicle on the following edge
    """

    if isinstance(vehicle_ids, str): vehicle_ids = [vehicle_ids]
    vehicle_ids = validate_list_types(vehicle_ids, str, param_name="vehicle_ids", curr_sim_step=self.curr_step)

    for vehicle_id in vehicle_ids:
        if not self.vehicle_exists(vehicle_id):
            desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)
        
        for command, value in kwargs.items():

            error, desc = test_valid_string(command, valid_set_vehicle_val_keys, "command")
            if error != None: raise_error(error, desc, self.curr_step)

            match command:
                case "type":
                    if not self.vehicle_type_exists(value):
                        desc = f"Vehicle type ID '{value}' not found."
                        raise_error(KeyError, desc, self.curr_step)
                    traci.vehicle.setType(vehicle_id, value)
                    if vehicle_id not in self._known_vehicles: self._known_vehicles[vehicle_id] = {}
                    self._known_vehicles[vehicle_id]["type"] = value
                    if "length" in self._known_vehicles[vehicle_id]:
                        self._known_vehicles[vehicle_id]["length"] = self.get_vehicle_vals(vehicle_id, "length")
                    
                case "colour":
                    if value != None:
                        traci.vehicle.setColor(vehicle_id, colour_to_rgba(value, self.curr_step, f"({command}): "))
                    else:
                        vehicle_type = self.get_vehicle_vals(vehicle_id, "type")
                        type_colour = tuple(traci.vehicletype.getColor(vehicle_type))
                        traci.vehicle.setColor(vehicle_id, type_colour)
                
                case "highlight":
                    if isinstance(value, bool):
                        if value: traci.vehicle.highlight(vehicle_id)
                        else: traci.vehicle.highlight(vehicle_id, color=(0, 0, 0, 0))
                    else:
                        desc = "({0}): Invalid speed_safety_checks value '{1}' (must be (str), not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "speed":
                    if isinstance(value, (int, float)): 
                        traci.vehicle.setSpeed(vehicle_id, convert_units(value, self._speed_unit, "m/s"))
                    else:
                        desc = "({0}): Invalid speed value '{1}' (must be [int | float], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                
                case "max_speed":
                    if isinstance(value, (int, float)):
                        traci.vehicle.setMaxSpeed(vehicle_id, convert_units(value, self._speed_unit, "m/s"))
                    else:
                        desc = "({0}): Invalid max_speed value '{1}' (must be [int | float], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "speed_factor":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid speed factor value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicle.setSpeedFactor(vehicle_id, value)
                    else:
                        desc = f"({command}): Invalid speed factor value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "headway":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid headway value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicle.setTau(vehicle_id, value)
                    else:
                        desc = f"({command}): Invalid headway value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "imperfection":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid imperfection value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicle.setImperfection(vehicle_id, value)
                    else:
                        desc = f"({command}): Invalid imperfection value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "acceleration":
                    if isinstance(value, (list, tuple)) and len(value) == 2:
                        if not isinstance(value[0], (int, float)):
                            desc = "({0}): Invalid acceleration '{1}' (must be [int | float], not '{2}').".format(command, value[0], type(value[0]).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                        if not isinstance(value[1], (int, float)):
                            desc = "({0}): Invalid duration '{1}' (must be [int | float], not '{2}').".format(command, value[1], type(value[1]).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                        traci.vehicle.setAcceleration(vehicle_id, float(value[0]), float(value[1]))
                    else:
                        desc = "({0}): '{0}' requires 2 parameters (acceleration [int | float], duration [int | float])".format(command)
                        raise_error(TypeError, desc, self.curr_step)

                case "max_acceleration":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid max_acceleration value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicle.setAccel(vehicle_id, value)
                    else:
                        desc = f"({command}): Invalid max_acceleration value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "max_deceleration":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid max_deceleration value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicle.setDecel(vehicle_id, value)
                    else:
                        desc = f"({command}): Invalid max_deceleration value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "lane_idx":
                    if isinstance(value, (list, tuple)) and len(value) == 2:
                        if not isinstance(value[0], int):
                            desc = "({0}): Invalid lane_idx '{1}' (must be int, not '{2}').".format(command, value[0], type(value[0]).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                        if not isinstance(value[1], (int, float)):
                            desc = "({0}): Invalid duration '{1}' (must be [int | float], not '{2}').".format(command, value[1], type(value[1]).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                        traci.vehicle.changeLane(vehicle_id, value[0], float(value[1]))
                    else:
                        desc = "({0}): '{0}' requires 2 parameters (lane_idx [int], duration [int | float])".format(command)
                        raise_error(TypeError, desc, self.curr_step)

                case "destination":
                    if isinstance(value, str):
                        if value not in self._all_edges:
                            desc = "({0}): Edge ID '{1}' not found.".format(command, value)
                            raise_error(KeyError, desc, self.curr_step)
                        traci.vehicle.changeTarget(vehicle_id, value)
                    else:
                        desc = "({0}): Invalid edge_id '{1}' (must be str, not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                
                case "route_id":
                    if isinstance(value, str):
                        if value not in self._all_routes.keys():
                            desc = "({0}): Route ID '{1}' not found.".format(command, value)
                            raise_error(KeyError, desc, self.curr_step)
                        traci.vehicle.setRouteID(vehicle_id, value)
                    else:
                        desc = "({0}): Invalid route_id value '{1}' (must be str, not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                
                case "route_edges":
                    if isinstance(value, (list, tuple)) and all(isinstance(x, str) for x in value):
                        for e_id in value:
                            if e_id not in self._all_edges:
                                desc = "({0}): Edge ID '{1}' in route edges not found.".format(command, e_id)
                                raise_error(KeyError, desc, self.curr_step)
                        traci.vehicle.setRoute(vehicle_id, value)
                    else:
                        desc = "({0}): Invalid route_egdes value '{1}' (must be (str), not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "speed_safety_checks":
                    if isinstance(value, bool):
                        if value: traci.vehicle.setSpeedMode(vehicle_id, 31)
                        else: traci.vehicle.setSpeedMode(vehicle_id, 32)
                    elif isinstance(value, int):
                        traci.vehicle.setSpeedMode(vehicle_id, value)
                    else:
                        desc = "({0}): Invalid speed_safety_checks value '{1}' (must be (str), not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "lc_safety_checks":
                    if isinstance(value, bool):
                        if value: traci.vehicle.setLaneChangeMode(vehicle_id, 1621)
                        else: traci.vehicle.setLaneChangeMode(vehicle_id, 1617)
                    elif isinstance(value, int):
                        traci.vehicle.setLaneChangeMode(vehicle_id, value)
                    else:
                        desc = "({0}): Invalid speed_safety_checks value '{1}' (must be (str), not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "stop":
                    if isinstance(value, bool):
                        if value and vehicle_id not in self._stopped_vehicles:
                            self.stop_vehicle(vehicle_id)
                        elif not value:
                            self.resume_vehicle(vehicle_id)
                    else:
                        desc = f"({command}: Invalid stop value '{value}' (must be bool, not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

def _get_vehicle_data(self, vehicle_ids: str | list | tuple, refresh: bool = False) -> dict | None:
    """
    Get data for specified vehicle(s).
    
    Args:
        `vehicle_ids` (str, list, tuple): Vehicle ID or list of IDs
        `refresh` (bool): Denotes whether to update static vehicle data
    
    Returns:
        (dict, optional): Vehicle data dictionary, returns None if does not exist in simulation
    """

    all_vehicle_data = {}
    if isinstance(vehicle_ids, str): vehicle_ids = [vehicle_ids]
    vehicle_ids = validate_list_types(vehicle_ids, str, param_name="vehicle_ids", curr_sim_step=self.curr_step)

    for vehicle_id in vehicle_ids:
        if not self.vehicle_exists(vehicle_id):
            desc = "Unrecognised vehicle ID found ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)
        
        static_data_keys = ("type", "length", "departure", "origin")
        dynamic_data_keys = ("edge_id", "lane_id", "speed", "allowed_speed", "acceleration", "is_stopped", "position", "altitude", "heading", "destination")
        vehicle_data = self.get_vehicle_vals(vehicle_id, dynamic_data_keys)

        if vehicle_id not in self._known_vehicles.keys(): new_vehicle = True
        elif len(set(static_data_keys) - set(self._known_vehicles[vehicle_id].keys())) > 0: new_vehicle = True
        else: new_vehicle = False

        if new_vehicle or refresh:
            static_veh_data = self.get_vehicle_vals(vehicle_id, static_data_keys)

            # Maintain _known_vehicles dictionary to not repeatedly need to fetch static data
            self._known_vehicles[vehicle_id] = {"type":          static_veh_data["type"],
                                                "edge_id":       vehicle_data["edge_id"],
                                                "lane_id":       vehicle_data["lane_id"],
                                                "longitude":     vehicle_data["position"][0],
                                                "latitude":      vehicle_data["position"][1],
                                                "speed":         vehicle_data["speed"],
                                                "allowed_speed": vehicle_data["allowed_speed"],
                                                "acceleration":  vehicle_data["acceleration"],
                                                "is_stopped":    vehicle_data["is_stopped"],
                                                "length":        static_veh_data["length"],
                                                "heading":       vehicle_data["heading"],
                                                "departure":     static_veh_data["departure"],
                                                "altitude":      vehicle_data["altitude"],
                                                "destination":   vehicle_data["destination"],
                                                "origin":        static_veh_data["origin"],
                                                "last_seen":     self.curr_step
                                                }
        else:

            # Update _known_vehicles with dynamic data
            for key in dynamic_data_keys:
                if key != "position":
                    self._known_vehicles[vehicle_id][key] = vehicle_data[key]
                else:
                    coors = vehicle_data[key]
                    self._known_vehicles[vehicle_id]["longitude"] = coors[0]
                    self._known_vehicles[vehicle_id]["latitude"] = coors[1]

            self._known_vehicles[vehicle_id]["last_seen"]    = self.curr_step

        vehicle_data = copy(self._known_vehicles[vehicle_id])
        del vehicle_data['last_seen']

        if len(vehicle_ids) == 1:
            return vehicle_data
        else:
            all_vehicle_data[vehicle_id] = vehicle_data

    return all_vehicle_data

def _get_all_vehicle_data(self, vehicle_types: list | tuple | None = None) -> dict:
    """
    Collects aggregated vehicle data (no. vehicles & no. waiting vehicles) and all individual vehicle data.
    Also calculates edge/lane flow, delay and density for the last time step.
    
    Args:
        `vehicle_types` (list, tuple, optional): Type(s) of vehicles to include
    
    Returns:
        dict: no vehicles, no waiting, all vehicle data
    """

    if len(self._last_step_delay) == 0 or len(self._last_step_flow) == 0:
        for lane_id in self._all_lanes:
            if not lane_id.startswith(":"):
                self._last_step_delay[lane_id] = 0
                self._last_step_flow[lane_id] = 0
                self._last_step_density[lane_id] = 0

                edge_id = self.get_geometry_vals(lane_id, "edge_id")
                self._last_step_delay[edge_id] = 0
                self._last_step_flow[edge_id] = 0
                self._last_step_density[edge_id] = 0
                self._lane_to_edges[lane_id] = edge_id
    
    else:
        self._last_step_delay = {g_id: 0 for g_id in self._last_step_delay}
        self._last_step_flow = {g_id: 0 for g_id in self._last_step_flow}
        self._last_step_density = {g_id: 0 for g_id in self._last_step_density}

    all_vehicle_data = {}
    total_vehicle_data = {"no_vehicles": 0, "no_waiting": 0, "delay": 0}
    lane_speeds, allowed_speeds = {}, {}
    #vtype_sf, lane_speed_lims = {}, {}

    for vehicle_id in self._all_curr_vehicle_ids:

        vehicle_type = self.get_vehicle_vals(vehicle_id, "type")

        if vehicle_types is None or (isinstance(vehicle_types, (list, tuple)) and vehicle_type in vehicle_types) or (isinstance(vehicle_types, str) and vehicle_type == vehicle_types):

            if self._get_fc_data:
                vehicle_data = self.get_vehicle_data(vehicle_id)
                all_vehicle_data[vehicle_id] = vehicle_data
            else: vehicle_data = self.get_vehicle_vals(vehicle_id, ["speed", "lane_id", "allowed_speed", "is_stopped"])

            total_vehicle_data["no_vehicles"] += 1
            if vehicle_data["is_stopped"]: total_vehicle_data["no_waiting"] += 1

            lane_id, speed, allowed_speed = vehicle_data["lane_id"], vehicle_data["speed"], vehicle_data["allowed_speed"]
            
            #if "orig" in sys.argv: allowed_speed = vehicle_data["allowed_speed"]
            #else:
            #    if v_type in vtype_sf: speed_factor = vtype_sf[v_type]
            #    else: speed_factor = self.get_vehicle_type_vals(v_type, "speed_factor")

            #    if lane_id in lane_speed_lims: speed_limit = lane_speed_lims[lane_id]
            #    else: speed_limit = self.get_geometry_vals(lane_id, "max_speed")

            #    allowed_speed = min(max_speed, speed_factor * speed_limit)

            # Vehicle is on an internal edge (in an intersection) - set to its last non-internal lane
            while lane_id.startswith(':'): lane_id = self._network.getLane(lane_id).getIncoming()[0].getID()

            # Convert allowed/measured speeds to m/s
            if lane_id not in lane_speeds: lane_speeds[lane_id], allowed_speeds[lane_id] = [], []
            (speed, allowed_speed) = convert_units([speed, allowed_speed], self._speed_unit, "m/s")
            lane_speeds[lane_id].append(speed)
            allowed_speeds[lane_id].append(allowed_speed)

    for lane_id, lane_data in lane_speeds.items():

        lane_delay = 0 # veh*s         # len(lane_data) = No. vehicles on the lane
        average_speed = sum(lane_data) / len(lane_data) # m/s
        free_flow_speed = sum(allowed_speeds[lane_id]) / len(allowed_speeds[lane_id]) # m/s

        if average_speed == 0:
            # delay = TTS on lane if speed == 0, and flow = 0
            lane_delay, lane_flow = len(lane_data) * self.step_length, 0

        else:
            lane_length = convert_units(self._lane_info[lane_id]["length"], self._l_dist_unit, "metres")

            # step flow (veh) = speed (m/s) x density (veh/m) x step length (s)
            lane_flow = (average_speed * (len(lane_data) / lane_length)) * self.step_length
            lane_delay = lane_flow * ((lane_length / average_speed) - (lane_length / free_flow_speed))

        # Bound delay to ≥ 0, to avoid cases where average_speed > free_flow_speed
        lane_delay = max(0, lane_delay)

        total_vehicle_data["delay"] += lane_delay

        if lane_id in self._last_step_delay:
            edge_id = self._lane_to_edges[lane_id]

            self._last_step_delay[lane_id] = lane_delay
            self._last_step_delay[edge_id] += lane_delay

            # Convert flow to veh/hr
            lane_flow = (lane_flow / self.step_length) * 3600
            self._last_step_flow[lane_id] = lane_flow
            self._last_step_flow[edge_id] += lane_flow

            # Calculate lane density as n_vehicles / lane length (in long dist units (km or mi))
            self._last_step_density[lane_id] = len(lane_data) / self._lane_info[lane_id]["length"]
            self._last_step_density[edge_id] += len(lane_data) # temporarily use ls density to store n vehicles on the edge

    for edge_id in self._all_edges:
        if edge_id in self._last_step_density:
            # Calculate edge density as n_vehicles / edge length
            self._last_step_density[edge_id] /= self._edge_info[edge_id]["length"]

    total_vehicle_data["to_depart"] = len(self._all_to_depart_vehicle_ids)

    # Delay of vehicles waiting to be inserted into the simulation (veh*s)
    insertion_delay = total_vehicle_data["to_depart"] * self.step_length
    total_vehicle_data["insertion_delay"] = insertion_delay
    if self._include_insertion_delay:
        total_vehicle_data["delay"] += insertion_delay

    return total_vehicle_data, all_vehicle_data


# --- VEHICLE TYPES ---

def _set_vehicle_type_vals(self, vehicle_types: list | tuple | str, **kwargs) -> None:
    """
    Changes vehicle type characteristics.
    
    Args:
        `vehicle_class` (str): Vehicle class ID
        `colour` (str, list, tuple): Vehicle colour, either hex code, list of rgb/rgba values or valid SUMO colour string
        `length` (int, float): Vehicle length in metres/feet
        `width` (int, float): Vehicle width in metres/feet
        `height` (int, float): Vehicle height in metres/feet
        `max_speed` (int, float): Vehicle max speed in km/h or mph
        `speed_factor` (int, float): Vehicle speed multiplier
        `speed_dev` (int, float): Vehicle deviation from speed factor
        `min_gap` (int, float): Minimum gap behind leader
        `max_acceleration` (int, float): Maximum vehicle acceleration
        `max_deceleration` (int, float): Maximum vehicle deceleration
        `headway` (int, float): Desired minimum time headway in seconds
        `imperfection` (int, float): Driver imperfection (0 denotes perfect driving)
        `max_lateral_speed` (int, float): Maximum lateral speed when lane changing
        `emission_class` (str): Vehicle emissions class ID
        `gui_shape` (str): Vehicle shape in GUI
    """

    if isinstance(vehicle_types, str): vehicle_types = [vehicle_types]
    vehicle_types = validate_list_types(vehicle_types, str, param_name="vehicle_types", curr_sim_step=self.curr_step)

    for vehicle_type in vehicle_types:
        if not self.vehicle_type_exists(vehicle_type):
            desc = f"Unrecognised vehicle type ID given ('{vehicle_type}')."
            raise_error(KeyError, desc, self.curr_step)
        
        for command, value in kwargs.items():

            error, desc = test_valid_string(command, valid_vehicle_type_val_keys, "command")
            if error != None: raise_error(error, desc, self.curr_step)

            if value == None: continue

            match command:
                case "vehicle_class":
                    if isinstance(value, str):
                        traci.vehicletype.setVehicleClass(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid vehicle_class value '{value}' (must be str, not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)
                    
                case "colour":
                    traci.vehicletype.setColor(vehicle_type, colour_to_rgba(value, self.curr_step, f"({command}): "))

                case "length":
                    if isinstance(value, (int, float)):
                        if value <= 0.1:
                            desc = f"({command}): Invalid length value '{value}' (must be > 0.1)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setLength(vehicle_type, convert_units(value, self._s_dist_unit, "metres"))
                    else:
                        desc = f"({command}): Invalid length value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "width":
                    if isinstance(value, (int, float)):
                        if value <= 0.1:
                            desc = f"({command}): Invalid width value '{value}' (must be > 0.1)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setWidth(vehicle_type, convert_units(value, self._s_dist_unit, "metres"))
                    else:
                        desc = f"({command}): Invalid width value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "height":
                    if isinstance(value, (int, float)):
                        if value <= 0.1:
                            desc = f"({command}): Invalid height value '{value}' (must be > 0.1)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setHeight(vehicle_type, convert_units(value, self._s_dist_unit, "metres"))
                    else:
                        desc = f"({command}): Invalid height value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "mass":
                    if isinstance(value, (int, float)):
                        if value <= 0.1:
                            desc = f"({command}): Invalid mass value '{value}' (must be > 0.1)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setMass(vehicle_type, convert_units(value, self._weight_unit, "kilograms"))
                    else:
                        desc = f"({command}): Invalid mass value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "max_speed":
                    if isinstance(value, (int, float)):
                        if value <= 0.1:
                            desc = f"({command}): Invalid max_speed value '{value}' (must be > 0.1)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setMaxSpeed(vehicle_type, convert_units(value, self._speed_unit, "m/s"))
                    else:
                        desc = f"({command}): Invalid max_speed value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "speed_factor":
                    if isinstance(value, (int, float)):
                        if value <= 0:
                            desc = f"({command}): Invalid speed_factor value '{value}' (must be > 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setSpeedFactor(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid speed_factor value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "speed_dev":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid speed_dev value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setSpeedDeviation(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid speed_dev value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "max_acceleration":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid max_acceleration value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setAccel(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid max_acceleration value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "max_deceleration":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid max_deceleration value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setDecel(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid max_deceleration value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "headway":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid headway value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setTau(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid headway value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "imperfection":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid imperfection value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setImperfection(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid imperfection value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "max_lateral_speed":
                    if isinstance(value, (int, float)):
                        if value < 0:
                            desc = f"({command}): Invalid max_lateral_speed value '{value}' (must be >= 0)."
                            raise_error(ValueError, desc, self.curr_step)
                        traci.vehicletype.setMaxSpeedLat(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid max_lateral_speed value '{value}' (must be [int | float], not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

                case "emission_class":
                    if isinstance(value, str):
                        traci.vehicletype.setEmissionClass(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid emission_class value '{value}' (must be 'str', not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)
                
                case "gui_shape":
                    if isinstance(value, str):
                        traci.vehicletype.setShapeClass(vehicle_type, value)
                    else:
                        desc = f"({command}): Invalid gui_shape value '{value}' (must be 'str', not '{type(value).__name__}')."
                        raise_error(TypeError, desc, self.curr_step)

def _get_vehicle_type_vals(self, vehicle_types: str | list | tuple, data_keys: str | list) -> dict | str | float | tuple:
    """
    Get data values for specific vehicle type(s) using a list of data keys. Valid data keys are;
    '_vehicle_class_', '_colour_', '_length_', '_width_', '_height_', '_headway_', '_imperfection_',
    '_max_speed_', '_speed_factor_', '_speed_dev_', '_min_gap_', '_max_acceleration_', '_max_deceleration_',
    '_max_lateral_speed_', '_emission_class_', '_gui_shape_'
    
    Args:
        `vehicle_types` (str, list, tuple): Vehicle type ID or list of IDs
        `data_keys` (str, list): Data key or list of keys
    
    Returns:
        (dict, str, float, tuple): Values by `data_key` (or single value)
    """
            
    all_data_vals = {}
    if isinstance(vehicle_types, str): vehicle_types = [vehicle_types]
    vehicle_types = validate_list_types(vehicle_types, str, param_name="vehicle_types", curr_sim_step=self.curr_step)

    for vehicle_type in vehicle_types:
        if not self.vehicle_type_exists(vehicle_type):
            desc = f"Unrecognised vehicle type ID given ('{vehicle_type}')."
            raise_error(KeyError, desc, self.curr_step)

        return_val = False
        if isinstance(data_keys, str):
            data_keys = [data_keys]
            return_val = True
        elif isinstance(data_keys, (list, tuple)):
            return_val = len(data_keys) == 1
        else:
            desc = "Invalid data_keys given '{0}' (must be [str | (str)], not '{1}').".format(data_keys, type(data_keys).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        data_vals = {}

        for data_key in data_keys:

            error, desc = test_valid_string(data_key, valid_vehicle_type_val_keys, "data key")
            if error != None: raise_error(error, desc, self.curr_step)

            match data_key:

                case "vehicle_class":
                    data_vals[data_key] = traci.vehicletype.getVehicleClass(vehicle_type)

                case "colour":
                    data_vals[data_key] = tuple(traci.vehicletype.getColor(vehicle_type))

                case "length":
                    length = convert_units(traci.vehicletype.getLength(vehicle_type), "metres", self._s_dist_unit)
                    data_vals[data_key] = length

                case "width":
                    width = convert_units(traci.vehicletype.getWidth(vehicle_type), "metres", self._s_dist_unit)
                    data_vals[data_key] = width

                case "height":
                    height = convert_units(traci.vehicletype.getHeight(vehicle_type), "metres", self._s_dist_unit)
                    data_vals[data_key] = height

                case "mass":
                    mass = convert_units(traci.vehicletype.getMass(vehicle_type), "kilograms", self._weight_unit)
                    data_vals[data_key] = mass

                case "max_speed":
                    max_speed = convert_units(traci.vehicletype.getMaxSpeed(vehicle_type), "m/s", self._speed_unit)
                    data_vals[data_key] = max_speed

                case "speed_factor":
                    data_vals[data_key] = traci.vehicletype.getSpeedFactor(vehicle_type)

                case "speed_dev":
                    data_vals[data_key] = traci.vehicletype.getSpeedDeviation(vehicle_type)

                case "max_acceleration":
                    data_vals[data_key] = traci.vehicletype.getAccel(vehicle_type)

                case "max_deceleration":
                    data_vals[data_key] = traci.vehicletype.getDecel(vehicle_type)

                case "headway":
                    data_vals[data_key] = traci.vehicletype.getTau(vehicle_type)

                case "imperfection":
                    data_vals[data_key] = traci.vehicletype.getImperfection(vehicle_type)

                case "max_lateral_speed":
                    data_vals[data_key] = traci.vehicletype.getMaxSpeedLat(vehicle_type)

                case "emission_class":
                    data_vals[data_key] = traci.vehicletype.getEmissionClass(vehicle_type)
                
                case "gui_shape":
                    data_vals[data_key] = traci.vehicletype.getShapeClass(vehicle_type)

        if len(vehicle_types) == 1:
            if return_val: return list(data_vals.values())[0]
            else: return data_vals
        else:
            if return_val: all_data_vals[vehicle_type] = list(data_vals.values())[0]
            else: all_data_vals[vehicle_type] = data_vals

    return all_data_vals


# --- GEOMETRY ---

def _get_geometry_vals(self, geometry_ids: str | list | tuple, data_keys: str | list) -> dict | str | int | float | list:
    """
    Get data values for specific edge or lane using a list of data keys. Valid data keys are:
    
    **Edge or Lane**:
    '_vehicle_count_', '_vehicle_ids_', '_avg_vehicle_length_', '_halting_no_', '_vehicle_speed_', '_vehicle_occupancy_',
    '_vehicle_flow_', '_vehicle_density_', '_vehicle_tts_', '_vehicle_delay_', '_curr_travel_time_', '_ff_travel_time_',
    '_emissions_', '_length_', '_max_speed_'

    **Edge only**:
    '_connected_edges_', '_incoming_edges_', '_outgoing_edges_', '_junction_ids_', '_linestring_', '_street_name_',
    '_n_lanes_', '_lane_ids_'

    **Lane only**:
    '_edge_id_', '_n_links_', '_allowed_', '_disallowed_', '_left_lc_', '_right_lc_'

    Args:
        `geometry_ids` (str, int): Edge/lane ID or list of IDs
        `data_keys` (str, list): Data key or list of keys
    
    Returns:
        dict: Values by `data_key` (or single value)
    """

    all_data_vals = {}
    if isinstance(geometry_ids, str): geometry_ids = [geometry_ids]
    geometry_ids = validate_list_types(geometry_ids, str, param_name="geometry_ids", curr_sim_step=self.curr_step)

    for geometry_id in geometry_ids:
        g_name = self.geometry_exists(geometry_id)
        if g_name == "edge": g_class = traci.edge
        elif g_name == "lane" or geometry_id.startswith(":"): g_class = traci.lane
        else:
            desc = "Geometry ID '{0}' not found.".format(geometry_id)
            raise_error(KeyError, desc, self.curr_step)

        return_val = False
        if isinstance(data_keys, str):
            data_keys = [data_keys]
            return_val = True
        elif isinstance(data_keys, (list, tuple)):
            return_val = len(data_keys) == 1
        else:
            desc = "Invalid data_keys given '{0}' (must be [str | (str)], not '{1}').".format(data_keys, type(data_keys).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        data_vals, subscribed_data = {}, g_class.getSubscriptionResults(geometry_id)
        for data_key in data_keys:

            error, desc = test_valid_string(data_key, valid_get_edge_val_keys + valid_get_lane_val_keys, "data key")
            if error != None: raise_error(error, desc, self.curr_step)
            elif g_name == "edge" and data_key not in valid_get_edge_val_keys:
                desc = f"Invalid data key '{data_key}' for edge '{geometry_id}' (only valid for lanes, not edges)."
                raise_error(ValueError, desc, self.curr_step)
            elif g_name == "lane" and data_key not in valid_get_lane_val_keys:
                desc = f"Invalid data key '{data_key}' for lane '{geometry_id}' (only valid for edges, not lanes)."
                raise_error(ValueError, desc, self.curr_step)

            subscription_key = traci_constants["geometry"][data_key] if data_key in traci_constants["geometry"] else None

            match data_key:
                case "vehicle_count":
                    if "vehicle_ids" in data_vals: vehicle_count = len(data_vals["vehicle_ids"])
                    elif traci_constants["geometry"]["vehicle_ids"] in subscribed_data: vehicle_count = len(list(subscribed_data[traci_constants["geometry"]["vehicle_ids"]]))
                    elif subscription_key in subscribed_data: vehicle_count = subscribed_data[subscription_key]
                    else: vehicle_count = g_class.getLastStepVehicleNumber(geometry_id)
                    data_vals[data_key] = vehicle_count

                case "vehicle_ids":
                    if subscription_key in subscribed_data: vehicle_ids = list(subscribed_data[subscription_key])
                    else: vehicle_ids = g_class.getLastStepVehicleIDs(geometry_id)
                    data_vals[data_key] = list(vehicle_ids)

                case "vehicle_speed":
                    if subscription_key in subscribed_data: vehicle_ids = subscribed_data[subscription_key]
                    else: vehicle_ids = list(g_class.getLastStepVehicleIDs(geometry_id))

                    if len(vehicle_ids) == 0: vehicle_speed = None
                    elif len(vehicle_ids) == 1: vehicle_speed = self.get_vehicle_vals(vehicle_ids, "speed")
                    else: vehicle_speed = sum(self.get_vehicle_vals(vehicle_ids, "speed").values()) / len(vehicle_ids)

                    data_vals[data_key] = vehicle_speed
                    
                case "vehicle_density":
                    if len(self._last_step_density) == 0: density = 0
                    else: density = self._last_step_density[geometry_id]
                    data_vals[data_key] = density

                case "vehicle_flow":
                    if len(self._last_step_flow) == 0: flow = 0
                    else: flow = self._last_step_flow[geometry_id]
                    data_vals[data_key] = flow

                case "vehicle_delay":
                    if len(self._last_step_delay) == 0: delay = 0
                    else: delay = self._last_step_delay[geometry_id]
                    data_vals[data_key] = delay

                case "vehicle_tts":
                    if "vehicle_ids" in data_vals: vehicle_count = len(data_vals["vehicle_ids"])
                    elif traci_constants["geometry"]["vehicle_ids"] in subscribed_data: vehicle_count = len(list(subscribed_data[traci_constants["geometry"]["vehicle_ids"]]))
                    elif subscription_key in subscribed_data: vehicle_count = subscribed_data[subscription_key]
                    else: vehicle_count = g_class.getLastStepVehicleNumber(geometry_id)
                    data_vals[data_key] = vehicle_count * self.step_length

                case "avg_vehicle_length":
                    if subscription_key in subscribed_data: length = subscribed_data[subscription_key]
                    else: length = g_class.getLastStepLength(geometry_id)
                    
                    data_vals[data_key] = convert_units(length, "metres", self._s_dist_unit)
                
                case "halting_no":
                    if subscription_key in subscribed_data: halting_no = subscribed_data[subscription_key]
                    else: halting_no = g_class.getLastStepHaltingNumber(geometry_id)
                    data_vals[data_key] = halting_no

                case "vehicle_occupancy":
                    if subscription_key in subscribed_data: vehicle_occupancy = subscribed_data[subscription_key]
                    else: vehicle_occupancy = g_class.getLastStepOccupancy(geometry_id)
                    data_vals[data_key] = vehicle_occupancy

                case "curr_travel_time":
                    speed_key = "max_speed" if self.get_geometry_vals(geometry_id, "vehicle_count") == 0 else "vehicle_speed"
                    vals = self.get_geometry_vals(geometry_id, ("length", speed_key))
                    length, speed = vals["length"], vals[speed_key]
                    if self.units.name == "UK": speed = convert_units(speed, "mph", "kmph")
                    data_vals[data_key] = length / max(speed, 0.1)

                case "ff_travel_time":
                    vals = self.get_geometry_vals(geometry_id, ("length", "max_speed"))
                    length, speed = vals["length"], vals["max_speed"]
                    if self.units.name == "UK": speed = convert_units(speed, "mph", "kmph")
                    data_vals[data_key] = length / speed

                case "emissions":
                    data_vals[data_key] = ({"CO2": g_class.getCO2Emission(geometry_id), "CO": g_class.getCO2Emission(geometry_id), "HC": g_class.getHCEmission(geometry_id),
                                            "PMx": g_class.getPMxEmission(geometry_id), "NOx": g_class.getNOxEmission(geometry_id)})

                case "length":
                    if g_name == "edge": length = self._edge_info[geometry_id][data_key]
                    else: length = self._lane_info[geometry_id][data_key]

                    data_vals[data_key] = length

                case "max_speed":
                    if geometry_id in self._edge_info: max_speed = self._edge_info[geometry_id][data_key]
                    elif geometry_id in self._lane_info: max_speed = self._lane_info[geometry_id][data_key]
                    else: max_speed = g_class.getMaxSpeed(geometry_id)

                    data_vals[data_key] = max_speed
                
                case "connected_edges":
                    data_vals[data_key] = {'incoming': self._edge_info[geometry_id]["incoming_edges"],
                                            'outgoing': self._edge_info[geometry_id]["outgoing_edges"]}
                case "edge_id":
                    data_vals[data_key] = g_class.getEdgeID(geometry_id)

                case "n_links":
                    data_vals[data_key] = g_class.getLinkNumber(geometry_id)

                case "allowed":
                    data_vals[data_key] = g_class.getAllowed(geometry_id)

                case "disallowed":
                    data_vals[data_key] = g_class.getDisallowed(geometry_id)

                case "left_lc":
                    data_vals[data_key] = g_class.getChangePermissions(geometry_id, 0)

                case "right_lc":
                    data_vals[data_key] = g_class.getChangePermissions(geometry_id, 1)

                case _:
                    data_vals[data_key] = self._edge_info[geometry_id][data_key]

        if len(geometry_ids) == 1:
            if return_val: return list(data_vals.values())[0]
            else: return data_vals
        else:
            if return_val: all_data_vals[geometry_id] = list(data_vals.values())[0]
            else: all_data_vals[geometry_id] = data_vals

    return all_data_vals

def _set_geometry_vals(self, geometry_ids: str | list | tuple, **kwargs) -> None:
    """
    Calls the TraCI API to change a edge or lane's state.

    Args:
        `geometry_ids` (str, list, tuple): Edge or lane ID or list of edge or lane IDs
        `max_speed` (int, float): Set new max speed value
        `allowed` (list): List of allowed vehicle type IDs, empty list allows all (lane only)
        `disallowed` (list): List of disallowed vehicle type IDs
        `left_lc` (list): Set left lane changing vehicle permission with by vehicle_type IDs (lane only)
        `right_lc` (list): Set right lane changing vehicle permission with by vehicle_type IDs (lane only)
    """
    
    if isinstance(geometry_ids, str): geometry_ids = [geometry_ids]
    geometry_ids = validate_list_types(geometry_ids, str, param_name="geometry_ids", curr_sim_step=self.curr_step)

    for geometry_id in geometry_ids:
        if geometry_id in self._all_edges:   g_class, g_name = traci.edge, "edge"
        elif geometry_id in self._all_lanes: g_class, g_name = traci.lane, "lane"
        else:
            desc = "Unrecognised egde or lane ID given ('{0}').".format(geometry_id)
            raise_error(KeyError, desc, self.curr_step)
        
        for command, value in kwargs.items():

            error, desc = test_valid_string(command, valid_set_geometry_val_keys, "command")
            if error != None: raise_error(error, desc, self.curr_step)

            match command:
                case "max_speed":
                    if isinstance(value, (int, float)):
                        min_unit = "kmph" if self.units.name == "METRIC" else "mph"
                        min_value = round(convert_units(0.1, "m/s", min_unit), 2)
                        if value >= min_value:
                            
                            if g_name == "lane":
                                edge_id = self.get_geometry_vals(geometry_id, "edge_id")
                                self._lane_info[geometry_id]["max_speed"] = value
                                lane_ids = self._edge_info[edge_id]["lane_ids"]
                                self._edge_info[edge_id]["max_speed"] = sum([self._lane_info[l_id]["max_speed"] for l_id in lane_ids]) / len(lane_ids)
                            else:
                                lane_ids = self._edge_info[geometry_id]["lane_ids"]
                                for l_id in lane_ids: self._lane_info[l_id]["max_speed"] = value
                                self._edge_info[geometry_id]["max_speed"] = value

                            units = "kmph" if self.units.name == "METRIC" else "mph"
                            new_speed = convert_units(value, units, "m/s")
                            g_class.setMaxSpeed(geometry_id, new_speed)
                            
                        else:
                            desc = f"({command}): Invalid speed value '{value}{min_unit}' (must be >= {min_value}{min_unit})."
                            raise_error(ValueError, desc, self.curr_step)
                    else:
                        desc = "({0}): Invalid max_speed value '{1}' (must be [int | float], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                case "allowed":
                    if g_name != "lane":
                        desc = "({0}): Command is only valid for lanes.".format(command)
                        raise_error(ValueError, desc, self.curr_step)
                    if isinstance(value, (list, tuple)):
                        curr_allowed = list(g_class.getAllowed(geometry_id))
                        allowed = tuple(set(curr_allowed + list(value)))
                        g_class.setAllowed(geometry_id, allowed)
                        
                        curr_disallowed = list(g_class.getDisallowed(geometry_id))
                        disallowed = tuple(set(curr_disallowed) - set(value))
                        g_class.setDisallowed(geometry_id, disallowed)
                    else:
                        desc = "({0}): Invalid allowed value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "disallowed":
                    if g_name != "lane":
                        desc = "({0}): Command is only valid for lanes.".format(command)
                        raise_error(ValueError, desc, self.curr_step)
                    if isinstance(value, (list, tuple)):
                        curr_disallowed = list(g_class.getDisallowed(geometry_id))
                        disallowed = tuple(set(curr_disallowed + list(value)))
                        g_class.setDisallowed(geometry_id, disallowed)
                        
                        curr_allowed = list(g_class.getAllowed(geometry_id))
                        allowed = tuple(set(curr_allowed) - set(value))
                        g_class.setAllowed(geometry_id, allowed)
                    else:
                        desc = "({0}): Invalid disallowed value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "left_lc":
                    if g_name != "lane":
                        desc = "({0}): Command is only valid for lanes.".format(command)
                        raise_error(ValueError, desc, self.curr_step)
                    if isinstance(value, (list, tuple)):
                        g_class.setChangePermissions(geometry_id, value[0], 1)
                    else:
                        desc = "({0}): Invalid left_lc value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                
                case "right_lc":
                    if g_name != "lane":
                        desc = "({0}): Command is only valid for lanes.".format(command)
                        raise_error(ValueError, desc, self.curr_step)
                    if isinstance(value, (list, tuple)):
                        g_class.setChangePermissions(geometry_id, value[0], -1)
                    else:
                        desc = "({0}): Invalid right_lc value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

def _close_road(self, geometry_ids: str | list | tuple) -> None:
    """
    Close a edge/lane indefinitely from its ID.

    Args:
        `geometry_ids` (str, list, tuple): Edge/lane ID or list of edge/lane IDs
    """

    if not isinstance(self._closed_lanes, dict): self._closed_lanes = {}
    geometry_ids = [geometry_ids] if not isinstance(geometry_ids, (list, tuple)) else geometry_ids

    for geometry_id in geometry_ids:
        geometry_type = self.geometry_exists(geometry_id)
        if geometry_type == "lane":
            self._closed_lanes[geometry_id] = _close_lane(self, geometry_id)
        elif geometry_type == "edge":
            self._closed_lanes[geometry_id] = {}
            lane_ids = self.get_geometry_vals(geometry_id, "lane_ids")
            for lane_id in lane_ids:
                self._closed_lanes[geometry_id][lane_id] = _close_lane(self, lane_id)
        else:
            desc = f"Geometry ID '{geometry_id} not found."
            raise_error(KeyError, desc, self.curr_step)

def _close_lane(self, lane_id: str) -> dict:
    """
    Disallows all vehicles on a specific lane.

    Args:
        `lane_id` (str): Lane ID
    """

    lane_permissions = self.get_geometry_vals(lane_id, ("allowed", "disallowed"))
    self.set_geometry_vals(lane_id, disallowed=list(set(lane_permissions["allowed"] + lane_permissions["disallowed"])))
    return lane_permissions

def _open_road(self, geometry_ids: str | list | tuple) -> None:
    """
    Open a closed a edge/lane indefinitely from its ID.

    Args:
        `geometry_ids` (str, list, tuple): Edge/lane ID or list of edge/lane IDs
    """

    geometry_ids = [geometry_ids] if not isinstance(geometry_ids, (list, tuple)) else geometry_ids

    for geometry_id in geometry_ids:
        geometry_type = self.geometry_exists(geometry_id)
        if geometry_type == "lane":
            e_id = None
            if geometry_id in self._closed_lanes:
                self.set_geometry_vals(geometry_id, **self._closed_lanes[geometry_id])
                del self._closed_lanes[geometry_id]
                continue
            for g_id, g_data in self._closed_lanes.items():
                if isinstance(g_data, dict) and geometry_id in g_data:
                    e_id = g_id
                    break
            if e_id != None:
                self.set_geometry_vals(geometry_id, **self._closed_lanes[e_id][geometry_id])
                del self._closed_lanes[e_id][geometry_id]
            else:
                desc = f"Lane with ID '{geometry_id}' not closed."
                raise_error(ValueError, desc, self.curr_step)
        elif geometry_type == "edge":
            if geometry_id in self._closed_lanes:
                for lane_id in self._closed_lanes[geometry_id]:
                    self.set_geometry_vals(lane_id, **self._closed_lanes[geometry_id][lane_id])
                del self._closed_lanes[geometry_id]
            else:
                desc = f"Edge with ID '{geometry_id}' not closed."
                raise_error(ValueError, desc, self.curr_step)

        else:
            desc = f"Geometry ID '{geometry_id} not found."
            raise_error(KeyError, desc, self.curr_step)

def _get_last_step_geometry_vehicles(self, geometry_ids: str | list, vehicle_types: list | None = None, flatten: bool = False) -> dict | list:
    """
    Get the IDs of vehicles on a lane or egde, by geometry ID.
    
    Args:
        `geometry_ids` (str, list):  Edge/lane ID or list of IDs
        `vehicle_types` (list, optional): Included vehicle type IDs
        `flatten` (bool): If `True`, all IDs are returned in a 1D array, else a dict with vehicles for each edge/lane
    
    Returns:
        (dict, list): List containing all vehicle IDs or dictionary containing IDs by edge/lane
    """

    geometry_ids = [geometry_ids] if not isinstance(geometry_ids, list) else geometry_ids
    if len(geometry_ids) == 1: flatten = True
    vehicle_types = [vehicle_types] if vehicle_types != None and not isinstance(vehicle_types, list) else vehicle_types

    vehicle_ids = [] if flatten else {}
    for geometry_id in geometry_ids:

        g_vehicle_ids = self.get_geometry_vals(geometry_id, "vehicle_ids")
        if vehicle_types != None:
            g_vehicle_ids = [vehicle_id for vehicle_id in g_vehicle_ids if self.get_vehicle_vals(vehicle_id, "type") in vehicle_types]

        if flatten: vehicle_ids += g_vehicle_ids
        else: vehicle_ids[geometry_id] = g_vehicle_ids

    if flatten: vehicle_ids = list(set(vehicle_ids))

    return vehicle_ids