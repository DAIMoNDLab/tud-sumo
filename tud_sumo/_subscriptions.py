import traci
from .utils import *

def _add_vehicle_subscriptions(self, vehicle_ids: str | list | tuple, data_keys: str | list | tuple) -> None:
    """
    Creates a new subscription for certain variables for **specific vehicles**. Valid data keys are;
    '_speed_', '_is_stopped_', '_max_speed_', '_acceleration_', '_position_', '_altitude_', '_heading_',
    '_edge_id_', '_lane_idx_', '_route_id_', '_route_idx_', '_route_edges_'.

    Args:
        `vehicle_ids` (str, list, tuple): Vehicle ID or list of IDs
        `data_keys` (str, list, tuple): Data key or list of keys
    """

    if isinstance(data_keys, str): data_keys = [data_keys]
    data_keys = validate_list_types(data_keys, str, param_name="data_keys", curr_sim_step=self.curr_step)

    if isinstance(vehicle_ids, str): vehicle_ids = [vehicle_ids]
    vehicle_ids = validate_list_types(vehicle_ids, str, param_name="vehicle_ids", curr_sim_step=self.curr_step)
    
    for data_key in data_keys:
        error, desc = test_valid_string(data_key, list(traci_constants["vehicle"].keys()), "data key")
        if error != None: raise_error(error, desc, self.curr_step)

    for vehicle_id in vehicle_ids:
        if self.vehicle_exists(vehicle_id):
            
            # Subscriptions are added using the traci_constants dictionary in tud_sumo.utils
            subscription_vars = [traci_constants["vehicle"][data_key] for data_key in data_keys]
            if "leader_id" in subscription_vars or "leader_dist" in subscription_vars:
                if "leader_id" in subscription_vars: subscription_vars.remove("leader_id")
                if "leader_dist" in subscription_vars: subscription_vars.remove("leader_dist")
                traci.vehicle.subscribeLeader(vehicle_id, 100)
            traci.vehicle.subscribe(vehicle_id, subscription_vars)

        else:
            desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)

def _remove_vehicle_subscriptions(self, vehicle_ids: str | list | tuple) -> None:
    """
    Remove **all** active subscriptions for a vehicle or list of vehicles.
    
    Args:
        `vehicle_ids` (str, list, tuple): Vehicle ID or list of IDs
    """

    if isinstance(vehicle_ids, str): vehicle_ids = [vehicle_ids]
    vehicle_ids = validate_list_types(vehicle_ids, str, param_name="vehicle_ids", curr_sim_step=self.curr_step)

    for vehicle_id in vehicle_ids:
        if self.vehicle_exists(vehicle_id):
            traci.vehicle.unsubscribe(vehicle_id)
        else:
            desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)

def _add_detector_subscriptions(self, detector_ids: str | list | tuple, data_keys: str | list | tuple) -> None:
    """
    Creates a new subscription for certain variables for **specific detectors**. Valid data keys are;
    '_vehicle_count_', '_vehicle_ids_', '_speed_', '_halting_no_', '_occupancy_', '_last_detection_'.
    
    Args:
        `detector_id` (str, list, tuple): Detector ID or list of IDs
        `data_keys` (str, list, tuple): Data key or list of keys
    """

    if isinstance(data_keys, str): data_keys = [data_keys]
    data_keys = validate_list_types(data_keys, str, param_name="data_keys", curr_sim_step=self.curr_step)

    if isinstance(detector_ids, str): detector_ids = [detector_ids]
    detector_ids = validate_list_types(detector_ids, str, param_name="detector_ids", curr_sim_step=self.curr_step)

    for data_key in data_keys:
        error, desc = test_valid_string(data_key, list(traci_constants["detector"].keys()), "data key")
        if error != None: raise_error(error, desc, self.curr_step)
    
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

        # Subscriptions are added using the traci_constants dictionary in tud_sumo.utils
        subscription_vars = [traci_constants["detector"][data_key] for data_key in data_keys]
        d_class.subscribe(detector_id, subscription_vars)

def _remove_detector_subscriptions(self, detector_ids: str | list | tuple) -> None:
    """
    Remove **all** active subscriptions for a detector or list of detectors.
    
    Args:
        `detector_ids` (str, list, tuple): Detector ID or list of IDs
    """

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

            d_class.unsubscribe(detector_id)

def _add_geometry_subscriptions(self, geometry_ids: str | list | tuple, data_keys: str | list | tuple) -> None:
    """
    Creates a new subscription for geometry (edge/lane) variables. Valid data keys are;
    '_vehicle_count_', '_vehicle_ids_', '_vehicle_speed_', '_halting_no_', '_occupancy_'.
    
    Args:
        `geometry_ids` (str, list, tuple): Geometry ID or list of IDs
        `data_keys` (str, list, tuple): Data key or list of keys
    """

    if isinstance(data_keys, str): data_keys = [data_keys]
    data_keys = validate_list_types(data_keys, str, param_name="data_keys", curr_sim_step=self.curr_step)

    if isinstance(geometry_ids, str): geometry_ids = [geometry_ids]
    geometry_ids = validate_list_types(geometry_ids, str, param_name="geometry_ids", curr_sim_step=self.curr_step)

    for data_key in data_keys:
        error, desc = test_valid_string(data_key, list(traci_constants["geometry"].keys()), "data key")
        if error != None: raise_error(error, desc, self.curr_step)
    
    for geometry_id in geometry_ids:
        g_name = self.geometry_exists(geometry_id)
        if g_name == "edge": g_class = traci.edge
        elif g_name == "lane": g_class = traci.lane
        else:
            desc = "Geometry ID '{0}' not found.".format(geometry_id)
            raise_error(KeyError, desc, self.curr_step)

        # Subscriptions are added using the traci_constants dictionary in tud_sumo.utils
        subscription_vars = [traci_constants["geometry"][data_key] for data_key in data_keys]
        g_class.subscribe(geometry_id, subscription_vars)

def _remove_geometry_subscriptions(self, geometry_ids: str | list | tuple) -> None:
    """
    Remove **all** active subscriptions for a geometry object or list of geometry objects.
    
    Args:
        `geometry_ids` (str, list, tuple): Geometry ID or list of IDs
    """

    if isinstance(geometry_ids, str): geometry_ids = [geometry_ids]
    geometry_ids = validate_list_types(geometry_ids, str, param_name="geometry_ids", curr_sim_step=self.curr_step)

    for geometry_id in geometry_ids:
        g_name = self.geometry_exists(geometry_id)
        if g_name == "edge": g_class = traci.edge
        elif g_name == "lane": g_class = traci.lane
        else:
            desc = "Geometry ID '{0}' not found.".format(geometry_id)
            raise_error(KeyError, desc, self.curr_step)

        g_class.unsubscribe(geometry_id)