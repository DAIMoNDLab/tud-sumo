import json, os, math, inspect
from enum import Enum
from datetime import datetime, timedelta
import traci.constants as tc
import pickle as pkl
from difflib import SequenceMatcher

date_format = "%d/%m/%Y"
time_format = "%H:%M:%S"
datetime_format = "%d/%m/%Y, %H:%M:%S"

unit_desc = {"METRIC": "Metric (km, km/h)", "UK": "UK (km, mph)", "IMPERIAL": "Imperial (miles, mph)"}
time_desc = {"s": "Seconds", "m": "Minutes", "hr": "Hours"}

valid_data_keys = { "detector": {
                        "type": None,
                        "position": None,
                        "vehicle_count": tc.LAST_STEP_VEHICLE_NUMBER,
                        "vehicle_ids": tc.LAST_STEP_VEHICLE_ID_LIST,
                        "lsm_speed": tc.LAST_STEP_MEAN_SPEED,
                        "halting_no": tc.LAST_STEP_VEHICLE_HALTING_NUMBER,
                        "last_detection": tc.LAST_STEP_TIME_SINCE_DETECTION,
                        "lsm_occupancy": tc.LAST_STEP_OCCUPANCY
                    },
                    "vehicle": {
                        "type":          {"getter": True,  "setter": True, "tc": None},
                        "colour":        {"getter": False, "setter": True,  "tc": None},
                        "length":        {"getter": True,  "setter": False, "tc": None},
                        "highlight":     {"getter": False, "setter": True,  "tc": None},
                        "speed":         {"getter": True,  "setter": True,  "tc": tc.VAR_SPEED},
                        "is_stopped":    {"getter": True,  "setter": False, "tc": tc.VAR_SPEED},
                        "max_speed":     {"getter": True,  "setter": True,  "tc": tc.VAR_MAXSPEED},
                        "allowed_speed": {"getter": True,  "setter": False, "tc": tc.VAR_ALLOWED_SPEED},
                        "acceleration":  {"getter": True,  "setter": True,  "tc": tc.VAR_ACCELERATION},
                        "position":      {"getter": True,  "setter": False, "tc": tc.VAR_POSITION},
                        "altitude":      {"getter": True,  "setter": False, "tc": tc.VAR_POSITION3D},
                        "heading":       {"getter": True,  "setter": False, "tc": tc.VAR_ANGLE},
                        "departure":     {"getter": True,  "setter": False, "tc": None},
                        "edge_id":       {"getter": True,  "setter": False, "tc": tc.VAR_ROAD_ID},
                        "lane_id":       {"getter": True,  "setter": False, "tc": tc.VAR_LANE_ID},
                        "lane_idx":      {"getter": True,  "setter": True,  "tc": tc.VAR_LANE_INDEX},
                        "origin":        {"getter": True,  "setter": False, "tc": None},
                        "destination":   {"getter": True,  "setter": True,  "tc": tc.VAR_ROUTE},
                        "route_id":      {"getter": True,  "setter": True,  "tc": tc.VAR_ROUTE_ID},
                        "route_idx":     {"getter": True,  "setter": False, "tc": tc.VAR_ROUTE_INDEX},
                        "route_edges":   {"getter": True,  "setter": True,  "tc": None},
                        "leader_id":     {"getter": True,  "setter": False, "tc": tc.VAR_LEADER},
                        "leader_dist":   {"getter": True,  "setter": False, "tc": tc.VAR_LEADER},
                        "speed_safety_checks":   {"getter": False, "setter": True, "tc": None},
                        "lc_safety_checks":      {"getter": False, "setter": True, "tc": None}
                    },
                    "vehicle_type": ["vehicle_class", "colour", "length", "width", "height", "max_speed", "speed_factor", "speed_dev", #"mass",
                                     "min_gap", "acceleration", "deceleration", "tau", "max_lateral_speed", "emission_class", "gui_shape"
                    ],
                    "geometry": {
                        "vehicle_count":        {"class": "both", "setter": False, "tc": tc.LAST_STEP_VEHICLE_NUMBER},
                        "vehicle_ids":          {"class": "both", "setter": False, "tc": tc.LAST_STEP_VEHICLE_ID_LIST},
                        "avg_vehicle_length":   {"class": "both", "setter": False, "tc": tc.LAST_STEP_LENGTH},
                        "halting_no":           {"class": "both", "setter": False, "tc": tc.LAST_STEP_VEHICLE_HALTING_NUMBER},
                        "vehicle_speed":        {"class": "both", "setter": False, "tc": tc.LAST_STEP_MEAN_SPEED},
                        "vehicle_occupancy":    {"class": "both", "setter": False, "tc": tc.LAST_STEP_OCCUPANCY},
                        #"vehicle_flow":         {"class": "both", "setter": False, "tc": None},
                        #"vehicle_density":      {"class": "both", "setter": False, "tc": None},
                        #"vehicle_delay":        {"class": "both", "setter": False, "tc": None},
                        "curr_travel_time":     {"class": "both", "setter": False, "tc": None},
                        "ff_travel_time":       {"class": "both", "setter": False, "tc": None},
                        "emissions":            {"class": "both", "setter": False, "tc": None},
                        "length":               {"class": "both", "setter": False, "tc": None},
                        "max_speed":            {"class": "both", "setter": True,  "tc": None},
                        "connected_edges":      {"class": "edge", "setter": False, "tc": None},
                        "incoming_edges":       {"class": "edge", "setter": False, "tc": None},
                        "outgoing_edges":       {"class": "edge", "setter": False, "tc": None},
                        "junction_ids":         {"class": "edge", "setter": False, "tc": None},
                        "linestring":           {"class": "edge", "setter": False, "tc": None},
                        "street_name":          {"class": "edge", "setter": False, "tc": None},
                        "n_lanes":              {"class": "edge", "setter": False, "tc": None},
                        "lane_ids":             {"class": "edge", "setter": False, "tc": None},
                        "edge_id":              {"class": "lane", "setter": False, "tc": None},
                        "n_links":              {"class": "lane", "setter": False, "tc": None},
                        "allowed":              {"class": "lane", "setter": True,  "tc": None},
                        "disallowed":           {"class": "lane", "setter": True,  "tc": None},
                        "left_lc":              {"class": "lane", "setter": True,  "tc": None},
                        "right_lc":             {"class": "lane", "setter": True,  "tc": None},
                    }}

traci_constants = {}

valid_detector_val_keys = list(valid_data_keys["detector"].keys())
traci_constants["detector"] = {data_key: tc for data_key, tc in valid_data_keys["detector"].items() if tc != None}

valid_set_vehicle_val_keys = [data_key for data_key, cfg in valid_data_keys["vehicle"].items() if cfg["setter"]]
valid_get_vehicle_val_keys = [data_key for data_key, cfg in valid_data_keys["vehicle"].items() if cfg["getter"]]
traci_constants["vehicle"] = {data_key: cfg["tc"] for data_key, cfg in valid_data_keys["vehicle"].items() if cfg["tc"] != None}

valid_vehicle_type_val_keys = valid_data_keys["vehicle_type"]

valid_get_edge_val_keys = [data_key for data_key, cfg in valid_data_keys["geometry"].items() if cfg["class"] != "lane"]
valid_get_lane_val_keys = [data_key for data_key, cfg in valid_data_keys["geometry"].items() if cfg["class"] != "edge"]
valid_set_geometry_val_keys = [data_key for data_key, cfg in valid_data_keys["geometry"].items() if cfg["setter"]]
traci_constants["geometry"] = {data_key: cfg["tc"] for data_key, cfg in valid_data_keys["geometry"].items() if cfg["tc"] != None}

sumo_colours = {
        "red": (255, 0, 0, 255),
        "green": (0, 255, 0, 255),
        "blue": (0, 0, 255, 255),
        "yellow": (255, 255, 0, 255),
        "cyan": (0, 255, 255, 255),
        "magenta": (255, 0, 255, 255),
        "orange": (255, 128, 0, 255),
        "white": (255, 255, 255, 255),
        "black": (0, 0, 0, 255),
        "grey": (128, 128, 128, 255),
        "gray": (128, 128, 128, 255),
        "invisible": (0, 0, 0, 0)
}

class Units(Enum):
    METRIC = 1
    IMPERIAL = 2
    UK = 3

class Controller(Enum):
    VSL = 1
    RG = 2
    METER = 3

def _get_type_str(valid_types, connector=","):

    if isinstance(valid_types, (list, tuple)):
        types = []

        for vt in valid_types:
            if isinstance(vt, type): types.append(vt.__name__)
            elif isinstance(vt, tuple):
                vt_arr = []
                for t in vt:
                    if isinstance(t, type): vt_arr.append(t.__name__)
                    else: vt_arr.append(type(t).__name__)
                types.append("[{0}]".format("|".join(vt_arr)))

        return "[{0}]".format(connector.join(types))
    
    elif isinstance(valid_types, type): return valid_types.__name__

    else: return valid_types

def validate_list_types(values, valid_types, element_wise=False, param_name=None, curr_sim_step=None, return_validity=False):

    invalid, desc = False, None
    param_name = param_name if param_name != None else "input list"

    inv_idx, inv_val = None, None

    if isinstance(values, (list, tuple)) and isinstance(valid_types, tuple):
        if element_wise:
            if len(values) == len(valid_types):
                for idx, (value, valid_type) in enumerate(zip(values, valid_types)):
                    if not isinstance(value, valid_type):
                        invalid, inv_idx, inv_val = True, idx, value
                        valid_types = valid_type
                        break
            else:
                desc = "Invalid {0} (must have length '{1}').".format(param_name, len(valid_types))
                invalid = True

        else:
            for idx, value in enumerate(values):
                if type(value) not in valid_types:
                    invalid, inv_idx, inv_val = True, idx, value
                    break

    if isinstance(values, (list, tuple)):
        for idx, value in enumerate(values):
            if not isinstance(value, valid_types):
                invalid, inv_idx, inv_val = True, idx, value
                break

    else:
        invalid = True
        if not return_validity:
            if desc == None: desc = "Invalid {0} (must be '[list|tuple]', not '{1}').".format(param_name, type(values).__name__)
            if 'self' in inspect.currentframe().f_back.f_locals:
                caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
            else: caller = ""
            caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
            error_msg = "{0}: {1}".format(caller, desc)
            if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
            raise TypeError(error_msg)
    
    if invalid and not return_validity:
        if desc == None: desc = "Invalid value found in {0} at idx {1} (must be '{3}', '{2}' is '{4}').".format(param_name, inv_idx, inv_val, _get_type_str(valid_types, "|"), type(value).__name__)
        if 'self' in inspect.currentframe().f_back.f_locals:
            caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
        else: caller = ""
        caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
        error_msg = "{0}: {1}".format(caller, desc)
        if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
        raise TypeError(error_msg)
    
    if return_validity: return invalid
    else: return values

def validate_type(value, valid_types, param_name=None, curr_sim_step=None, return_validity=False):

    if not isinstance(value, valid_types) or (isinstance(value, tuple) and type(value) not in valid_types):
        if return_validity: return False
        else:
            param_name = param_name if param_name != None else "input"
            val_str = "" if len(str(value)) >= 20 else "'{0}' ".format(value)
            desc = "Invalid {0} {1}(must be '{2}', not '{3}').".format(param_name, val_str, _get_type_str(valid_types, "|"), type(value).__name__)
            if 'self' in inspect.currentframe().f_back.f_locals:
                caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
            else: caller = ""
            caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
            error_msg = "{0}: {1}".format(caller, desc)
            if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
            raise TypeError(error_msg)
    
    if return_validity: return True
    else: return value

def raise_error(error, desc, curr_sim_step=None):
    if 'self' in inspect.currentframe().f_back.f_locals:
        caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
    else: caller = ""
    caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
    error_msg = "{0}: {1}".format(caller, desc)
    if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
    raise error(error_msg)

def raise_warning(desc, curr_sim_step=None):
    if 'self' in inspect.currentframe().f_back.f_locals:
        caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
    else: caller = ""
    caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
    warning_msg = "(WARNING) {0}: {1}".format(caller, desc)
    if curr_sim_step != None: warning_msg = "\n(step {0}) ".format(curr_sim_step) + warning_msg
    print(warning_msg)

def convert_units(values, orig_units, new_units, step_length=1, keep_arr=False):

    if orig_units == new_units: return values
    
    if isinstance(values, (int, float)): values = [values]
    elif not isinstance(values, (list, tuple)):
        desc = "Invalid values '{0}' type (must be [int|float|list|tuple], not '{1}').".format(values, type(values).__name__)
        raise_error(TypeError, desc)
    elif False in [isinstance(val, (int, float)) for val in values]:
        desc = "Invalid values '{0}' type (list must be only contain [int|float]).".format(values)
        raise_error(ValueError, desc)

    c_mats = {"distance":
                  {"units": ["metres", "kilometres", "yards", "feet", "miles"],
                   "matrix": [[1, 1/1000, 1.093613, 3.280840, 1/1609.344498], # m
                              [1000, 1, 1093.613298, 3280.839895, 1/1.609344], # km
                              [1/1.093613,  1/1093.613298, 1, 3, 1/1760], # yd
                              [1/3.280840,  1/3280.839895, 1/3, 1, 1/5280], # ft
                              [1609.344498, 1.609344, 1760, 5280, 1]] # mi
                  },
              "speed":
                  {"units": ["m/s", "kmph", "mph"],
                   "matrix": [[1, 3.6, 2.236936], # m/s
                              [1/3.6, 1, 1/1.609344], # kmph
                              [1/2.236936, 1.609344, 1]] # mph
                  },
              "time":
                  {"units": ["steps", "seconds", "minutes", "hours"],
                   "matrix": [[1, step_length, step_length/60, step_length/3600], # steps
                              [1/step_length, 1, 1/60, 1/3600], # s
                              [60/step_length, 60, 1, 1/60], # m
                              [3600/step_length, 3600, 60, 1]] # hr
                  },
               "weight":
                  {"units": ["kilograms", "pounds"],
                   "matrix": [[1, 2.204623], # kg
                              [1/2.204623, 1]] # lb
                  },
             }
    
    o_class = [key for key, val in c_mats.items() if orig_units in val["units"]]
    n_class = [key for key, val in c_mats.items() if new_units in val["units"]]

    if len(o_class) == 0:
        desc = "Unknown unit '{0}'.".format(orig_units)
        raise_error(ValueError, desc)
    else: o_class = o_class[0]

    if len(n_class) == 0:
        desc = "Unknown unit '{0}'.".format(new_units)
        raise_error(ValueError, desc)
    else: n_class = n_class[0]

    if o_class != n_class:
        desc = "Invalid conversion (cannot convert '{0}' ({1}) -> '{2}' ({3}))".format(o_class, orig_units, n_class, new_units)
        raise_error(ValueError, desc)
    
    o_idx = c_mats[o_class]["units"].index(orig_units)
    n_idx = c_mats[o_class]["units"].index(new_units)
    values = [value * c_mats[o_class]["matrix"][o_idx][n_idx] for value in values]
    if len(values) == 1 and not keep_arr: values = values[0]

    return values

def colour_to_rgba(colour, curr_step=None, err_prefix=None):

    err_prefix = "" if err_prefix == None else err_prefix
    if isinstance(colour, str):
        if colour in sumo_colours: return sumo_colours[colour]
        if "#" in colour: colour = colour.lstrip("#")
        if len(colour) != 6:
            desc = f"{err_prefix}'{colour}' is not a valid hex colour."
            raise_error(ValueError, desc, curr_step)
        colour = tuple(int(colour[i:i+2], 16) for i in (0, 2, 4))
    elif not isinstance(colour, (list, tuple)):
        desc = f"{err_prefix}Invalid colour (must be [str|list|tuple], not '{type(colour).__name__}')."
        raise_error(TypeError, desc, curr_step)
    elif len(colour) not in [3, 4] or all(x > 255 for x in colour) or all(x < 0 for x in colour):
        desc = f"{err_prefix}'{colour}' is not a valid RGB or RGBA colour."
        raise_error(ValueError, desc, curr_step)
    
    if len(colour) == 3: colour = list(colour) + [255]

    return colour

def get_time_steps(data_vals, unit, step_len=None, start=0):
    time_vals = list(range(len(data_vals)))
    time_vals = [val + start for val in time_vals]

    return convert_units(time_vals, "steps", unit, step_len)
         
def get_time_str():
    date_str = datetime.now().strftime(datetime_format)
    return date_str

def get_sim_time_str(step_no, step_len) -> str:
    seconds = step_no * step_len
    return str(timedelta(seconds = seconds))

def get_cumulative_arr(arr: list, start: int=0) -> list:
    arr = [0] + arr
    for i in range(start + 1, len(arr)):
        arr[i] += arr[i - 1]
    return arr[1:]

def get_scenario_name(filepath: str) -> str:
    """
    Get scenario name from filepath.
    
    Args:
        `filepath` (str): SUMO filename
    
    Returns:
        str: Scenario name
    """
    cfg_file = filepath.split('/')[-1]
    if cfg_file.endswith('.sumocfg'): cfg_file = cfg_file.removesuffix('.sumocfg')
    elif cfg_file.endswith('.neteditcfg'): cfg_file = cfg_file.removesuffix('.neteditcfg')
    elif cfg_file.endswith('.add.xml'): cfg_file = cfg_file.removesuffix('.add.xml')
    elif cfg_file.endswith('.rou.xml'): cfg_file = cfg_file.removesuffix('.rou.xml')
    elif cfg_file.endswith('.net.xml'): cfg_file = cfg_file.removesuffix('.net.xml')
    return cfg_file

def load_params(parameters: str|dict, params_name: str|None = None, step: int|None = None) -> dict:
    """
    Load parameters file. Handles either dict or json file.
    
    Args:
        `parameters` (str, dict): Parameters dict or filepath
        `params_name` (str, optional): Parameter dict function (for error messages)
        `step` (int, optional): Current simulation step (for error messages)
    
    Returns:
        dict: Parameters dict
    """

    caller = "{0}.{1}()".format(inspect.currentframe().f_back.f_locals['self'].__name__(), inspect.currentframe().f_back.f_code.co_name)
    if step != None: caller = "(step {0}) {1}".format(step, caller)
    if params_name == None: params_name = "parameter input"

    if not isinstance(parameters, (dict, str)):
        raise TypeError("{0}: Invalid {1} (must be [dict|filepath (str)], not '{2}').".format(caller, params_name, type(parameters).__name__))
    elif isinstance(parameters, str):
        if parameters.endswith(".json"): r_class, r_mode = json, "r"
        elif parameters.endswith(".pkl"): r_class, r_mode = pkl, "rb"    
        else:
            raise ValueError("{0}: Invalid parameters file '{1}' (must be '.json' or '.pkl' file).".format(caller, parameters))

        if os.path.exists(parameters):
            with open(parameters, r_mode) as fp:
                parameters = r_class.load(fp)
        else: raise FileNotFoundError("{0}: Parameters file '{1}' not found.".format(caller, parameters))

    return parameters

def get_aggregated_data(data_vals, time_steps, interval, avg=True, min0=True):
    """
    Args:
        `data_vals` (list): Data values array
        `time_steps` (list): List of time step values
        `interval` (int): Aggregation interval
        `avg` (bool): Denotes whether to average/sum values

    Returns:
        (list, list): Aggregated data, time steps
    """
    
    agg_start, agg_data, agg_steps = 0, [], []
    while agg_start < len(data_vals):
        period_data = data_vals[agg_start:int(min(agg_start+interval, len(data_vals)))]
        if min0: period_data = [max(val, 0) for val in period_data]
        if avg: agg_data.append(sum(period_data) / len(period_data))
        else: agg_data.append(sum(period_data))
        period_data = time_steps[agg_start:int(min(agg_start+interval, len(time_steps)))]
        agg_steps.append(period_data[-1])
        agg_start += interval

    return agg_data, agg_steps

def get_axis_lim(data_vals, end_buff = 0.05):
    """
    Get axis limit rounded to nearest 1000/100/10 (with buffer).
    
    Args:
        `data_vals`: Single (max) axis value, or list of values
        `end_buff`: Minimum axis buffer above maximum value (default to 5%)
    
    Returns:
        float: Axis limit
    """
    
    pct_buff = 1 + end_buff
    if isinstance(data_vals, (list, tuple)): max_val = max(data_vals)
    else:
        max_val = data_vals
        data_vals = [data_vals]

    if max_val == min(data_vals) == 0: return 1

    for scale in [1000, 100, 10, 1]:
        if max_val >= scale:
            return math.ceil((max_val * pct_buff) / (scale / 5)) * (scale / 5)
        
    return max_val * pct_buff

def limit_vals_by_range(time_steps, data_vals=None, time_range=None) -> list|tuple:
    """
    For plotting, to limit data values between to those within a given time range.
    
    Args:
        `time_steps`: List of time step values
        `data_vals`: List of data values, same length as time_steps, (if not given, only time_steps is limited)
        `time_range`: (1x2) array containing minimum and maximum time step values (if not given, data is returned unchanged)
    
    Returns:
        list: Limited time_steps (& data_vals if given, where tuple returned is (steps, vals))
    """

    if time_range == None or (time_range[0] < time_steps[0] and time_range[1] > time_steps[-1]):
        if data_vals != None: return time_steps, data_vals
        else: return time_steps

    if data_vals != None:
        new_vals, new_steps = [], []
        for val, step in zip(data_vals, time_steps):
            if step >= time_range[0] and step <= time_range[1]:
                new_vals.append(val)
                new_steps.append(step)
            elif step > time_range[1]:
                return new_steps, new_vals
    else: return [step for step in time_steps if step >= time_range[0] and step <= time_range[1]]

def get_most_similar_string(input_string, valid_strings, req_similarity=0.6):
    best_match, best_similarity = None, 0
    for valid_string in valid_strings:
        similarity = SequenceMatcher(None, input_string, valid_string).ratio()
        if similarity > best_similarity and similarity >= req_similarity:
            best_match, best_similarity = valid_string, similarity
    return best_match

def test_valid_string(input_string, valid_strings, param_name, case_sensitive=True, req_similarity=0.6):
    if not isinstance(input_string, str):
        desc = "Invalid {0} '{1}' (must be 'str', not '{2}').".format(param_name, input_string, type(input_string).__name__)
        return TypeError, desc

    if case_sensitive: found = input_string in valid_strings
    elif not case_sensitive: found = input_string.upper() in [string.upper() for string in valid_strings]
    else: found = False

    if not found:
        valid_strings.sort()
        desc = "Unknown {0} '{1}'".format(param_name, input_string)
        closest = get_most_similar_string(input_string, valid_strings, req_similarity)
        if closest != None: desc = desc + ". Did you mean '{0}'?".format(closest)
        else: desc = desc + " (must be ['{0}']).".format("'|'".join(valid_strings))

        return ValueError, desc

    return None, None

def test_input_dict(input_dict, valid_params, dict_name="", required=None) -> str:

    desc = None
    dict_name = dict_name + " " if dict_name != "" else dict_name

    if len(input_dict) == 0:
        desc = "Empty {0}parameters given.".format(dict_name)
        return (ValueError, desc)
    
    if isinstance(required, bool) and required: required = list(valid_params.keys())
    if required != None:
        missing_keys = list(set(required) - set(input_dict.keys()))
        if len(missing_keys) > 0:
            desc = "Missing required {0}parameters ('{1}').".format(dict_name, "', '".join(list(missing_keys)))
            return (KeyError, desc)
        
    for key, item in input_dict.items():
        if key not in valid_params:
            desc = "Unrecognised {0}parameter '{1}'".format(dict_name, key)
            close_match = get_most_similar_string(key, valid_params.keys())
            desc = "{0}. Did you mean '{1}'?".format(desc, close_match) if close_match != None else desc + "."
            return (KeyError, desc)
        if not isinstance(item, valid_params[key]):
            if isinstance(valid_params[key], (list, tuple)):
                type_str = "[{0}]".format("|".join([str(val.__name__) for val in valid_params[key]]))
            else: type_str == valid_params[key].__name__
            desc = "Invalid {0} type '{1}' (must be '{2}' not '{3}')".format(key, item, type_str, type(item).__name__)
            return (TypeError, desc)
        
    return (None, None)