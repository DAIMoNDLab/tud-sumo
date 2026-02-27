import json, os, pickle as pkl
from .demand import DemandProfile
from .utils import *

def _save_objects(self, filename: str | None = None, overwrite: bool = True, json_indent: int = 4) -> None:
    """
    Save parameters of all TUD-SUMO objects created for the simulation (tracked edges/junctions, phases, controllers, events, demand, routes).

    Args:
        `filename` (str, optional): Output filename ('_.json_' or '_.pkl_')
        `overwrite` (bool): Denotes whether to allow overwriting previous outputs
        `json_indent` (int, optional): Indent used when saving JSON files
    """

    object_params = {}

    if len(self.tracked_edges) > 0:
        e_ids = list(self.tracked_edges.keys())
        object_params["edges"] = e_ids if len(e_ids) > 1 else e_ids[0]
    
    if len(self.tracked_junctions) > 0:
        object_params["junctions"] = {junc_id: junc._init_params for junc_id, junc in self.tracked_junctions.items()}

    if self._junc_phases != None:
        junc_phases = {}
        for junc_id, phase_dict in self._junc_phases.items():
            if junc_id in self.tracked_junctions and self.tracked_junctions[junc_id]._is_meter: continue
            junc_phases[junc_id] = {key: phase_dict[key] for key in ["phases", "times"]}
        if len(junc_phases) > 0:
            object_params["phases"] = junc_phases

    if len(self.controllers) > 0:
        object_params["controllers"] = {c_id: c._init_params for c_id, c in self.controllers.items()}

    if self._scheduler != None:
        events = self._scheduler.get_events()
        if len(events) > 0: object_params["events"] = {e.id: e._init_params for e in events}

    if len(self._demand_profiles) > 0:
        all_files = []
        for dp in self._demand_profiles.values():
            dp_dict = {k: v for k, v in dp.__dict__.items() if k not in ['id', 'sim', 'step_length', '_demand_headers']}
            all_files.append(dp_dict)
        object_params["demand"] = all_files if len(all_files) > 1 else all_files[0]

    if len(self._new_routes) > 0:
        object_params["routes"] = self._new_routes

    if filename == None:
        if self.scenario_name != None:
            filename = self.scenario_name
        else:
            desc = "No filename given."
            raise_error(ValueError, desc, self.curr_step)

    if len(object_params) == 0:
        desc = "Object save file '{0}' could not be saved (no objects found).".format(filename)
        raise_error(KeyError, desc, self.curr_step)

    if filename.endswith(".json"):
        w_class, w_mode = json, "w"
    elif filename.endswith(".pkl"):
        w_class, w_mode = pkl, "wb"
    else:
        filename += ".json"
        w_class, w_mode = json, "w"

    if os.path.exists(filename) and overwrite:
        if not self._suppress_warnings: raise_warning("File '{0}' already exists and will be overwritten.".format(filename), self.curr_step)
    elif os.path.exists(filename) and not overwrite:
        desc = "File '{0}' already exists and cannot be overwritten.".format(filename)
        raise_error(FileExistsError, desc, self.curr_step)

    with open(filename, w_mode) as fp:
        if w_mode == "wb": w_class.dump(object_params, fp)
        else: w_class.dump(object_params, fp, indent=json_indent)

def _load_objects(self, object_parameters: str | dict) -> None:
    """
    Load parameters of all TUD-SUMO objects created for the simulation (tracked edges/junctions, phases, controllers, events, demand, routes).
    
    Args:
        `object_parameters` (str, dict): Either dict containing object parameters or '_.json_'/'_.pkl_' filepath
    """
    
    object_parameters = validate_type(object_parameters, (str, dict), "object_parameters", self.curr_step)
    if isinstance(object_parameters, str):

        if object_parameters.endswith(".json"): r_class, r_mode = json, "r"
        elif object_parameters.endswith(".pkl"): r_class, r_mode = pkl, "rb"
        else:
            desc = "Invalid object parameter file '{0}' (must be '.json' or '.pkl).".format(object_parameters)
            raise_error(ValueError, desc, self.curr_step)

        if os.path.exists(object_parameters):
            with open(object_parameters, r_mode) as fp:
                object_parameters = r_class.load(fp)
        else:
            desc = "Object parameter file '{0}' not found.".format(object_parameters)
            raise_error(FileNotFoundError, desc, self.curr_step)

    valid_params = {"edges": (str, list), "junctions": dict, "phases": dict, "controllers": dict,
                    "events": dict, "demand": (str, list), "routes": dict}
    error, desc = test_input_dict(object_parameters, valid_params, "'object parameters")
    if error != None: raise_error(error, desc, self.curr_step)
    
    if "edges" in object_parameters:
        self.add_tracked_edges(object_parameters["edges"])

    if "junctions" in object_parameters:
        self.add_tracked_junctions(object_parameters["junctions"])

    if "phases" in object_parameters:
        self.set_phases(object_parameters["phases"], overwrite=True)
    
    if "controllers" in object_parameters:
        self.add_controllers(object_parameters["controllers"])

    if "events" in object_parameters:
        self.add_events(object_parameters["events"])

    if "demand" in object_parameters:
        _ = self.load_demand_profiles(object_parameters["demand"])

    if "routes" in object_parameters:

        for r_id, route in object_parameters["routes"].items():
            self.add_route(route, r_id)

def _load_demand_profiles(self, demand_profiles: str | dict | list | tuple) -> list:
    """
    Load demand profile(s) into the simulation.

    Args:
        `demand_profiles` (str, list, tuple): Either a filename to a previously saved DemandProfile object, or list of filenames

    Returns:
        list: List of DemandProfile objects
    """

    if not isinstance(demand_profiles, (list, tuple)): demand_profiles = [demand_profiles]
    validate_list_types(demand_profiles, (str, dict), param_name='demand_profiles', curr_sim_step=self.curr_step)

    loaded = []
    for dp_f in demand_profiles:

        if isinstance(dp_f, str):
            if not os.path.exists(dp_f):
                desc = f"DemandProfile file '{dp_f}' not found."
                raise_error(FileNotFoundError, desc, self.curr_step)
            
            with open(dp_f, "rb") as fp:
                dp_dict = pkl.load(fp)

        else: dp_dict = dp_f

        demand_profile = DemandProfile(self)
        demand_profile._demand_arrs = dp_dict['_demand_arrs']
        demand_profile._vehicle_types = dp_dict['_vehicle_types']

        for vehicle_type_id, vehicle_type_data in demand_profile._vehicle_types.items():
            
            if not self.vehicle_type_exists(vehicle_type_id):
                self.add_vehicle_type(vehicle_type_id, **vehicle_type_data)

        for demand_arr in demand_profile._demand_arrs:
            routing = demand_arr[0]
            
            if isinstance(routing, str) and self.route_exists(routing) == None:
                desc = f"Route with ID '{routing}' not found."
                raise_error(KeyError, desc, self.curr_step)
            
            elif isinstance(routing, (list, tuple)) and len(routing) == 2:
                e_ids = {e: e_id for e, e_id in zip(["Origin", "Destination"], routing)}
                for e, e_id in e_ids.items():
                    if self.geometry_exists(e_id) == None:
                        desc = f"{e} edge '{e_id}' not found."
                        raise_error(KeyError, desc, self.curr_step)

            else:
                desc = f"Invalid routing '{routing}'."
                raise_error(ValueError, desc, self.curr_step)

        demand_profile.step_length = self.step_length

        self._demand_profiles[demand_profile.id] = demand_profile

        self._manual_flow = True

        loaded.append(demand_profile)

    if len(loaded) == 0: loaded = loaded[0]
    return loaded

def _save_data(self, filename: str | None = None, overwrite: bool = True, json_indent: int | None = 4) -> None:
    """
    Save all vehicle, detector and junction data in a JSON or pickle file.
    
    Args:
        `filename` (str, optional): Output filepath (defaults to '_./{scenario_name}.pkl_')
        `overwrite` (bool): Prevent previous outputs being overwritten
        `json_indent` (int, optional): Indent used when saving JSON files
    """

    filename, w_class, w_mode, warning, error = _get_save_filename(self, filename, overwrite)

    if warning != None: raise_warning(warning, self.curr_step)
    elif error != None:
        desc = f"File '{filename}' already exists and cannot be overwritten."
        raise_error(error, desc, self.curr_step)

    if self._scheduler != None: self._all_data["data"]["events"] = self._scheduler.__dict__()
    with open(filename, w_mode) as fp:
        if w_mode == "wb": w_class.dump(self._all_data, fp)
        else: w_class.dump(self._all_data, fp, indent=json_indent)
        
def _save_fc_data(self, filename: str | None = None, overwrite: bool = True, json_indent: int | None = 4) -> None:
    """
    Save all floating car data in a JSON or pickle file.

    Args:
        `filename` (str, optional): Output filepath (defaults to '_./{scenario_name}_fc_data.pkl_')
        `overwrite` (bool): Prevent previous outputs being overwritten
        `json_indent` (int, optional): Indent used when saving JSON files
    """

    if not self._get_fc_data:
        desc = "No floating car data collected."
        raise_error(KeyError, desc, self.curr_step)

    filename, w_class, w_mode, warning, error = _get_save_filename(self, filename, overwrite, "_fc_data")

    if warning != None: raise_warning(warning, self.curr_step)
    elif error != None:
        desc = f"File '{filename}' already exists and cannot be overwritten."
        raise_error(error, desc, self.curr_step)

    static_keys = ["type", "departure", "destination", "origin"]
    dynamic_keys = ["longitude", "latitude", "altitude", "heading", "speed", "lane_id"]

    new_data, veh_info = {}, {}
    for key, value in self._all_data.items():
        if key != "data": new_data[key] = value
        else: 

            fc_data = []

            for step in value["fc_data"]:
                step_data = {}
                for veh_id, veh_data in step.items():
                    if veh_id not in veh_info: veh_info[veh_id] = {sk: veh_data[sk] for sk in static_keys}
                    step_data[veh_id] = {dk: veh_data[dk] for dk in dynamic_keys}

                fc_data.append(step_data)

            new_data["veh_info"] = veh_info
            new_data["fc_data"] = fc_data


    with open(filename, w_mode) as fp:
        if w_mode == "wb": w_class.dump(new_data, fp)
        else: w_class.dump(new_data, fp, indent=json_indent)

def _get_save_filename(self, filename, overwrite, def_suffix=""):

    if self._all_data == None:
        desc = "No data to save as a simulation has not been run."
        raise_error(SimulationError, desc, self.curr_step)

    if filename == None:
        if self.scenario_name != None:
            filename = self.scenario_name
        else:
            desc = "No filename given."
            raise_error(ValueError, desc, self.curr_step)

    if filename.endswith(".json"):
        w_class, w_mode = json, "w"
    elif filename.endswith(".pkl"):
        w_class, w_mode = pkl, "wb"
    else:
        filename += f"{def_suffix}.pkl"
        w_class, w_mode = pkl, "wb"

    warning, error = None, None
    if os.path.exists(filename) and overwrite:
        if not self._suppress_warnings: warning = f"File '{filename}' already exists and will be overwritten."
    elif os.path.exists(filename) and not overwrite: error = FileExistsError

    return filename, w_class, w_mode, warning, error