import os, sys, io, traci, sumolib, json, pickle, math, inspect, importlib.util
import pickle as pkl
import numpy as np
from tqdm import tqdm
from copy import copy, deepcopy
from random import choices, choice, random, seed as set_seed
from .events import EventScheduler, Event
from .controllers import VSLController, RGController
from .videos import Recorder
from .demand import DemandProfile
from shapely.geometry import LineString, Point
from .utils import *

class Simulation:
    """ Main simulation interface."""

    def __init__(self, scenario_name: str|None = None, scenario_desc: str|None = None, verbose: bool = True) -> None:
        """
        Args:
            `scenario_name` (str, optional):Scenario label saved to simulation object (defaults to name of '_.sumocfg_')
            `scenario_desc` (str, optional):Simulation scenario description, saved along with all files
            `verbose` (bool): Denotes whether to print simulation information
        """

        self.curr_step = 0
        self.curr_time = 0
        self.step_length = None
        self.units = Units(1)
        
        self.scenario_name = validate_type(scenario_name, (str, type(None)), "scenario_name")
        self.scenario_desc = validate_type(scenario_desc, (str, type(None)), "scenario_desc")

        self._verbose = verbose
        
        self._seed = None
        self._running = False
        self._gui = False
        self._pbar = None
        self._pbar_length = None
        self._start_called = False

        self._all_data = None

        self.track_juncs = False
        self.tracked_junctions = {}
        self._junc_phases = None
        self._all_juncs = []
        self._all_tls = []

        self._manual_flow = False

        self._demand_profiles = {}
        self._man_flow_id = 0

        self.controllers = {}
        self._scheduler = None

        self.tracked_edges = {}
        self.available_detectors = {}

        self._all_edges = None
        self._all_lanes = None
        self._all_routes = None
        self._new_routes = {}

        self._get_individual_vehicle_data = True
        self._all_curr_vehicle_ids = set([])
        self._all_loaded_vehicle_ids = set([])
        self._all_added_vehicles = set([])
        self._all_removed_vehicles = set([])
        self._vehicle_types = set([])
        self._added_vehicle_types = set([])
        self._known_vehicles = {}
        self._trips = {"incomplete": {}, "completed": {}}

        self._v_in_funcs = []
        self._v_out_funcs = []
        self._v_func_params = {}
        self._valid_v_func_params = ["simulation", "curr_step", "vehicle_id", "route_id",
                                     "vehicle_type", "departure", "origin", "destination"] 
        
        self._default_view = 'View #0'
        self._gui_views = []
        self._gui_veh_tracking = {}
        self._recorder = None

        from .__init__ import __version__
        self._tuds_version = __version__

    def __str__(self):
        if self.scenario_name != None:
            return "<{0}: '{1}'>".format(self.__name__, self.scenario_name)
        else: return "<{0}>".format(self.__name__)

    def __name__(self): return "Simulation"

    def __dict__(self): return {} if self._all_data == None else self._all_data

    def start(self, config_file: str|None = None, net_file: str|None = None, route_file: str|None = None, add_file: str|None = None, gui_file: str|None = None, cmd_options: list|None = None, units: str|int = 1, get_individual_vehicle_data: bool = True, automatic_subscriptions: bool = True, suppress_warnings: bool = False, suppress_traci_warnings: bool = True, suppress_pbar: bool = False, seed: str = "random", gui: bool = False, sumo_home: str|None = None) -> None:
        """
        Intialises SUMO simulation.

        Args:
            `config_file` (str, optional): Location of '_.sumocfg_' file (can be given instead of net_file)
            `net_file` (str, optional): Location of '_.net.xml_' file (can be given instead of config_file)
            `route_file` (str, optional): Location of '_.rou.xml_' route file
            `add_file` (str, optional): Location of '_.add.xml_' additional file
            `gui_file` (str, optional): Location of '_.xml_' gui (view settings) file
            `cmd_options` (list, optional): List of any other command line options
            `units` (str, int): Data collection units [1 (metric) | 2 (IMPERIAL) | 3 (UK)] (defaults to 'metric')
            `get_individual_vehicle_data` (bool): Denotes whether to get individual vehicle data (set to `False` to improve performance)
            `automatic_subscriptions` (bool): Denotes whether to automatically subscribe to commonly used vehicle data (speed and position, defaults to `True`)
            `suppress_warnings` (bool): Suppress simulation warnings
            `suppress_traci_warnings` (bool): Suppress warnings from TraCI
            `suppress_pbar` (bool): Suppress automatic progress bar when not using the GUI
            `seed` (bool): Either int to be used as seed, or `random.random()`/`random.randint()`, where a random seed is used
            `gui` (bool): Bool denoting whether to run GUI
            `sumo_home` (str, optional):SUMO base directory, if the `$SUMO_HOME` variable is not already set within the environment
        """

        if isinstance(sumo_home, str):
            if os.path.exists(sumo_home):
                os.environ["SUMO_HOME"] = sumo_home
            else:
                desc = "SUMO_HOME filepath '{0}' does not exist.".format(sumo_home)
                raise_error(FileNotFoundError, desc)
        elif sumo_home != None:
            desc = "Invalid SUMO_HOME '{0}' (must be str, not '{1}').".format(sumo_home, type(sumo_home).__name__)
            raise_error(TypeError, desc)

        if "SUMO_HOME" in os.environ:
            path_tools = os.path.join(os.environ.get("SUMO_HOME"), 'tools')
        else:
            desc = "Environment SUMO_HOME variable not set."
            raise_error(SimulationError, desc)

        if path_tools in sys.path: pass
        else: sys.path.append(path_tools)

        self._start_called = True
        self._gui = gui
        sumoCMD = ["sumo-gui"] if self._gui else ["sumo"]

        if config_file == net_file == None:
            desc = "Either config or network file required."
            raise_error(ValueError, desc)
        
        if config_file != None:
            if config_file.endswith(".sumocfg"):
                self._sumo_cfg = config_file
                sumoCMD += ["-c", config_file]
            else:
                desc = "Invalid config file extension."
                raise_error(ValueError, desc)
        else:
            self._sumo_cfg = None
            sumoCMD += ["-n", net_file]
            
        if route_file != None: sumoCMD += ["-r", route_file]
        if add_file != None: sumoCMD += ["-a", add_file]
        if gui_file != None: sumoCMD += ["-c", gui_file]
        
        if self.scenario_name == None:
            for filename in [config_file, net_file, route_file, add_file]:
                if filename != None:
                    self.scenario_name = get_scenario_name(filename)
                    break

        if cmd_options != None: sumoCMD += cmd_options

        # Allow seed as either int or str (stil only valid digit or 'random')
        # Setting seed to "random" uses a random seed.
        if seed != None:
            seed = validate_type(seed, (str, int), "seed", self.curr_step)
            if isinstance(seed, str):
                if seed.isdigit():
                    sumoCMD += ["--seed", seed]
                    set_seed(int(seed))
                    np.random.seed(int(seed))
                    self._seed = int(seed)

                elif seed.upper() == "RANDOM":
                    sumoCMD.append("--random")
                    self._seed = "random"

                else:
                    desc = "Invalid seed '{0}' (must be valid 'int' or str 'random').".format(seed)
                    raise_error(ValueError, desc)

            elif isinstance(seed, int):
                sumoCMD += ["--seed", str(seed)]
                set_seed(seed)
                np.random.seed(seed)
                self._seed = seed

        else:
            self._seed = "random"

        # Suppress SUMO step log (and warnings)
        default_cmd_args = ["--no-step-log", "true"]
        if suppress_traci_warnings: default_cmd_args += ["--no-warnings", "true"]
        sumoCMD += default_cmd_args

        traci.start(sumoCMD)
        self._running = True
        self.step_length = float(traci.simulation.getOption("step-length"))

        if self._junc_phases != None: self._update_lights()

        # Get all static information for detectors (position, lanes etc.),
        # and add subscriptions for their data.
        for detector_id in list(traci.multientryexit.getIDList()):
            self.available_detectors[detector_id] = {'type': 'multientryexit', 'position': {'entry_lanes': traci.multientryexit.getEntryLanes(detector_id),
                                                                                            'exit_lanes': traci.multientryexit.getExitLanes(detector_id),
                                                                                            'entry_positions': traci.multientryexit.getEntryPositions(detector_id),
                                                                                            'exit_positions': traci.multientryexit.getExitPositions(detector_id)}}
            self.add_detector_subscriptions(detector_id, ["vehicle_ids", "lsm_speed"])
            
        for detector_id in list(traci.inductionloop.getIDList()):
            self.available_detectors[detector_id] = {'type': 'inductionloop', 'position': {'lane_id': traci.inductionloop.getLaneID(detector_id), 'position': traci.inductionloop.getPosition(detector_id)}}
            self.add_detector_subscriptions(detector_id, ["vehicle_ids", "lsm_speed", "lsm_occupancy"])

        units = validate_type(units, (str, int), "units", self.curr_step)
        if isinstance(units, str):
            valid_units = ["METRIC", "IMPERIAL", "UK"]
            error, desc = test_valid_string(units, valid_units, "simulation units", case_sensitive=False)
            if error != None: raise_error(error, desc)
            self.units = Units(valid_units.index(units.upper())+1)

        elif units in [1, 2, 3]: self.units = Units(units)

        match self.units.name:
            case "METRIC":
                self._speed_unit = "kmph"
                self._l_dist_unit = "kilometres"
                self._s_dist_unit = "metres"
                self._weight_unit = "kilograms"
            case "IMPERIAL":
                self._speed_unit = "mph"
                self._l_dist_unit = "miles"
                self._s_dist_unit = "feet"
                self._weight_unit = "pounds"
            case "UK":
                self._speed_unit = "mph"
                self._l_dist_unit = "kilometres"
                self._s_dist_unit = "metres"
                self._weight_unit = "kilograms"
        
        self._all_juncs = list(traci.junction.getIDList())
        self._all_tls = list(traci.trafficlight.getIDList())
        self._all_edges = list(traci.edge.getIDList())
        self._all_lanes = list(traci.lane.getIDList())
        self._vehicle_types = set(traci.vehicletype.getIDList())

        self._all_edges = [e_id for e_id in self._all_edges if not e_id.startswith(":")]
        self._all_lanes = [l_id for l_id in self._all_lanes if not l_id.startswith(":")]

        # Get network file using sumolib to fetch information about
        # the network itself.
        network_file = traci.simulation.getOption("net-file")
        self._network = sumolib.net.readNet(network_file, withInternal=True)

        self._lane_info = {}
        for l_id in self._all_lanes:
            lane = self._network.getLane(l_id)

            length = convert_units(LineString(lane.getShape()).length, "metres", self._l_dist_unit)
            max_speed = convert_units(traci.lane.getMaxSpeed(l_id), "m/s", self._speed_unit)

            self._lane_info[l_id] = {"length": length, "max_speed": max_speed}

        self._edge_info = {}
        for e_id in self._all_edges:
            edge = self._network.getEdge(e_id)
            lane_ids = [f"{e_id}_{idx}" for idx in range(traci.edge.getLaneNumber(e_id))]

            self._edge_info[e_id] = {"incoming_edges": [incoming.getID() for incoming in edge.getIncoming()],
                                     "outgoing_edges": [outgoing.getID() for outgoing in edge.getOutgoing()],
                                     "junction_ids": [edge.getFromNode().getID(), edge.getToNode().getID()],
                                     "linestring": edge.getShape(),
                                     "street_name": traci.edge.getStreetName(e_id),
                                     "n_lanes": len(lane_ids),
                                     "lane_ids": lane_ids
                                     }
            
            e_length = LineString(self._edge_info[e_id]["linestring"]).length
            self._edge_info[e_id]["length"] = convert_units(e_length, "metres", self._l_dist_unit)
            self._edge_info[e_id]["max_speed"] = sum([self._lane_info[l_id]["max_speed"] for l_id in lane_ids]) / len(lane_ids)

        all_route_ids, self._all_routes = list(traci.route.getIDList()), {}
        for route_id in all_route_ids:
            self._all_routes[route_id] = traci.route.getEdges(route_id)

        self._get_individual_vehicle_data = get_individual_vehicle_data
        self._automatic_subscriptions = automatic_subscriptions

        if self._verbose:
            self._suppress_warnings = suppress_warnings
            self._suppress_pbar = suppress_pbar
        else:
            self._suppress_warnings = True
            self._suppress_pbar = True

        self._sim_start_time = get_time_str()

        if not self._gui:
            if self.scenario_name == None: _name = "Simulation"
            else: _name = "'{0}' Scenario".format(self.scenario_name)

            if self._verbose:
                print(f"Running {_name}")
                print(f"  - TUD-SUMO version: {self._tuds_version}")
                print(f"  - Start time: {self._sim_start_time}")

    def save_objects(self, filename: str|None = None, overwrite: bool = True, json_indent: int|None = 4) -> None:
        """
        Save parameters of all TUD-SUMO objects created for the simulation (tracked edges/junctions, phases, controllers, events, demand, routes).

        Args:
            `filename` (str, optional):Output filename ('_.json_' or '_.pkl_')
            `overwrite` (bool): Denotes whether to allow overwriting previous outputs
            `json_indent` (int, optional):Indent used when saving JSON files
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

    def load_objects(self, object_parameters: str|dict) -> None:
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

    def load_demand_profiles(self, demand_profiles: str|dict|list|tuple) -> list:
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
                    dp_dict = pickle.load(fp)

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

    def _add_demand_vehicles(self) -> None:
        """ Implements demand in the demand table. """

        if self._manual_flow and len(self._demand_profiles) > 0:

            for demand_profile in self._demand_profiles.values():
                
                if not demand_profile.active: continue
                
                for demand_arr in demand_profile._demand_arrs:
                    step_range = demand_arr[1]
                    if self.curr_step < step_range[0]: continue
                    elif self.curr_step > step_range[1]: continue

                    routing = demand_arr[0]
                    veh_per_step = (demand_arr[2] / 3600) * self.step_length
                    vehicle_types = demand_arr[3]
                    vehicle_type_dists = demand_arr[4]
                    initial_speed = demand_arr[5]
                    origin_lane = demand_arr[6]
                    origin_pos = demand_arr[7]
                    insertion_sd = demand_arr[8]
                    colour = demand_arr[9]
                    
                    added = 0
                    if veh_per_step <= 0: continue
                    elif veh_per_step < 1:
                        # If the number of vehicles to create per step is less than 0,
                        # use veh_per_step as a probability to add a new vehicle
                        n_vehicles = 1 if random() < veh_per_step else 0
                    else:
                        # Calculate the number of vehicles per step to add using a
                        # normal distribution with veh_per_step and the insertion_sd (standard deviation)
                        if insertion_sd > 0: n_vehicles = round(np.random.normal(veh_per_step, veh_per_step * insertion_sd, 1)[0])
                        else: n_vehicles = veh_per_step

                    n_vehicles = max(0, n_vehicles)
                    vehicle_ids = []

                    while added < n_vehicles:
                        if isinstance(vehicle_types, list):
                            vehicle_type = choices(vehicle_types, vehicle_type_dists, k=1)[0]
                        else: vehicle_type = vehicle_types

                        vehicle_id = "{0}_md_{1}".format(vehicle_type, self._man_flow_id)
                        
                        self.add_vehicle(vehicle_id, vehicle_type, routing, initial_speed, origin_lane, origin_pos)

                        vehicle_ids.append(vehicle_id)
                        added += 1
                        self._man_flow_id += 1

                    if colour != None: self.set_vehicle_vals(vehicle_ids, colour=colour)

        elif not self._suppress_warnings:
            desc = "Cannot add flow manually (no demand profiles)."
            raise_warning(desc, self.curr_step)

    def add_vehicle_type(self, vehicle_type_id: str, vehicle_class: str="passenger", colour: str|list|tuple|None = None, length: int|float|None = None, width: int|float|None = None, height: int|float|None = None, max_speed: int|float|None = None, speed_factor: int|float|None = None, speed_dev: int|float|None = None, min_gap: int|float|None = None, acceleration: int|float|None = None, deceleration: int|float|None = None, tau: int|float|None = None, max_lateral_speed: int|float|None = None, emission_class: str|None = None, gui_shape: str|None = None) -> None:
        """
        Adds a new vehicle type to the simulation.

        Args:
            `vehicle_type_id` (str): ID for the new vehicle type
            `vehicle_class` (str, optional): Vehicle class (defaults to passenger)
            `colour` (str, list, tuple, optional): Vehicle colour, either hex code, list of rgb/rgba values or valid SUMO colour string
            `length` (int, float, optional): Vehicle length in metres/feet
            `width` (int, float, optional): Vehicle width in metres/feet
            `height` (int, float, optional): Vehicle height in metres/feet
            `max_speed` (int, float, optional): Vehicle max speed in km/h or mph
            `speed_factor` (int, float, optional): Vehicle speed multiplier
            `speed_dev` (int, float, optional): Vehicle deviation from speed factor
            `min_gap` (int, float, optional): Minimum gap behind leader
            `acceleration` (int, float, optional): Maximum vehicle acceleration
            `deceleration` (int, float, optional): Maximum vehicle deceleration
            `tau` (int, float, optional): Vehicle car following parameter
            `max_lateral_speed` (int, float, optional): Maximum lateral speed when lane changing
            `emission_class` (str, optional): Vehicle emissions class ID
            `gui_shape` (str, optional): Vehicle shape in GUI (defaults to vehicle class name)
        """
        
        if self.vehicle_type_exists(vehicle_type_id):
            desc = f"Cannot create vehicle type (ID '{vehicle_type_id}' already exists)."
            raise_error(KeyError, desc, self.curr_step)

        traci.vehicletype.copy('DEFAULT_VEHTYPE', vehicle_type_id)
        self._added_vehicle_types.add(vehicle_type_id)

        if self._gui: gui_shape = vehicle_class

        self.set_vehicle_type_vals(vehicle_type_id, vehicle_class=vehicle_class, colour=colour,
                                   length=length, width=width, height=height, max_speed=max_speed,
                                   speed_factor=speed_factor, speed_dev=speed_dev, min_gap=min_gap,
                                   acceleration=acceleration, deceleration=deceleration, tau=tau,
                                   max_lateral_speed=max_lateral_speed, emission_class=emission_class,
                                   gui_shape=gui_shape)

    def add_tracked_junctions(self, junctions: str|list|tuple|dict|None = None) -> dict:
        """
        Initalise junctions and start tracking states and flows. Defaults to all junctions with traffic lights.
        
        Args:
            `junctions` (str, list, tuple, dict, optional):Junction IDs or list of IDs, or dict containing junction(s) parameters
        
        Returns:
            dict|TrackedJunction: Dictionary of added junctions or single TrackedJunction  object
        """

        self.track_juncs = True
        added_junctions = {}

        # If none given, track all junctions with traffic lights
        if junctions == None: 
            track_list, junc_params = self._all_tls, None
        else:
            
            junctions = validate_type(junctions, (str, list, tuple, dict), "junctions", self.curr_step)
            if isinstance(junctions, dict):
                junc_ids, junc_params = list(junctions.keys()), junctions
            elif isinstance(junctions, (list, tuple)):
                junc_ids, junc_params = junctions, None
            elif isinstance(junctions, str):
                junc_ids, junc_params = [junctions], None

            if len(set(self._all_juncs).intersection(set(junc_ids))) != len(junc_ids):
                desc = "Junction ID(s) not found ('{0}').".format("', '".join(set(junc_ids) - set(self._all_juncs)))
                raise_error(KeyError, desc, self.curr_step)
            else: track_list = junc_ids

        for junc_id in track_list:
            if junc_id not in self.tracked_junctions:
                junc_param = junc_params[junc_id] if junc_params != None else None
                self.tracked_junctions[junc_id] = TrackedJunction(junc_id, self, junc_param)
                self.tracked_junctions[junc_id].update_vals = True
                added_junctions[junc_id] = self.tracked_junctions[junc_id]
            else:
                desc = "Junction with ID '{0}' already exists.".format(junc_id)
                raise_error(ValueError, desc, self.curr_step)

        if len(added_junctions) == 1: return added_junctions[list(added_junctions.keys())[0]]
        else: return added_junctions

    def reset_data(self, reset_juncs: bool = True, reset_edges: bool = True, reset_controllers: bool = True, reset_trips: bool = True) -> None:
        """
        Resets object/simulation data collection.
        
        Args:
            `reset_juncs` (bool): Reset tracked junction data
            `reset_edges` (bool): Reset tracked edge data
            `reset_controllers` (bool): Reset controller data
            `reset_trips` (bool): Reset complete/incomplete trip data
        """

        if reset_juncs:
            for junction in self.tracked_junctions.values():
                junction.reset()

        if reset_edges:
            for edge in self.tracked_edges.values():
                edge.reset()

        if reset_controllers:
            for controller in self.controllers.values():
                controller.reset()

        if reset_trips:
            self._trips = {"incomplete": {}, "completed": {}}

        self._sim_start_time = get_time_str()
        self._all_data = None

    def is_running(self, close: bool = True) -> bool:
        """
        Returns whether the simulation is running.
        
        Args:
            `close` (bool): If `True`, end Simulation
        
        Returns:
            bool: Denotes if the simulation is running
        """

        if not self._running: return self._running

        if traci.simulation.getMinExpectedNumber() == 0:

            if len(self._demand_profiles) != 0:
                if False not in [dp.is_complete() for dp in self._demand_profiles.values()]: return True
            if close: self.end()
            if not self._suppress_warnings:
                raise_warning("Ended simulation early (no vehicles remaining).", self.curr_step)
            return False
        
        return True

    def end(self) -> None:
        """ Ends the simulation. """

        if self._recorder != None:
            for recording in self._recorder.get_recordings():
                self.save_recording(recording)

        try:
            traci.close()
        except traci.exceptions.FatalTraCIError:
            pass
        self._running = False

    def save_data(self, filename: str|None = None, overwrite: bool = True, json_indent: int|None = 4) -> None:
        """
        Save all vehicle, detector and junction data in a JSON or pickle file.
        
        Args:
            `filename` (str, optional):Output filepath (defaults to '_./{scenario_name}.json_')
            `overwrite` (bool): Prevent previous outputs being overwritten
            `json_indent` (int, optional):Indent used when saving JSON files
        """

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
            filename += ".json"
            w_class, w_mode = json, "w"

        if os.path.exists(filename) and overwrite:
            if not self._suppress_warnings: raise_warning("File '{0}' already exists and will be overwritten.".format(filename), self.curr_step)
        elif os.path.exists(filename) and not overwrite:
            desc = "File '{0}' already exists and cannot be overwritten.".format(filename)
            raise_error(FileExistsError, desc, self.curr_step)

        if self._all_data != None:
            if self._scheduler != None: self._all_data["data"]["events"] = self._scheduler.__dict__()
            with open(filename, w_mode) as fp:
                if w_mode == "wb": w_class.dump(self._all_data, fp)
                else: w_class.dump(self._all_data, fp, indent=json_indent)
        else:
            desc = "No data to save as a simulation has not been run."
            raise_error(SimulationError, desc, self.curr_step)

    def add_tracked_edges(self, edge_ids: str|list|None = None):
        """
        Initalise edges and start collecting data.
        
        Args:
            `edge_ids` (str, list, optional):List of edge IDs or single ID, defaults to all
        """

        if edge_ids == None: edge_ids = self._all_edges
        if not isinstance(edge_ids, list): edge_ids = [edge_ids]
        edge_ids = validate_list_types(edge_ids, str, param_name="edge_ids", curr_sim_step=self.curr_step)

        for edge_id in edge_ids:
            if self.geometry_exists(edge_id) == None:
                desc = "Geometry ID '{0}' not found.".format(edge_id)
                raise_error(KeyError, desc, self.curr_step)
            elif edge_id in self.tracked_edges:
                desc = "Tracked geometry with ID '{0}' already exists.".format(edge_id)
                raise_error(ValueError, desc, self.curr_step)
            self.tracked_edges[edge_id] = TrackedEdge(edge_id, self)

    def add_events(self, event_params: Event|str|list|dict) -> None:
        """
        Add events and event scheduler.
        
        Args:
            `event_parms` (Event, str, list, dict): Event parameters [Event|[Event]|dict|filepath]
        """

        if self._scheduler == None:
            self._scheduler = EventScheduler(self)
        self._scheduler.add_events(event_params)

    def add_controllers(self, controller_params: str|dict) -> dict:
        """
        Add controllers from parameters in a dictionary/JSON file.
        
        Args:
            `controller_params` (str, dict): Controller parameters dictionary or filepath

        Returns:
            dict: Dictionary of controller objects by their ID
        """

        controller_params = load_params(controller_params, "controller_params", self.curr_step)

        for c_id, c_params  in controller_params.items():
            if c_id in self.controllers:
                desc = "Controller with ID '{0}' already exists.".format(c_id)
                raise_error(ValueError, desc, self.curr_step)
            if isinstance(c_params, (RGController, VSLController)):
                self.controllers[c_id] = c_params
            elif isinstance(c_params, dict):
                if 'type' not in c_params.keys():
                    desc = "No type given (must be [1 (RG)|2 (VSL)])."
                    raise_error(KeyError, desc, self.curr_step)
                if c_params['type'] in [1, 2, "VSL", "RG"]:
                    
                    if c_params['type'] in [1, "VSL"]: controller = VSLController(c_id, c_params, self)
                    elif c_params['type'] in [2, "RG"]: controller = RGController(c_id, c_params, self)
                    
                    self.controllers[c_id] = controller

                else:
                    desc = "Invalid controller type (must be [1 (RG)|2 (VSL)])."
                    raise_error(ValueError, desc, self.curr_step)
            else:
                desc = "Invalid parameters type in dictionary (must be [dict|RGController|VSLController], not '{0}').".format(type(c_params).__name__)
                raise_error(TypeError, desc, self.curr_step)

        return self.controllers

    def step_through(self, n_steps: int|None = None, end_step: int|None = None, n_seconds: int|None = None, vehicle_types: list|tuple|None = None, keep_data: bool = True, append_data: bool = True, pbar_max_steps: int|None = None) -> dict:
        """
        Step through simulation from the current time until end_step, aggregating data during this period.
        
        Args:
            `n_steps` (int, optional):Perform n steps of the simulation (defaults to 1)
            `end_step` (int, optional):End point for stepping through simulation (given instead of `n_steps` or `n_seconds`)
            `n_seconds` (int, optional):Simulation duration in seconds (given instead of `n_steps` or `end_step`)
            `vehicle_types` (list, tuple, optional):Vehicle type(s) to collect data of (type ID or list of IDs, defaults to all)
            `keep_data` (bool): Denotes whether to store and process data collected during this run (defaults to `True`)
            `append_data` (bool): Denotes whether to append simulation data to that of previous runs or overwrite previous data (defaults to `True`)
            `pbar_max_steps` (int, optional):Max value for progress bar (persistent across calls) (negative values remove the progress bar)
        
        Returns:
            dict: All data collected through the time period, separated by detector
        """

        if not self.is_running(): return

        if append_data == True: prev_data = self._all_data
        else: prev_data = None

        detector_list = list(self.available_detectors.keys())
        start_time = self.curr_step

        # Can only be given 1 (or none) of n_steps, end_steps and n_seconds.
        # If none given, defaults to simulating 1 step.
        n_params = 3 - [n_steps, end_step, n_seconds].count(None)
        if n_params == 0:
            n_steps = 1
        elif n_params != 1:
            strs = ["{0}={1}".format(param, val) for param, val in zip(["n_steps", "end_step", "n_seconds"], [n_steps, end_step, n_seconds]) if val != None]
            desc = "More than 1 time value given ({0}).".format(", ".join(strs))
            raise_error(ValueError, desc, self.curr_step)
        
        # All converted to end_step
        if n_steps != None:
            end_step = self.curr_step + n_steps
        elif n_seconds != None:
            end_step = self.curr_step + int(n_seconds / self.step_length)

        n_steps = end_step - self.curr_step

        if keep_data:
            # Create a new blank sim_data dictionary here
            if prev_data == None:
                all_data = {"scenario_name": "", "scenario_desc": "", "tuds_version": self._tuds_version, "data": {}, "start": start_time, "end": self.curr_step, "step_len": self.step_length, "units": self.units.name, "seed": self._seed, "sim_start": self._sim_start_time, "sim_end": get_time_str()}
                
                if self.scenario_name == None: del all_data["scenario_name"]
                else: all_data["scenario_name"] = self.scenario_name

                if self.scenario_desc == None: del all_data["scenario_desc"]
                else: all_data["scenario_desc"] = self.scenario_desc

                if len(self.available_detectors) > 0: all_data["data"]["detectors"] = {}
                if self.track_juncs: all_data["data"]["junctions"] = {}
                if len(self.tracked_edges) > 0: all_data["data"]["edges"] = {}
                if len(self.controllers) > 0: all_data["data"]["controllers"] = {}
                all_data["data"]["vehicles"] = {"no_vehicles": [], "no_waiting": [], "tts": [], "twt": [], "delay": [], "to_depart": []}
                if len(self._demand_profiles) > 0:
                    all_data["data"]["demand"] = {"headers": list(self._demand_profiles.values())[0]._demand_headers, "profiles": [dp._demand_arrs for dp in self._demand_profiles.values()]}
                all_data["data"]["trips"] = {"incomplete": {}, "completed": {}}
                if self._get_individual_vehicle_data: all_data["data"]["all_vehicles"] = []
                if self._scheduler != None: all_data["data"]["events"] = {}
            
            else: all_data = prev_data

        create_pbar = False
        if not self._suppress_pbar:
            if self._gui: create_pbar = False
            elif pbar_max_steps != None:

                # A new progress bars are created if there is a change in
                # the value of pbar_max_steps (used to maintain the same progress
                # bar through multiple calls of step_through())
                if isinstance(pbar_max_steps, (int, float)):

                    # The current progress bar is removed if pbar_max_steps < 0
                    if pbar_max_steps < 0:
                        create_pbar = False
                        self._pbar, self._pbar_length = None, None
                    elif pbar_max_steps != self._pbar_length:
                        create_pbar, self._pbar_length = True, pbar_max_steps
                        
                else:
                    desc = "Invalid pbar_max_steps '{0}' (must be 'int', not '{1}').".format(pbar_max_steps, type(pbar_max_steps).__name__)
                    raise_error(TypeError, desc, self.curr_step)
            
            elif not isinstance(self._pbar, tqdm) and not self._gui and n_steps >= 10:
                create_pbar, self._pbar_length = True, n_steps

        if create_pbar:
            self._pbar = tqdm(desc="Simulating ({0} - {1} vehs)".format(get_sim_time_str(self.curr_step, self.step_length), len(self._all_curr_vehicle_ids)),
                              total=self._pbar_length, unit="steps", colour='CYAN')

        while self.curr_step < end_step:

            last_step_data, all_v_data = self._step(vehicle_types, keep_data)

            if keep_data:
                if self._get_individual_vehicle_data: all_data["data"]["all_vehicles"].append(all_v_data)

                # Append all detector data to the sim_data dictionary
                if "detectors" in all_data["data"]:
                    if len(all_data["data"]["detectors"]) == 0:
                        for detector_id in detector_list:
                            all_data["data"]["detectors"][detector_id] = self.available_detectors[detector_id]
                            all_data["data"]["detectors"][detector_id].update({"speeds": [], "vehicle_counts": [], "vehicle_ids": []})
                            if self.available_detectors[detector_id]["type"] == "inductionloop":
                                all_data["data"]["detectors"][detector_id]["occupancies"] = []

                    for detector_id in last_step_data["detectors"].keys():
                        if detector_id not in all_data["data"]["detectors"].keys():
                            desc = "Unrecognised detector ID found ('{0}').".format(detector_id)
                            raise_error(KeyError, desc, self.curr_step)
                        for data_key, data_val in last_step_data["detectors"][detector_id].items():
                            all_data["data"]["detectors"][detector_id][data_key].append(data_val)
                
                for data_key, data_val in last_step_data["vehicles"].items():
                    all_data["data"]["vehicles"][data_key].append(data_val)

                all_data["data"]["trips"] = self._trips

            # Stop updating progress bar if reached total (even if simulation is continuing)
            if isinstance(self._pbar, tqdm):
                self._pbar.update(1)
                self._pbar.set_description("Simulating ({0} - {1} vehs)".format(get_sim_time_str(self.curr_step, self.step_length), len(self._all_curr_vehicle_ids)))
                if self._pbar.n == self._pbar.total:
                    self._pbar, self._pbar_length = None, None

        if keep_data:

            all_data["end"] = self.curr_step
            all_data["sim_end"] = get_time_str()

            # Get all object data and update in sim_data
            if self.track_juncs: all_data["data"]["junctions"] = last_step_data["junctions"]
            if self._scheduler != None: all_data["data"]["events"] = self._scheduler.__dict__()
            for e_id, edge in self.tracked_edges.items(): all_data["data"]["edges"][e_id] = edge.__dict__()
            for c_id, controller in self.controllers.items(): all_data["data"]["controllers"][c_id] = controller.__dict__()

            self._all_data = all_data
            return all_data
        
        else: return None
            
    def _step(self, vehicle_types: list|None = None, keep_data: bool = True) -> dict:
        """
        Increment simulation by one time step, updating light state. Use `Simulation.step_through()` to run the simulation.
        
        Args:
            `vehicle_types` (list, optional):Vehicle type(s) to collect data of (type ID or list of IDs, defaults to all)
            `keep_data` (bool): Denotes whether to store and process data collected during this run (defaults to `True`)

        Returns:
            dict: Simulation data
        """

        self.curr_step += 1
        self.curr_time += self.step_length

        # First, implement the demand in the demand table (if exists)
        if self._manual_flow:
            self._add_demand_vehicles()

        if self._recorder != None:
            recording_ids = self._recorder.get_recordings()
            for recording_id in recording_ids:
                recording_data = self._recorder.get_recording_data(recording_id)
                
                frame_file = f"{recording_data['frames_loc']}/f_{len(recording_data['frame_files'])+1}.png"
                self.take_screenshot(filename=frame_file, view_id=recording_data["view_id"], bounds=recording_data["bounds"], zoom=recording_data["zoom"])
                recording_data["frame_files"].append(frame_file)

        # Step through simulation
        traci.simulationStep()

        if self._recorder != None:
            recording_ids = self._recorder.get_recordings()
            for recording_id in recording_ids:
                recording_data = self._recorder.get_recording_data(recording_id)
                if "vehicle_id" in recording_data and recording_data["vehicle_id"] not in self._all_curr_vehicle_ids:
                    self._recorder.save_recording(recording_id)
                    continue

        # Update all vehicle ID lists
        all_prev_vehicle_ids = self._all_curr_vehicle_ids
        self._all_curr_vehicle_ids = set(traci.vehicle.getIDList())
        self._all_loaded_vehicle_ids = set(traci.vehicle.getLoadedIDList())
        self._all_to_depart_vehicle_ids = self._all_loaded_vehicle_ids - self._all_curr_vehicle_ids

        all_vehicles_in = self._all_curr_vehicle_ids - all_prev_vehicle_ids
        all_vehicles_out = all_prev_vehicle_ids - self._all_curr_vehicle_ids

        # Add all automatic subscriptions
        if self._automatic_subscriptions:
            subscriptions = ["speed", "lane_id", "allowed_speed", "lane_idx"]

            # Subscribing to 'altitude' uses POSITION3D, which includes the vehicle x,y coordinates.
            # If not collecting all individual vehicle data, we can instead subcribe to the 2D position.
            if self._get_individual_vehicle_data: subscriptions += ["acceleration", "heading", "altitude"]
            else: subscriptions.append("position")

            self.add_vehicle_subscriptions(list(all_vehicles_in), subscriptions)

        # Process all vehicles entering/exiting the simulation
        self._vehicles_in(all_vehicles_in)
        self._vehicles_out(all_vehicles_out)

        # Update all traffic signal phases
        if self._junc_phases != None:
            update_junc_lights = []
            for junction_id, phases in self._junc_phases.items():
                phases["curr_time"] += self.step_length
                if phases["curr_time"] >= phases["cycle_len"]:
                    phases["curr_time"] -= phases["cycle_len"]
                    phases["curr_phase"] = 0
                    update_junc_lights.append(junction_id)

                elif phases["curr_time"] >= sum(phases["times"][:phases["curr_phase"] + 1]):
                    phases["curr_phase"] += 1
                    update_junc_lights.append(junction_id)

            self._update_lights(update_junc_lights)

        # Update all edge & junction data for the current step and implement any actions
        # for controllers and event schedulers.
        for controller in self.controllers.values(): controller.update(keep_data)
        for edge in self.tracked_edges.values(): edge.update(keep_data)
        for junc in self.tracked_junctions.values(): junc.update(keep_data)
        if self._scheduler != None: self._scheduler.update_events()

        if keep_data:
            data = {"detectors": {}, "vehicles": {}}
            if self.track_juncs: data["junctions"] = {}
            
            # Collect all detector data for the step
            detector_list = list(self.available_detectors.keys())
            for detector_id in detector_list:
                data["detectors"][detector_id] = {}
                if detector_id not in self.available_detectors.keys():
                    desc = "Unrecognised detector ID found ('{0}').".format(detector_id)
                    raise_error(KeyError, desc, self.curr_step)
                if self.available_detectors[detector_id]["type"] == "multientryexit":

                    detector_data = self.get_detector_vals(detector_id, ["lsm_speed", "vehicle_count"])
                    data["detectors"][detector_id]["speeds"] = detector_data["lsm_speed"]
                    data["detectors"][detector_id]["vehicle_counts"] = detector_data["vehicle_count"]
                    
                elif self.available_detectors[detector_id]["type"] == "inductionloop":

                    detector_data = self.get_detector_vals(detector_id, ["lsm_speed", "vehicle_count", "lsm_occupancy"])
                    data["detectors"][detector_id]["speeds"] = detector_data["lsm_speed"]
                    data["detectors"][detector_id]["vehicle_counts"] = detector_data["vehicle_count"]
                    data["detectors"][detector_id]["occupancies"] = detector_data["lsm_occupancy"]

                else:
                    if not self._suppress_warnings: raise_warning("Unknown detector type '{0}'.".format(self.available_detectors[detector_id]["type"]), self.curr_step)

                data["detectors"][detector_id]["vehicle_ids"] = self.get_last_step_detector_vehicles(detector_id)

            total_data, all_v_data = self._get_all_vehicle_data(vehicle_types=vehicle_types)
            data["vehicles"]["no_vehicles"] = total_data["no_vehicles"]
            data["vehicles"]["no_waiting"] = total_data["no_waiting"]
            data["vehicles"]["tts"] = total_data["no_vehicles"] * self.step_length
            data["vehicles"]["twt"] = total_data["no_waiting"] * self.step_length
            data["vehicles"]["delay"] = total_data["delay"]
            data["vehicles"]["to_depart"] = total_data["to_depart"]

            if self.track_juncs:
                for junc_id, junc in self.tracked_junctions.items():
                    data["junctions"][junc_id] = junc.__dict__()

            return data, all_v_data
        
        else:
            self.reset_data(reset_trips=False)
            return None, None
    
    def get_no_vehicles(self) -> int:
        """
        Returns the number of vehicles in the simulation during the last simulation step.
        
        Returns:
            int: No. of vehicles
        """
        
        if self._all_data == None:
            desc = "No data to return (simulation likely has not been run, or data has been reset)."
            raise_error(SimulationError, desc, self.curr_step)
        return self._all_data["data"]["vehicles"]["no_vehicles"][-1]
    
    def get_no_waiting(self) -> float:
        """
        Returns the number of waiting vehicles in the simulation during the last simulation step.
        
        Returns:
            float: No. of waiting vehicles
        """
        
        if self._all_data == None:
            desc = "No data to return (simulation likely has not been run, or data has been reset)."
            raise_error(SimulationError, desc, self.curr_step)
        return self._all_data["data"]["vehicles"]["no_waiting"][-1]
    
    def get_tts(self) -> int:
        """
        Returns the total time spent by vehicles in the simulation during the last simulation step.
        
        Returns:
            float: Total Time Spent (TTS) in seconds
        """
        
        if self._all_data == None:
            desc = "No data to return (simulation likely has not been run, or data has been reset)."
            raise_error(SimulationError, desc, self.curr_step)
        return self._all_data["data"]["vehicles"]["tts"][-1]
    
    def get_twt(self) -> int:
        """
        Returns the total waiting time of vehicles in the simulation during the last simulation step.
        
        Returns:
            float: Total Waiting Time (TWT) in seconds
        """
        
        if self._all_data == None:
            desc = "No data to return (simulation likely has not been run, or data has been reset)."
            raise_error(SimulationError, desc, self.curr_step)
        return self._all_data["data"]["vehicles"]["twt"][-1]
    
    def get_delay(self) -> int:
        """
        Returns the total delay of vehicles during the last simulation step.
        
        Returns:
            float: Total delay in seconds
        """
        
        if self._all_data == None:
            desc = "No data to return (simulation likely has not been run, or data has been reset)."
            raise_error(SimulationError, desc, self.curr_step)
        return self._all_data["data"]["vehicles"]["delay"][-1]
    
    def get_to_depart(self) -> int:
        """
        Returns the total number of vehicles waiting to enter the simulation during the last simulation step.
        
        Returns:
            float: No. of vehicles waiting to depart
        """
        
        if self._all_data == None:
            desc = "No data to return (simulation likely has not been run, or data has been reset)."
            raise_error(SimulationError, desc, self.curr_step)
        return self._all_data["data"]["vehicles"]["to_depart"][-1]
    
    def get_interval_network_data(self, data_keys: list|tuple, n_steps: int, interval_end: int = 0, get_avg: bool=False) -> float|dict:
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
    
    def _add_v_func(self, functions, parameters: dict, func_arr: list, valid_sim_params: list) -> None:
        """
        Add a vehicle in/out function.
        
        Args:
            `functions` (function, list):  Function or list of functions
            `parameters` (dict): Dictionary containing values for extra custom parameters
            `func_arr` (list):   Either list of _v_in_funcs or _v_out_funcs
            `valid_sim_params` (list): List of valid simulation parameters
        """

        if not isinstance(functions, list):
            if parameters != None: parameters = {functions.__name__: parameters}
            functions = [functions]

        for function in functions:
            func_params_arr = list(function.__code__.co_varnames)
            self._v_func_params[function.__name__] = {}
            func_arr.append(function)

            if parameters != None and function.__name__ in parameters:
                if len(set(parameters[function.__name__].keys()) - set(func_params_arr)) != 0:
                    desc = "Unknown function parameters (['{0}'] are not parameter(s) of '{1}()').".format("','".join(list(set(parameters[function.__name__].keys()) - set(func_params_arr))), function.__name__)
                    raise_error(KeyError, desc, self.curr_step)

            # Parameters for each function are either:
            #  - Valid simulation parameter (such as vehicle_id), which is set with each vehicle
            #  - In the extra parameters dictionary, so the value is set here
            #  - Not given, where it is assumed this has a default value already defined (error thrown if not the case)
            for func_param in func_params_arr:
                if func_param in valid_sim_params:
                    self._v_func_params[function.__name__][func_param] = None
                elif parameters != None and function.__name__ in parameters and func_param in parameters[function.__name__]:
                    self._v_func_params[function.__name__][func_param] = parameters[function.__name__][func_param]

    def _remove_v_func(self, functions, v_func_type: str) -> None:
        """
        Remove a vehicle in/out function.
        
        Args:
            `functions` (function, list): Function (or function name) or list of functions
            `v_func_type` (str): Either 'in' or 'out'
        """
        
        if not isinstance(functions, list): functions = [functions]
        rm_func_names = [func.__name__ if not isinstance(func, str) else func for func in functions]

        for func_name in rm_func_names:
            if func_name in self._v_func_params:
                del self._v_func_params[func_name]
            else:
                desc = "Function '{0}()' not found.".format(func_name)
                raise_error(KeyError, desc, self.curr_step)

        func_arr = self._v_in_funcs if v_func_type.upper() == "IN" else self._v_out_funcs
        new_v_funcs = []
        for func in func_arr:
            if func.__name__ not in rm_func_names:
                new_v_funcs.append(func)

        if v_func_type.upper() == "IN": self._v_in_funcs = new_v_funcs
        elif v_func_type.upper() == "OUT": self._v_out_funcs = new_v_funcs
    
    def add_vehicle_in_functions(self, functions, parameters: dict|None = None) -> None:
        """
        Add a function (or list of functions) that will are called with each new vehicle that enters the simulation.
        Valid TUD-SUMO parameters are '_simulation_', '_curr_step_', '_vehicle_id_', '_route_id_', '_vehicle_type_',
        '_departure_', '_origin_', '_destination_'. These values are collected from the simulation with each call. Extra
        parameters for any function can be given in the parameters dictionary.
        
        Args:
            `functions` (function, list): Function or list of functions
            `parameters` (dict): Dictionary containing values for extra custom parameters
        """

        self._add_v_func(functions, parameters, self._v_in_funcs, self._valid_v_func_params)

    def remove_vehicle_in_functions(self, functions) -> None:
        """
        Stop a function called with each new vehicle that enters the simulation.
        
        Args:
            functions (function, list): Function (or function name) or list of functions
        """
        
        self._remove_v_func(functions, "in")

    def add_vehicle_out_functions(self, functions, parameters: dict|None = None) -> None:
        """
        Add a function (or list of functions) that will are called with each vehicle that exits the simulation.
        Valid TUD-SUMO parameters are '_simulation_', '_curr_step_' and '_vehicle_id_'. These values are collected
        from the simulation with each call. Extra parameters for any function can be given in the parameters dictionary.
        
        Args:
            `functions` (function, list): Function or list of functions
            `parameters` (dict, optional):Dictionary containing values for extra custom parameters
        """

        self._add_v_func(functions, parameters, self._v_out_funcs, self._valid_v_func_params[:3])

    def remove_vehicle_out_functions(self, functions) -> None:
        """
        Stop a function called with each vehicle that exits the simulation.
        
        Args:
            `functions` (function, list): Function (or function name) or list of functions
        """
        
        self._remove_v_func(functions, "out")

    def update_vehicle_function_parameters(self, parameters: dict) -> None:
        """
        Update parameters for previously added vehicle in/out functions.
        
        Args:
            `parameters` (dict): Dictionary containing new parameters for vehicle in/out functions.
        """
        
        validate_type(parameters, dict, "parameters", self.curr_step)

        for func_name, params in parameters.items():
            if func_name in self._v_func_params:
                
                if isinstance(params, dict):
                    self._v_func_params[func_name].update(params)

                else:
                    desc = "Invalid '{0}' function parameters type (must be 'dict', not '{1}').".format(func_name, type(params).__name__)
                    raise_error(TypeError, desc, self.curr_step)
            
            else:
                desc = "Vehicle function '{0}' not found.".format(func_name)
                raise_error(KeyError, desc, self.curr_step)

    def _vehicles_in(self, vehicle_ids: str|list|tuple, is_added: bool = False) -> None:
        """
        Updates simulation with each new vehicle.

        Args:
            `vehicle_ids` (str, list, tuple): Vehicle ID or list of IDs
            `is_added` (bool): Denotes whether the vehicle(s) has been manually added
        """

        if isinstance(vehicle_ids, str): vehicle_ids = [vehicle_ids]
        elif isinstance(vehicle_ids, set): vehicle_ids = list(vehicle_ids)
        vehicle_ids = validate_list_types(vehicle_ids, str, param_name="vehicle_ids", curr_sim_step=self.curr_step)

        for vehicle_id in vehicle_ids:

            if vehicle_id not in self._all_curr_vehicle_ids: self._all_curr_vehicle_ids.add(vehicle_id)
            if vehicle_id not in self._all_loaded_vehicle_ids: self._all_loaded_vehicle_ids.add(vehicle_id)
            if vehicle_id not in self._all_added_vehicles and is_added: self._all_added_vehicles.add(vehicle_id)

            # Create a new incomplete trip in the trip data
            veh_data = self.get_vehicle_vals(vehicle_id, ("type", "route_id", "route_edges"))
            veh_type, route_id, origin, destination = veh_data["type"], veh_data["route_id"], veh_data["route_edges"][0], veh_data["route_edges"][-1]
            self._trips["incomplete"][vehicle_id] = {"route_id": route_id,
                                                     "vehicle_type": veh_type,
                                                     "departure": self.curr_step,
                                                     "origin": origin,
                                                     "destination": destination}
            
            # Call vehicle in functions
            for func in self._v_in_funcs:
                param_dict, trip_data = {}, self._trips["incomplete"][vehicle_id]
                for param, val in self._v_func_params[func.__name__].items():
                    if param in trip_data: param_dict[param] = trip_data[param]
                    elif param == "simulation": param_dict[param] = self
                    elif param == "vehicle_id": param_dict[param] = vehicle_id
                    elif param == "curr_step": param_dict[param] = self.curr_step
                    else: param_dict[param] = val
                func(**param_dict)
            
    def _vehicles_out(self, vehicle_ids: str|list|tuple, is_removed: bool = False) -> None:
        """
        Updates simulation with each leaving vehicle.

        Args:
            `vehicle_ids` (str, list, tuple): Vehicle IDs or list of IDs
            `is_removed` (bool): Denotes whether the vehicle(s) has been manually removed
        """

        if isinstance(vehicle_ids, str): vehicle_ids = [vehicle_ids]
        elif isinstance(vehicle_ids, set): vehicle_ids = list(vehicle_ids)
        vehicle_ids = validate_list_types(vehicle_ids, str, param_name="vehicle_ids", curr_sim_step=self.curr_step)

        for vehicle_id in vehicle_ids:

            if self.vehicle_loaded(vehicle_id) and self.vehicle_to_depart(vehicle_id): continue
            
            if is_removed or vehicle_id in self._all_removed_vehicles:
                if vehicle_id in self._trips["incomplete"].keys():
                    self._trips["incomplete"][vehicle_id]["removal"] = self.curr_step
                if vehicle_id not in self._all_removed_vehicles:
                    self._all_removed_vehicles.add(vehicle_id)
                if vehicle_id in self._all_curr_vehicle_ids:
                    self._all_curr_vehicle_ids.remove(vehicle_id)
                if vehicle_id in self._all_loaded_vehicle_ids:
                    self._all_loaded_vehicle_ids.remove(vehicle_id)
                continue

            if vehicle_id in self._trips["incomplete"].keys():
                trip_data = self._trips["incomplete"][vehicle_id]
                veh_type, route_id, departure, origin, destination = trip_data["vehicle_type"], trip_data["route_id"], trip_data["departure"], trip_data["origin"], trip_data["destination"]
                del self._trips["incomplete"][vehicle_id]
                self._trips["completed"][vehicle_id] = {"route_id": route_id, "vehicle_type": veh_type, "departure": departure, "arrival": self.curr_step, "origin": origin, "destination": destination}
            
            else:
                if vehicle_id in self._known_vehicles.keys():
                    trip_data = {}
                    if "departure" in self._known_vehicles[vehicle_id].keys():
                        trip_data["departure"] = self._known_vehicles[vehicle_id]["departure"]
                    if "arrival" in self._known_vehicles[vehicle_id].keys():
                        trip_data["arrival"] = self._known_vehicles[vehicle_id]["arrival"]
                    if "origin" in self._known_vehicles[vehicle_id].keys():
                        trip_data["origin"] = self._known_vehicles[vehicle_id]["origin"]
                    if "destination" in self._known_vehicles[vehicle_id].keys():
                        trip_data["destination"] = self._known_vehicles[vehicle_id]["destination"]    
                else: 
                    desc = "Unrecognised vehicle ID '{0}' in completed trips.".format(vehicle_id)
                    raise_error(KeyError, desc, self.curr_step)

            for func in self._v_out_funcs:
                param_dict = {}
                for param, val in self._v_func_params[func.__name__].items():
                    if param == "vehicle_id": param_dict[param] = vehicle_id
                    elif param == "simulation": param_dict[param] = self
                    elif param == "curr_step": param_dict[param] = self.curr_step
                    else: param_dict[param] = val
                func(**param_dict)
                
    def get_last_step_detector_vehicles(self, detector_ids: str|list|tuple, vehicle_types: list|None = None, flatten: bool = False) -> dict|list:
        """
        Get the IDs of vehicles that passed over the specified detectors.
        
        Args:
            `detector_ids` (str, list, tuple): Detector ID or list of IDs (defaults to all)
            `vehicle_types` (list, optional):Included vehicle types
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
    
    def get_detector_vals(self, detector_ids: list|tuple|str, data_keys: str|list) -> int|float|dict:
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
    
    def get_interval_detector_data(self, detector_ids: str|list|tuple, data_keys: str|list, n_steps: int, interval_end: int = 0, avg_step_vals: bool = True, avg_det_vals: bool = True, sum_counts: bool = True) -> float|list|dict:
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
            # the matrix on the y axis, from ([n_steps|1] x len(detector_ids)) to ([n_steps|1] x 1)
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

    def set_phases(self, junction_phases: dict, start_phase: int = 0, overwrite: bool = True) -> None:
        """
        Sets the phases for the simulation, starting at the next simulation step.
        
        Args:
            `junction_phases` (dict): Dictionary containing junction phases and times
            `start_phase` (int): Phase number to start at, defaults to 0
            `overwrite` (bool): If `True`, the `junc_phases` dict is overwitten with `junction_phases`. If `False`, only specific junctions are overwritten.
        """

        # If overwriting, the junc phases dictionary is replaced with
        # the new version. Otherwise, only specific junctions are overwritten.
        if overwrite or self._junc_phases == None:
            self._junc_phases = junction_phases
        else:
            for junc_id, new_phases in junction_phases.items():
                self._junc_phases[junc_id] = deepcopy(new_phases)

        for junc_id in junction_phases.keys():

            if junc_id not in list(traci.trafficlight.getIDList()):
                desc = "Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(junc_id)
                raise_error(KeyError, desc, self.curr_step)

            junc_phase = self._junc_phases[junc_id]

            valid_params = {"times": list, "phases": list, "curr_phase": int}
            error, desc = test_input_dict(junc_phase, valid_params, "'{0}' phases".format(junc_id), required=["times", "phases"])
            if error != None: raise_error(error, desc, self.curr_step)

            # Check times and colours match length, are of the right type, and assert all
            # phase times are greater than the simulation step length.
            validate_list_types(junc_phase["times"], (int, float), param_name="'{0}' phase times".format(junc_id), curr_sim_step=self.curr_step)
            validate_list_types(junc_phase["phases"], str, param_name="'{0}' phase colours".format(junc_id), curr_sim_step=self.curr_step)

            if len(junc_phase["times"]) != len(junc_phase["phases"]):
                desc = "'{0}' phase colours and times do not match length ('times' {1} != 'phases' {2}).".format(junc_id, len(junc_phase["times"]), len(junc_phase["phases"]))
                raise_error(ValueError, desc, self.curr_step)

            for t in junc_phase["times"]:
                if t < self.step_length:
                    desc = "Invalid phase duration (phase_dur ({0}) < resolution ({1}))\n.  - {2}".format(t, self.step_length, junc_phase)
                    raise_error(ValueError, desc, self.curr_step)

            if "curr_phase" not in junc_phase.keys(): junc_phase["curr_phase"] = start_phase
            if junc_phase["curr_phase"] > len(junc_phase["phases"]): junc_phase["curr_phase"] -= len(junc_phase["phases"])

            junc_phase["curr_time"] = sum(junc_phase["times"][:junc_phase["curr_phase"]])
            junc_phase["cycle_len"] = sum(junc_phase["times"])

        self._update_lights(list(junction_phases.keys()))

    def set_m_phases(self, junction_phases: dict, start_phase: int = 0, overwrite: bool = True) -> None:
        """
        Sets the traffic light phases for the simulation based on movements, starting at the next simulation step.
        
        Args:
            `junction_phases` (dict): Dictionary containing junction phases, times and masks for different movements
            `start_phase` (int): Phase number to start at, defaults to 0
            `overwrite` (bool): If `True`, the `junc_phases` dict is overwitten with `junction_phases`. If `False`, only specific junctions are overwritten.
        """

        new_phase_dict = {}

        for junction_id, junc_phase in junction_phases.items():

            if junction_id not in list(traci.trafficlight.getIDList()):
                desc = "Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(junction_id)
                raise_error(KeyError, desc, self.curr_step)
            else:
                if junction_id in self.tracked_junctions.keys():
                    m_len = self.tracked_junctions[junction_id]._m_len
                else:
                    state_str = traci.trafficlight.getRedYellowGreenState(junction_id)
                    m_len = len(state_str)

            valid_params = {"phases": dict, "times": dict, "masks": dict, "curr_phase": int}
            error, desc = test_input_dict(junc_phase, valid_params, "'{0}' phase dict".format(junction_id), required=["times", "phases", "masks"])
            if error != None: raise_error(error, desc, self.curr_step)

            if set(junc_phase["phases"].keys()) == set(junc_phase["times"].keys()) == set(junc_phase["masks"].keys()):
                m_keys = list(junc_phase["phases"].keys())

                valid_params = {m_key: list for m_key in m_keys}
                error, desc = test_input_dict(junc_phase["phases"], valid_params, "'{0}' phases".format(junction_id), required=True)
                if error != None: raise_error(error, desc, self.curr_step)

                cycle_length = None
                for m_key in m_keys:
                    colours, times, mask = junc_phase["phases"][m_key], junc_phase["times"][m_key], junc_phase["masks"][m_key]

                    validate_list_types(colours, str, param_name="junction '{0}', movement '{1}' colours".format(junction_id, m_key), curr_sim_step=self.curr_step)
                    validate_list_types(times, (int, float), param_name="junction '{0}', movement '{1}' times".format(junction_id, m_key), curr_sim_step=self.curr_step)

                    if not isinstance(mask, str):
                        desc = "Invalid mask in junction '{0}', movement '{1}' (mask '{2}' is '{3}', must be str).".format(junction_id, m_key, mask, type(mask).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                    elif len(mask) != m_len:
                        desc = "Invalid mask in junction '{0}', movement '{1}' (mask '{2}' length does not match junction '{3}').".format(junction_id, m_key, mask, m_len)
                        raise_error(ValueError, desc, self.curr_step)

                    if len(colours) != len(times):
                        desc = "Invalid phases in junction '{0}', movement '{1}' (colour and time arrays are different lengths).".format(junction_id, m_key)
                        raise_error(ValueError, desc, self.curr_step)
                    elif cycle_length != None and sum(times) != cycle_length:
                        desc = "Invalid phases in junction '{0}', movement '{1}' (movement cycle length '{2}' != junction cycle length '{3}').".format(junction_id, m_key, sum(times), cycle_length)
                        raise_error(ValueError, desc, self.curr_step)
                    else: cycle_length = sum(times)

                    valid_colours = {"G", "g", "y", "r", "-"}
                    invalid_colours = list(set(colours) - valid_colours)
                    if len(invalid_colours) > 0:
                        invalid_colours.sort()
                        desc = "Invalid phase colour(s) in junction '{0}', movement '{1}' (['{2}'] are invalid, must be in ['{3}']).".format(junction_id, m_key, "','".join(invalid_colours), "'|'".join(list(valid_colours)))
                        raise_error(ValueError, desc, self.curr_step)

                complete, new_junc_phases = False, {"phases": [], "times": []}

                all_phases, all_times = junc_phase["phases"], junc_phase["times"]
                curr_phases = {m_key: junc_phase["phases"][m_key][0] for m_key in m_keys}
                curr_times = {m_key: junc_phase["times"][m_key][0] for m_key in m_keys}
                masks = junc_phase["masks"]

                while not complete:

                    phase_colours = _get_phase_string(curr_phases, masks)
                    phase_time = min(list(curr_times.values()))

                    new_junc_phases["phases"].append(phase_colours)
                    new_junc_phases["times"].append(phase_time)

                    for m_key in m_keys:
                        curr_times[m_key] -= phase_time

                        if curr_times[m_key] <= 0:
                            all_phases[m_key].pop(0)
                            all_times[m_key].pop(0)
                            
                            if len(all_phases[m_key]) > 0:
                                curr_phases[m_key] = all_phases[m_key][0]
                                curr_times[m_key] = all_times[m_key][0]

                    complete = sum([len(phases) for phases in all_phases.values()]) == 0

                new_phase_dict[junction_id] = new_junc_phases

            else:
                desc = "Invalid phases for junction '{0}' (movement keys do not match).".format(junction_id)
                raise_error(KeyError, desc, self.curr_step)

        self.set_phases(new_phase_dict, start_phase, overwrite)

    def set_tl_colour(self, junction_id: str|int, colour_str: str) -> None:
        """
        Sets a junction to a colour for an indefinite amount of time. Can be used when tracking phases separately (ie. not within TUD-SUMO).
        
        Args:
            `junction_id` (str, int): Junction ID
            `colour_str` (str): Phase colour string (valid characters are ['_G_'|'_g_'|'_y_'|'_r_'|'-'])
        """
        
        if junction_id not in list(traci.trafficlight.getIDList()):
            desc = "Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(junction_id)
            raise_error(KeyError, desc, self.curr_step)
        else:
            if junction_id in self.tracked_junctions.keys():
                m_len = self.tracked_junctions[junction_id]._m_len
            else:
                state_str = traci.trafficlight.getRedYellowGreenState(junction_id)
                m_len = len(state_str)

        colour_str = validate_type(colour_str, str, "colour_str", self.curr_step)
        
        if len(colour_str) == 1:
            junc_phases = {junction_id: {"phases": [colour_str*m_len], "times": [math.inf]}}
        elif len(colour_str) == m_len:
            junc_phases = {junction_id: {"phases": [colour_str], "times": [math.inf]}}
        else:
            desc = "Invalid colour_str (must be char or len(str) == junction movements length)."
            raise_error(ValueError, desc, self.curr_step)
        
        self.set_phases(junc_phases, overwrite=False)

    def set_tl_metering_rate(self, rm_id: str, metering_rate: int|float, g_time: int|float = 1, y_time: int|float = 1, min_red: int|float = 1, vehs_per_cycle: int|None = None, control_interval: int|float = 60) -> dict:
        """
        Set ramp metering rate of a meter at a junction. Uses a one-car-per-green policy with a default
        1s green and yellow time, with red phase duration changed to set flow. All phase durations must
        be larger than the simulation step length.
        
        Args:
            `rm_id` (str): Ramp meter (junction) ID
            `metering_rate` (int, float): On-ramp inflow in veh/hr (from all lanes)
            `g_time` (int, float): Green phase duration (s), defaults to 1
            `y_time` (int, float): Yellow phase duration (s), defaults to 1
            `min_red` (int, float): Minimum red phase duration (s), defaults to 1
            `vehs_per_cycle` (int, optional):Number of vehicles released with each cycle, defaults to the number of lanes
            `control_interval` (int, float): Ramp meter control interval (s)
        
        Returns:
            dict: Resulting phase dictionary
        """
        
        if min([g_time, y_time, min_red]) <= self.step_length:
            desc = "Green ({0}), yellow ({1}) and minimum red ({2}) times must all be greater than sim step length ({3}).".format(g_time, y_time, min_red, self.step_length)

        if rm_id not in list(traci.trafficlight.getIDList()):
            desc = "Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(rm_id)
            raise_error(KeyError, desc, self.curr_step)
        else:
            if rm_id in self.tracked_junctions.keys():
                m_len = self.tracked_junctions[rm_id]._m_len
            else:
                state_str = traci.trafficlight.getRedYellowGreenState(rm_id)
                m_len = len(state_str)

        if self.track_juncs and rm_id in self.tracked_junctions.keys():
            if len(self.tracked_junctions[rm_id]._rate_times) == 0 or self.curr_step > self.tracked_junctions[rm_id]._rate_times[-1]:
                self.tracked_junctions[rm_id]._metering_rates.append(metering_rate)
                self.tracked_junctions[rm_id]._rate_times.append(self.curr_step)
            else:
                self.tracked_junctions[rm_id]._metering_rates[-1] = metering_rate

        # Max flow for one-car-per-green
        max_flow = (3600 / (g_time + y_time + min_red))
        
        if vehs_per_cycle == None: vehs_per_cycle = m_len

        # Max flow for n-car-per-green (accounting for no. lanes)
        max_flow *= vehs_per_cycle

        # With one lane and a g_time, y_time and min_red of 1s, the meter cannot physically release
        # more than 1200 veh/hr without reducing minimum red. So, when the metering rate is above
        # this upper bound, the meter is set to green for the whole control interval.

        # This maximum flow is increased with 2 (or more) lanes as, even with 1s green time, this essentially
        # becomes a two-car-per-green policy, and so the maximum flow is doubled.
        if metering_rate > max_flow:
            phases_dict = {"phases": ["G"*m_len], "times": [control_interval]}
        elif metering_rate == 0:
            phases_dict = {"phases": ["r"*m_len], "times": [control_interval]}
        elif metering_rate < 0:
            desc = "Metering rate must be greater than 0 (set to '{0}').".format(metering_rate)
            raise_error(ValueError, desc, self.curr_step)
        else:

            # Number of vehicles to be released per control interval
            vehicles_per_ci = (metering_rate / 3600) * control_interval

            # Number of cycles needed per control interval to achieve metering rate
            n_cycles_per_ci = vehicles_per_ci / vehs_per_cycle

            # red time calculated with the number of cycles per control interval, minus g + y time
            cycle_length = control_interval / n_cycles_per_ci
            red_time = cycle_length - g_time - y_time

            phases_dict = {"phases": ["G"*m_len, "y"*m_len, "r"*m_len],
                        "times":  [g_time, y_time, red_time]}
        self.set_phases({rm_id: phases_dict}, overwrite=False)
        return phases_dict

    def change_phase(self, junction_id: str|int, phase_number: int) -> None:
        """
        Change to a different phase at the specified junction_id.
        
        Args:
            `junction_id` (str, int): Junction ID
            `phase_number` (int): Phase number
        """
        
        if 0 < phase_number < len(self._junc_phases[junction_id]["phases"]):
            self._junc_phases[junction_id]["curr_phase"] = phase_number
            self._junc_phases[junction_id]["curr_time"] = sum(self.junc_phase["times"][:phase_number])

            self._update_lights(junction_id)

        else:
            desc = "Invalid phase number '{0}' (must be [0-{1}]).".format(phase_number, len(self._junc_phases[junction_id]["phases"]))
            raise_error(ValueError, desc, self.curr_step)

    def _update_lights(self, junction_ids: list|str|None = None) -> None:
        """
        Update light settings for given junctions.
        
        Args:
            `junction_ids` (list, str, optional):Junction ID, or list of IDs (defaults to all)
        """

        if junction_ids is None: junction_ids = self._junc_phases.keys()
        elif isinstance(junction_ids, str): junction_ids = [junction_ids]
        junction_ids = validate_list_types(junction_ids, str, param_name="junction_ids", curr_sim_step=self.curr_step)

        for junction_id in junction_ids:
            curr_setting = traci.trafficlight.getRedYellowGreenState(junction_id)
            new_phase = self._junc_phases[junction_id]["phases"][self._junc_phases[junction_id]["curr_phase"]]
            if '-' in new_phase:
                new_phase = new_phase.split()
                new_phase = "".join([new_phase[i] if new_phase[i] != '-' else curr_setting[i] for i in range(len(new_phase))])
            traci.trafficlight.setRedYellowGreenState(junction_id, new_phase)

    def add_vehicle(self, vehicle_id: str, vehicle_type: str, routing: str|list|tuple, initial_speed: str|int|float="max", origin_lane: str|int="best", origin_pos: str|int|float = "base") -> None:
        """
        Add a new vehicle into the simulation.
        
        Args:
            `vehicle_id` (str): ID for new vehicle, **must be unique**
            `vehicle_type` (str): Vehicle type ID for new vehicle
            `routing` (str, list, tuple): Either route ID or (2x1) list of edge IDs for origin-destination pair
            `initial_speed` (str, int, float): Initial speed at insertion, either ['_max_'|'_random_'] or number > 0
            `origin_lane` (str, int, float): Lane for insertion at origin, either ['_random_'|'_free_'|'_allowed_'|'_best_'|'_first_'] or lane index
            `origin_pos` (str, int): Longitudinal position at insertion, either ['_random_'|'_free_'|'_random_free_'|'_base_'|'_last_'|'_stop_'|'_splitFront_'] or offset
        """

        if self.vehicle_exists(vehicle_id):
            desc = "Invalid vehicle_id given '{0}' (must be unique).".format(vehicle_id)
            raise_error(ValueError, desc, self.curr_step)
        
        origin_lane = validate_type(origin_lane, (str, int), "origin_lane", self.curr_step)
        initial_speed = validate_type(initial_speed, (str, int, float), "initial_speed", self.curr_step)
        if isinstance(initial_speed, str) and initial_speed not in ["max", "random"]:
            desc = "Invalid initial_speed string given '{0}' (must be ['_max_'|'_random_']).".format(initial_speed, type(initial_speed).__name__)
            raise_error(TypeError, desc, self.curr_step)
        elif isinstance(initial_speed, (int, float)) and initial_speed < 0:
            desc = "Invalid initial_speed value given '{0}' (must be > 0).".format(initial_speed, type(initial_speed).__name__)
            raise_error(TypeError, desc, self.curr_step)

        if isinstance(initial_speed, (int, float)):
            initial_speed = convert_units(initial_speed, self._speed_unit, "m/s")

        if not self.vehicle_type_exists(vehicle_type) and vehicle_type != "default":
            desc = "Vehicle type ID '{0}' not found.".format(vehicle_type)
            raise_error(TypeError, desc, self.curr_step)

        routing = validate_type(routing, (str, list, tuple), "routing", self.curr_step)
        if isinstance(routing, str):
            route_id = routing
            routing = self.route_exists(route_id)
            if routing != None:
                if vehicle_type != "default": traci.vehicle.add(vehicle_id, route_id, vehicle_type, departLane=origin_lane, departSpeed=initial_speed, departPos=origin_pos)
                else: traci.vehicle.add(vehicle_id, route_id, departLane=origin_lane, departSpeed=initial_speed, departPos=origin_pos)
            else:
                desc = "Route ID '{0}' not found.".format(route_id)
                raise_error(KeyError, desc, self.curr_step)

        elif isinstance(routing, (list, tuple)):
            routing = validate_list_types(routing, str, param_name="routing", curr_sim_step=self.curr_step)
            if len(routing) == 2:
                if not self.is_valid_path(routing):
                    desc = "No route between edges '{0}' and '{1}'.".format(routing[0], routing[1])
                    raise_error(ValueError, desc, self.curr_step)

                for geometry_id in routing:
                    g_class = self.geometry_exists(geometry_id)
                    if g_class == "lane":
                        desc = "Invalid geometry type (Edge ID required, '{0}' is a lane).".format(geometry_id)
                        raise_error(TypeError, desc, self.curr_step)
                    
                    if isinstance(origin_lane, int):
                        n_lanes = self.get_geometry_vals(geometry_id, "n_lanes")
                        if origin_lane >= n_lanes or origin_lane < 0:
                            desc = "Invalid origin lane index '{0}' (must be (0 <= origin_lane < n_lanes '{1}'))".format(origin_lane, n_lanes)
                            raise_error(ValueError, desc, self.curr_step)

                route_id = "_".join(routing)

                if self.route_exists(route_id) == None:
                    traci.route.add(route_id, routing)
                    self._all_routes[route_id] = tuple(routing)

                if vehicle_type != "default": traci.vehicle.add(vehicle_id, route_id, vehicle_type, departLane=origin_lane, departSpeed=initial_speed, departPos=origin_pos)
                else: traci.vehicle.add(vehicle_id, route_id, departLane=origin_lane, departSpeed=initial_speed, departPos=origin_pos)
                self._vehicles_in(vehicle_id)

            else:
                desc = "Invalid routing given '[{0}]' (must have shape (2x1)).".format(",".join(routing))
                raise_error(TypeError, desc, self.curr_step)
    
    def remove_vehicles(self, vehicle_ids: str|list|tuple) -> None:
        """
        Remove a vehicle or list of vehicles from the simulation.
        
        Args:
            `vehicle_ids` (str, list, tuple): List of vehicle IDs or single ID
        """
        
        if isinstance(vehicle_ids, str): vehicle_ids = [vehicle_ids]
        vehicle_ids = validate_list_types(vehicle_ids, str, param_name="vehicle_ids", curr_sim_step=self.curr_step)

        for vehicle_id in vehicle_ids:
            if self.vehicle_exists(vehicle_id):
                traci.vehicle.remove(vehicle_id, reason=traci.constants.REMOVE_VAPORIZED)
                self._vehicles_out(vehicle_id, is_removed=True)
            else:
                desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
                raise_error(KeyError, desc, self.curr_step)

    def cause_incident(self, duration: int, vehicle_ids: str|list|tuple|None=None, geometry_ids: str|list|tuple=None, n_vehicles: int=1, vehicle_separation: float=0, assert_n_vehicles: bool=False, edge_speed: int|float|None=-1, highlight_vehicles: bool=True, incident_id: str|None=None) -> bool:
        """
        Simulates an incident by stopping vehicles on the road for a period of time, before removing
        them from the simulation. Vehicle(s) can either be specified using `vehicle_ids`, chosen
        randomly based location using `geometry_ids`, or vehicles can be chosen randomly throughout
        the network if neither `vehicle_ids` or `geometry_ids` are given.
        
        Args:
            `duration` (int): Duration of incident (in simulation steps)
            `vehicle_ids` (str, list, tuple, optional):Vehicle ID or list of IDs to include in the incident
            `geometry_ids` (str, list, tuple, optional):Geometry ID or list of IDs to randomly select vehicles from
            `n_vehicles` (int): Number of vehicles in the incident, if randomly chosen
            `vehicle_separation` (float): Factor denoting how separated randomly chosen vehicles are (0.1-1)
            `assert_n_vehicles` (bool): Denotes whether to throw an error if the correct number of vehicles cannot be found
            `edge_speed` (int, float, optional):New max speed for edges where incident vehicles are located (defaults to 15km/h or 10mph)
            `highlight_vehicles` (bool): Denotes whether to highlight vehicles in the SUMO GUI
            `incident_id` (str, optional):Incident event ID used in the simulation data file (defaults to '_incident_{n}_')

        Returns:
            bool: Denotes whether incident was successfully created
        """
        
        if self._scheduler == None:
            self._scheduler = EventScheduler(self)
            
        if incident_id == None:
            id_idx = 1
            while self._scheduler.get_event_status("incident_{0}".format(id_idx)) != None: id_idx += 1
            incident_id = "incident_{0}".format(id_idx)

        event_dict = {"start_time": (self.curr_step + 1) * self.step_length, "end_time": (self.curr_step + duration) * self.step_length,
                      "vehicles": {"actions": {"speed": 0}, "effect_duration": duration, "remove_affected_vehicles": True,
                                   "speed_safety_checks": False, "lc_safety_checks": False}}
        
        check_n_vehicles = vehicle_ids == None

        # Completely random incident (no location or vehicles specified)
        if geometry_ids == None and vehicle_ids == None:
            if n_vehicles < len(self._all_curr_vehicle_ids) and n_vehicles > 0:
                all_geometry_ids, geometry_ids, vehicle_ids, found_central = list(self._all_edges), [], [], False

                # A central edge is chosen (one that contains at least 1 vehicle)
                while not found_central:
                    central_id = choice(all_geometry_ids)
                    found_central = self.get_geometry_vals(central_id, "vehicle_count") > 0 and not central_id.startswith(":")

                vehicle_separation = min(0.9, max(0, vehicle_separation))
                searched, to_search, prob = [], [central_id], 1 - vehicle_separation

                # Then, vehicles are chosen for the incident, starting on the central edge.
                while len(vehicle_ids) < n_vehicles and len(to_search) > 0:
                    curr_geometry_id = choice(to_search)

                    all_geometry_vehicles = self.get_geometry_vals(curr_geometry_id, "vehicle_ids")
                    
                    for g_veh_id in all_geometry_vehicles:

                        # Vehicles are chosen randomly using the vehicle_separation
                        # parameter as the probability. High vehicle separation will likely
                        # mean vehicles are spread across different edges (assuming n_vehicles is also high)
                        if random() < prob:
                            vehicle_ids.append(g_veh_id)
                            geometry_ids.append(curr_geometry_id)
                            if len(vehicle_ids) >= n_vehicles:
                                break

                    if len(vehicle_ids) < n_vehicles:
                        to_search.remove(curr_geometry_id)
                        searched.append(curr_geometry_id)
                        
                        connected_edges = self.get_geometry_vals(curr_geometry_id, "connected_edges")
                        to_search += connected_edges['incoming']
                        to_search += connected_edges['outgoing']

                        # If there are still not enough vehicles, we then search an adjacent edge.
                        to_search = [g_id for g_id in to_search if g_id not in searched and not g_id.startswith(":")]

                geometry_ids = list(set(geometry_ids))

            else:
                desc = "Invalid n_vehicles '{0}' (must be 0 < '{0}' < no. vehicles in the simulation '{1}').".format(n_vehicles, len(self._all_curr_vehicle_ids))
                raise_error(ValueError, desc, self.curr_step)

        # Location specified, but vehicles are randomly chosen
        elif geometry_ids != None and vehicle_ids == None:
            if isinstance(geometry_ids, str): geometry_ids = [geometry_ids]
            geometry_ids = validate_list_types(geometry_ids, str, param_name="geometry_ids", curr_sim_step=self.curr_step)

            all_geometry_vehicles = self.get_last_step_geometry_vehicles(geometry_ids)
            vehicle_ids = choices(all_geometry_vehicles, min(n_vehicles, len(all_geometry_vehicles)))

        # Neither location or vehicles specified - an error is thrown
        elif geometry_ids != None and vehicle_ids != None:
            desc = "Invalid inputs (cannot use both vehicle_ids and geometry_ids)."
            raise_error(ValueError, desc, self.curr_step)
            
        if check_n_vehicles:
            if len(vehicle_ids) != n_vehicles:
                if assert_n_vehicles:
                    desc = "Incident could not be started (could not find enough vehicles, {0} != {1}).".format()
                    raise_error(SimulationError, desc, self.curr_step)
                else:
                    if not self._suppress_warnings: raise_warning("Incident could not be started (could not find enough vehicles, {0} != {1}).".format())
                    return False

        # Either specific vehicles are given to be included in the incident, or
        # vehicle_ids contains the list of randomly selected vehicles
        if vehicle_ids != None:
            if isinstance(vehicle_ids, str): vehicle_ids = [vehicle_ids]
            vehicle_ids = validate_list_types(vehicle_ids, str, param_name="vehicle_ids", curr_sim_step=self.curr_step)

            for vehicle_id in vehicle_ids:
                if not self.vehicle_exists(vehicle_id):
                    desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
                    raise_error(KeyError, desc, self.curr_step)
                elif highlight_vehicles:
                    self.set_vehicle_vals(vehicle_id, highlight=True)

            event_dict["vehicles"]["vehicle_ids"] = vehicle_ids
        
            if edge_speed != None:
                if edge_speed < 0: edge_speed = 15 if self.units.name == "METRIC" else 10
                if geometry_ids == None: geometry_ids = [self.get_vehicle_vals(veh_id, "edge_id") for veh_id in vehicle_ids]
                event_dict["edges"] = {"edge_ids": geometry_ids, "actions": {"max_speed": edge_speed}}

        self.add_events({incident_id: event_dict})
        return True
            
    def vehicle_exists(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle exists in the network and has departed.
        
        Returns:
            bool: `True` if ID in list of current vehicle IDs 
        """

        vehicle_id = validate_type(vehicle_id, str, "vehicle_id", self.curr_step)

        return vehicle_id in self._all_curr_vehicle_ids
    
    def junction_exists(self, junction_id: str) -> bool:
        """
        Tests if a junction exists in the network.
        
        Returns:
            bool: `True` if ID in list of all junction IDs 
        """

        junction_id = validate_type(junction_id, str, "junction_id", self.curr_step)

        return junction_id in self._all_juncs
    
    def tracked_junction_exists(self, junction_id: str) -> bool:
        """
        Tests if a tracked junction exists in the network.
        
        Returns:
            bool: `True` if ID in list of tracked junction IDs
        """

        junction_id = validate_type(junction_id, str, "junction_id", self.curr_step)

        return junction_id in self.tracked_junctions
    
    def tracked_edge_exists(self, edge_id: str) -> bool:
        """
        Tests if a tracked junction exists in the network.
        
        Returns:
            bool: `True` if ID in list of tracked junction IDs
        """

        edge_id = validate_type(edge_id, str, "edge_id", self.curr_step)

        return edge_id in self.tracked_edges
    
    def vehicle_loaded(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle is loaded (may not have departed).
        
        Returns:
            bool: `True` if ID in list of loaded vehicle IDs
        """

        vehicle_id = validate_type(vehicle_id, str, "vehicle_id", self.curr_step)

        return vehicle_id in self._all_loaded_vehicle_ids
    
    def vehicle_to_depart(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle is loaded but has not departed yet.
        
        Returns:
            bool: `True` if vehicle has not departed yet
        """

        vehicle_id = validate_type(vehicle_id, str, "vehicle_id", self.curr_step)

        if not self.vehicle_loaded(vehicle_id):
            desc = "Vehicle with ID '{0}' has not been loaded.".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)
        
        return vehicle_id in self._all_to_depart_vehicle_ids
    
    def add_vehicle_subscriptions(self, vehicle_ids: str|list|tuple, data_keys: str|list|tuple) -> None:
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

    def remove_vehicle_subscriptions(self, vehicle_ids: str|list|tuple) -> None:
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
    
    def add_detector_subscriptions(self, detector_ids: str|list|tuple, data_keys: str|list|tuple) -> None:
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

    def remove_detector_subscriptions(self, detector_ids: str|list|tuple) -> None:
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
    
    def add_geometry_subscriptions(self, geometry_ids: str|list|tuple, data_keys: str|list|tuple) -> None:
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

    def remove_geometry_subscriptions(self, geometry_ids: str|list|tuple) -> None:
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
    
    def set_vehicle_vals(self, vehicle_ids: list|tuple|str, **kwargs) -> None:
        """
        Changes vehicle characteristics.
        
        Args:
            `type` (str): Vehicle type ID
            `vehicle_ids` (list, tuple, str): Vehicle ID or list of IDs
            `colour` (str, list, tuple, optional): Vehicle colour, either hex code, list of rgb/rgba values or valid SUMO colour string
            `highlight` (bool): Highlights the vehicle with a circle (bool)
            `speed` (int, float): Set new speed value
            `max_speed` (int, float): Set new max speed value
            `acceleration` ((int, float), (int, float)): Set acceleration for a given duration 
            `lane_idx` (int, (int, float)): Try and change lane for a given duration
            `destination` (str): Set vehicle destination edge ID
            `route_id` (str): Set vehicle route by route ID or list of edges
            `route_edges` (list): Set vehicle route by list of edges
            `speed_safety_checks` (bool): (**Indefinitely**) set whether speed/acceleration safety constraints are followed when setting speed
            `lc_safety_checks` (bool): (**Indefinitely**) set whether lane changing safety constraints are followed when changing lane
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
                            desc = "({0}): Invalid speed value '{1}' (must be [int|float], not '{2}').".format(command, value, type(value).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                    
                    case "max_speed":
                        if isinstance(value, (int, float)):
                            traci.vehicle.setMaxSpeed(vehicle_id, convert_units(value, self._speed_unit, "m/s"))
                        else:
                            desc = "({0}): Invalid max_speed value '{1}' (must be [int|float], not '{2}').".format(command, value, type(value).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                    
                    case "acceleration":
                        if isinstance(value, (list, tuple)) and len(value) == 2:
                            if not isinstance(value[0], (int, float)):
                                desc = "({0}): Invalid acceleration '{1}' (must be [int|float], not '{2}').".format(command, value[0], type(value[0]).__name__)
                                raise_error(TypeError, desc, self.curr_step)
                            if not isinstance(value[1], (int, float)):
                                desc = "({0}): Invalid duration '{1}' (must be [int|float], not '{2}').".format(command, value[1], type(value[1]).__name__)
                                raise_error(TypeError, desc, self.curr_step)
                            traci.vehicle.setAcceleration(vehicle_id, float(value[0]), float(value[1]))
                        else:
                            desc = "({0}): '{0}' requires 2 parameters (acceleration [int|float], duration [int|float])".format(command)
                            raise_error(TypeError, desc, self.curr_step)

                    case "lane_idx":
                        if isinstance(value, (list, tuple)) and len(value) == 2:
                            if not isinstance(value[0], int):
                                desc = "({0}): Invalid lane_idx '{1}' (must be int, not '{2}').".format(command, value[0], type(value[0]).__name__)
                                raise_error(TypeError, desc, self.curr_step)
                            if not isinstance(value[1], (int, float)):
                                desc = "({0}): Invalid duration '{1}' (must be [int|float], not '{2}').".format(command, value[1], type(value[1]).__name__)
                                raise_error(TypeError, desc, self.curr_step)
                            traci.vehicle.changeLane(vehicle_id, value[0], float(value[1]))
                        else:
                            desc = "({0}): '{0}' requires 2 parameters (lane_idx [int], duration [int|float])".format(command)
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

    def set_vehicle_type_vals(self, vehicle_types: list|tuple|str, **kwargs) -> None:
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
            `acceleration` (int, float): Maximum vehicle acceleration
            `deceleration` (int, float): Maximum vehicle deceleration
            `tau` (int, float): Vehicle car following parameter
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
                            desc = f"({command}): Invalid length value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "width":
                        if isinstance(value, (int, float)):
                            if value <= 0.1:
                                desc = f"({command}): Invalid width value '{value}' (must be > 0.1)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setWidth(vehicle_type, convert_units(value, self._s_dist_unit, "metres"))
                        else:
                            desc = f"({command}): Invalid width value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "height":
                        if isinstance(value, (int, float)):
                            if value <= 0.1:
                                desc = f"({command}): Invalid height value '{value}' (must be > 0.1)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setHeight(vehicle_type, convert_units(value, self._s_dist_unit, "metres"))
                        else:
                            desc = f"({command}): Invalid height value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "mass":
                        if isinstance(value, (int, float)):
                            if value <= 0.1:
                                desc = f"({command}): Invalid mass value '{value}' (must be > 0.1)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setMass(vehicle_type, convert_units(value, self._weight_unit, "kilograms"))
                        else:
                            desc = f"({command}): Invalid mass value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "max_speed":
                        if isinstance(value, (int, float)):
                            if value <= 0.1:
                                desc = f"({command}): Invalid max_speed value '{value}' (must be > 0.1)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setMaxSpeed(vehicle_type, convert_units(value, self._speed_unit, "m/s"))
                        else:
                            desc = f"({command}): Invalid max_speed value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "speed_factor":
                        if isinstance(value, (int, float)):
                            if value <= 0:
                                desc = f"({command}): Invalid speed_factor value '{value}' (must be > 0)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setSpeedFactor(vehicle_type, value)
                        else:
                            desc = f"({command}): Invalid speed_factor value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "speed_dev":
                        if isinstance(value, (int, float)):
                            if value < 0:
                                desc = f"({command}): Invalid speed_dev value '{value}' (must be >= 0)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setSpeedDeviation(vehicle_type, value)
                        else:
                            desc = f"({command}): Invalid speed_dev value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "acceleration":
                        if isinstance(value, (int, float)):
                            if value < 0:
                                desc = f"({command}): Invalid acceleration value '{value}' (must be >= 0)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setAccel(vehicle_type, value)
                        else:
                            desc = f"({command}): Invalid acceleration value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "deceleration":
                        if isinstance(value, (int, float)):
                            if value < 0:
                                desc = f"({command}): Invalid deceleration value '{value}' (must be >= 0)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setDecel(vehicle_type, value)
                        else:
                            desc = f"({command}): Invalid deceleration value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "tau":
                        if isinstance(value, (int, float)):
                            if value < 0:
                                desc = f"({command}): Invalid tau value '{value}' (must be >= 0)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setTau(vehicle_type, value)
                        else:
                            desc = f"({command}): Invalid tau value '{value}' (must be [int|float], not '{type(value).__name__}')."
                            raise_error(TypeError, desc, self.curr_step)

                    case "max_lateral_speed":
                        if isinstance(value, (int, float)):
                            if value < 0:
                                desc = f"({command}): Invalid max_lateral_speed value '{value}' (must be >= 0)."
                                raise_error(ValueError, desc, self.curr_step)
                            traci.vehicletype.setTau(vehicle_type, value)
                        else:
                            desc = f"({command}): Invalid max_lateral_speed value '{value}' (must be [int|float], not '{type(value).__name__}')."
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

    def get_vehicle_type_vals(self, vehicle_types: str|list|tuple, data_keys: str|list) -> dict|str|float|tuple:
        """
        Get data values for specific vehicle type(s) using a list of data keys. Valid data keys are;
        '_vehicle_class_', '_colour_', '_length_', '_width_', '_height_', '_mass_', '_max_speed_', '_speed_factor_', '_speed_dev_',
        '_min_gap_', '_acceleration_', '_deceleration_', '_tau_', '_max_lateral_speed_', '_emission_class_', '_gui_shape_'
        
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
                desc = "Invalid data_keys given '{0}' (must be [str|(str)], not '{1}').".format(data_keys, type(data_keys).__name__)
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

                    case "acceleration":
                        data_vals[data_key] = traci.vehicletype.getAccel(vehicle_type)

                    case "deceleration":
                        data_vals[data_key] = traci.vehicletype.getDecel(vehicle_type)

                    case "tau":
                        data_vals[data_key] = traci.vehicletype.getTau(vehicle_type)

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

    def get_vehicle_ids(self, vehicle_types: str|list|tuple|None = None) -> list:
        """
        Return list of IDs for all vehicles currently in the simulation.
        
        Args:
            `vehicle_types` (str, list, tuple, optional):Vehicle type ID or list of IDs (defaults to all)

        Returns:
            list: All current vehicle IDs
        """

        if vehicle_types == None:
            return list(self._all_curr_vehicle_ids)
        else:
            if isinstance(vehicle_types, str): vehicle_types = [vehicle_types]
            vehicle_types = validate_list_types(vehicle_types, str, param_name="vehicle_types", curr_sim_step=self.curr_step)

            for vehicle_type in vehicle_types:
                if not self.vehicle_type_exists(vehicle_type):
                    desc = "Vehicle type ID '{0}' not found.".format(vehicle_type, type(vehicle_type).__name__)
                    raise_error(TypeError, desc, self.curr_step)
            
            vehicle_ids = []
            for vehicle_id in list(self._all_curr_vehicle_ids):
                if self.get_vehicle_vals(vehicle_id, "type") in vehicle_types: vehicle_ids.append(vehicle_id)

            return vehicle_ids
    
    def get_junction_ids(self) -> list:
        """
        Return list of all junctions in the network.
        
        Returns:
            list: All junction IDs
        """

        return list(self._all_juncs)
    
    def get_tracked_junction_ids(self) -> list:
        """
        Return list of all tracked junctions in the network.
        
        Returns:
            list: All tracked junction IDs
        """

        return list(self.tracked_junctions.keys())
    
    def get_tracked_edge_ids(self) -> list:
        """
        Return list of all tracked edges in the network.
        
        Returns:
            list: All tracked edges IDs
        """

        return list(self.tracked_edges.keys())
    
    def get_vehicle_types(self) -> list:
        """
        Return list of all valid vehicle type IDs.
        
        Returns:
            list: All vehicle type IDs
        """

        return list(self._vehicle_types) + list(self._added_vehicle_types)
    
    def get_geometry_ids(self, geometry_types: str|list|tuple|None = None) -> list:
        """
        Return list of IDs for all edges and lanes in the network.
        
        Args:
            `geometry_types` (str, list, tuple, optional):Geometry type ['_edge_'|'_lane_'] or list of types
        
        Returns:
            list: All geometry types (of type)
        """

        valid_types = ["edge", "lane"]

        if geometry_types == None: geometry_types = valid_types
        elif isinstance(geometry_types, str): geometry_types = [geometry_types]
        geometry_types = validate_list_types(geometry_types, str, param_name="geometry_types", curr_sim_step=self.curr_step)

        if len(set(geometry_types) - set(valid_types)) != 0:
            desc = "Invalid geometry types (must be 'edge' and/or 'lane')."
            raise_error(ValueError, desc, self.curr_step)
        else: geometry_types = [g_type.lower() for g_type in geometry_types]

        geometry_ids = []
        if "edge" in geometry_types:
            geometry_ids += self._all_edges
        if "lane" in geometry_types:
            geometry_ids += self._all_lanes
        
        return geometry_ids

    def get_detector_ids(self, detector_types: str|list|tuple|None = None) -> list:
        """
        Return list of IDs for all edges and lanes in the network.
        
        Args:
            `detector_types` (str, list, tuple, optional):Detector type ['_multientryexit_'|'_inductionloop_'] or list of types
        
        Returns:
            list: All detector IDs (of type)
        """

        valid_types = ["multientryexit", "inductionloop"]

        if detector_types == None: detector_types = valid_types
        elif isinstance(detector_types, str): detector_types = [detector_types]
        detector_types = validate_list_types(detector_types, str, param_name="detector_types", curr_sim_step=self.curr_step)
            
        if len(set(detector_types) - set(valid_types)) != 0:
            desc = "Invalid detector types (must be 'multientryexit' and/or 'inductionloop')."
            raise_error(ValueError, desc, self.curr_step)

        detector_ids = []
        for det_id, det_info in self.available_detectors.items():
            if det_info["type"] in detector_types: detector_ids.append(det_id)
        
        return detector_ids
    
    def get_path_edges(self, origin: str, destination: str, curr_optimal: bool = True) -> list|None:
        """
        Find an optimal route between 2 edges (using the A* algorithm).
        
        Args:
            `origin` (str): Origin edge ID
            `destination` (str): Destination edge ID
            `curr_optimal` (str): Denotes whether to find current optimal route (ie. whether to consider current conditions)
        
        Returns:
            (None, (list, float)): List of edge IDs & travel time (s) (based on curr_optimal), or None if no route exists
        """
        origin = validate_type(origin, str, "origin", self.curr_step)
        destination = validate_type(destination, str, "destination", self.curr_step)

        if origin == destination:
            desc = "Invalid origin-destination pair ({0}, {1}).".format(origin, destination)
            raise_error(ValueError, desc, self.curr_step)
        if self.geometry_exists(origin) != "edge":
            desc = "Origin edge with ID '{0}' not found.".format(origin)
            raise_error(KeyError, desc, self.curr_step)
        if self.geometry_exists(destination) != "edge":
            desc = "Destination edge with ID '{0}' not found.".format(origin)
            raise_error(KeyError, desc, self.curr_step)

        tt_key = "curr_travel_time" if curr_optimal else "ff_travel_time"

        h = {node: 1 for node in self._all_edges}
        open_list = set([origin])
        closed_list = set([])
        distances = {origin: 0}
        adjacencies = {origin: origin}

        # A* algorithm implemented
        while len(open_list) > 0:
            curr_edge = None

            for node in open_list:
                if curr_edge == None or distances[node] + h[node] < distances[curr_edge] + h[curr_edge]: curr_edge = node
 
            if curr_edge == None: return None
 
            if curr_edge == destination:
                optimal_path = []
 
                while adjacencies[curr_edge] != curr_edge:
                    optimal_path.append(curr_edge)
                    curr_edge = adjacencies[curr_edge]
 
                optimal_path.append(origin)
                optimal_path.reverse()

                return optimal_path, self.get_path_travel_time(optimal_path, curr_optimal)
 
            outgoing_edges = self.get_geometry_vals(curr_edge, "outgoing_edges")
            for connected_edge in outgoing_edges:

                if connected_edge not in open_list and connected_edge not in closed_list:
                    open_list.add(connected_edge)
                    adjacencies[connected_edge] = curr_edge
                    distances[connected_edge] = distances[curr_edge] + self.get_geometry_vals(connected_edge, tt_key)
                    
                else:
                    if distances[connected_edge] > distances[curr_edge] + self.get_geometry_vals(connected_edge, tt_key):
                        distances[connected_edge] = distances[curr_edge] + self.get_geometry_vals(connected_edge, tt_key)
                        adjacencies[connected_edge] = curr_edge
 
                        if connected_edge in closed_list:
                            closed_list.remove(connected_edge)
                            open_list.add(connected_edge)
 
            open_list.remove(curr_edge)
            closed_list.add(curr_edge)
 
        return None
    
    def get_path_travel_time(self, edge_ids: list|tuple, curr_tt: bool = True, unit: str = "seconds") -> float:
        """
        Calculates the travel time for a route.
        
        Args:
            `edge_ids` (list, tuple): List of edge IDs
            `curr_tt` (bool): Denotes whether to find the current travel time (ie. whether to consider current conditions)
            `unit` (str): Time unit (either ['_steps_'|'_seconds_'|'_minutes_'|'_hours_']) (defaults to seconds)
        
        Returns:
            float: Travel time in specified unit
        """
        
        edge_ids = validate_list_types(edge_ids, str, param_name="edge_ids", curr_sim_step=self.curr_step)
        
        tt_key = "curr_travel_time" if curr_tt else "ff_travel_time"
        total_tt = sum([self.get_geometry_vals(edge_id, tt_key) for edge_id in edge_ids])

        return convert_units(total_tt, "hours", unit, self.step_length)
    
    def is_valid_path(self, edge_ids: list|tuple) -> bool:
        """
        Checks whether a list of edges is a valid connected path. If two disconnected
        edges are given, it returns whether there is a path between them.
        
        Args:
            `edge_ids` (list, tuple): List of edge IDs
        
        Returns:
            bool: Denotes whether it is a valid path
        """

        edge_ids = validate_list_types(edge_ids, str, param_name="edge_ids", curr_sim_step=self.curr_step)
        if isinstance(edge_ids, (list, tuple)):
            if len(edge_ids) == 0:
                desc = "Empty edge ID list."
                raise_error(ValueError, desc, self.curr_step)
                
            for edge_id in edge_ids:
                if self.geometry_exists(edge_id) != 'edge':
                    desc = "Edge with ID '{0}' not found.".format(edge_id)
                    raise_error(KeyError, desc, self.curr_step)

            if len(edge_ids) == 1:
                return True
            
            elif len(edge_ids) == 2:
                if edge_ids[-1] not in self.get_geometry_vals(edge_ids[0], "outgoing_edges"):
                    return self.get_path_edges(edge_ids[0], edge_ids[1]) != None
                else: return True
            
            else:
                for idx in range(len(edge_ids) - 1):
                    if edge_ids[idx + 1] not in self.get_geometry_vals(edge_ids[idx], "outgoing_edges"): return False

        return True
    
    def add_route(self, routing: list|tuple, route_id: str|None = None, assert_new_id: bool = True) -> None:
        """
        Add a new route. If only 2 edge IDs are given, vehicles calculate
        optimal route at insertion, otherwise vehicles take specific edges.
        
        Args:
            `routing` (list, tuple): List of edge IDs
            `route_id` (str, optional):Route ID, if not given, generated from origin-destination
            `assert_new_id` (bool): If True, an error is thrown for duplicate route IDs
        """
        
        routing = validate_list_types(routing, str, param_name="routing", curr_sim_step=self.curr_step)
        if isinstance(routing, (list, tuple)):
            if len(routing) > 1 and self.is_valid_path(routing):
                if route_id == None:
                    route_id = "{0}_{1}".format(routing[0], routing[-1])

                if self.route_exists(route_id) == None:
                    traci.route.add(route_id, routing)
                    self._all_routes[route_id] = tuple(routing)
                    self._new_routes[route_id] = tuple(routing)

                elif assert_new_id:
                    desc = "Route or route with ID '{0}' already exists.".format(route_id)
                    raise_error(ValueError, desc, self.curr_step)

                elif not self._suppress_warnings:
                    raise_warning("Route or route with ID '{0}' already exists.".format(route_id), self.curr_step)

            else:
                desc = "No valid path between edges '{0}' and '{1}.".format(routing[0], routing[-1])
                raise_error(ValueError, desc, self.curr_step)

    def get_vehicle_vals(self, vehicle_ids: str|list|tuple, data_keys: str|list) -> dict|str|int|float|list:
        """
        Get data values for specific vehicle using a list of data keys. Valid data keys are;
        
        **Vehicle Characteristics**:
        '_type_', '_colour_', '_length_', 

        **Vehicle Status**:
        '_speed_', '_is_stopped_', '_max_speed_', '_allowed_speed_', '_acceleration_', '_position_', '_altitude_',
        '_heading_', '_edge_id_', '_lane_id_', '_lane_idx_', '_route_id_', '_route_idx_', '_route_edges_',
        '_leader_id_', '_leader_dist_'

        **Trip Data**:
        '_departure_', '_origin_', '_destination_'
        
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
                desc = "Invalid data_keys given '{0}' (must be [str|(str)], not '{1}').".format(data_keys, type(data_keys).__name__)
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

                    case "acceleration":
                        if subscription_key in subscribed_data: acceleration = subscribed_data[subscription_key]
                        else: acceleration = traci.vehicle.getAcceleration(vehicle_id)
                        data_vals[data_key] = acceleration

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
    
    def get_vehicle_data(self, vehicle_ids: str|list|tuple, refresh: bool = False) -> dict|None:
        """
        Get data for specified vehicle(s).
        
        Args:
            `vehicle_ids` (str, list, tuple): Vehicle ID or list of IDs
            `refresh` (bool): Denotes whether to update static vehicle data
        
        Returns:
            (dict, optional):Vehicle data dictionary, returns None if does not exist in simulation
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

    def _get_all_vehicle_data(self, vehicle_types: list|tuple|None = None) -> dict:
        """
        Collects aggregated vehicle data (no. vehicles & no. waiting vehicles) and all individual vehicle data.
        
        Args:
            `vehicle_types` (list, tuple, optional):Type(s) of vehicles to include
        
        Returns:
            dict: no vehicles, no waiting, all vehicle data
        """

        all_vehicle_data = {}
        total_vehicle_data = {"no_vehicles": 0, "no_waiting": 0, "delay": 0}
        lane_speeds, allowed_speeds = {}, {}

        for vehicle_id in self._all_curr_vehicle_ids:

            vehicle_type = self.get_vehicle_vals(vehicle_id, "type")

            if vehicle_types is None or (isinstance(vehicle_types, (list, tuple)) and vehicle_type in vehicle_types) or (isinstance(vehicle_types, str) and vehicle_type == vehicle_types):

                if self._get_individual_vehicle_data:
                    vehicle_data = self.get_vehicle_data(vehicle_id)
                    all_vehicle_data[vehicle_id] = vehicle_data
                else: vehicle_data = self.get_vehicle_vals(vehicle_id, ["speed", "lane_id", "allowed_speed", "is_stopped"])

                total_vehicle_data["no_vehicles"] += 1
                if vehicle_data["is_stopped"]: total_vehicle_data["no_waiting"] += 1

                lane_id, speed, allowed_speed = vehicle_data["lane_id"], vehicle_data["speed"], vehicle_data["allowed_speed"]

                # Vehicle is on an internal edge (in an intersection) - set to its previous lane
                if lane_id.startswith(':'): lane_id = self._network.getLane(lane_id).getIncoming()[0].getID()

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
                # delay = TTS on lane if speed == 0
                lane_delay = len(lane_data) * self.step_length

            elif average_speed < free_flow_speed:

                lane_length = convert_units(self._lane_info[lane_id]["length"], self._l_dist_unit, "metres")

                # step flow (veh) = speed (m/s) x density (veh/m) x step length (s)
                flow = (average_speed * (len(lane_data) / lane_length)) * self.step_length
                lane_delay = flow * ((lane_length / average_speed) - (lane_length / free_flow_speed))

            total_vehicle_data["delay"] += lane_delay

        total_vehicle_data["to_depart"] = len(self._all_to_depart_vehicle_ids)

        # Delay of vehicles waiting to be inserted into the simulation (veh*s)
        #total_vehicle_data["insertion_delay"] = total_vehicle_data["to_depart"] * self.step_length

        return total_vehicle_data, all_vehicle_data
    
    def get_geometry_vals(self, geometry_ids: str|list|tuple, data_keys: str|list) -> dict|str|int|float|list:
        """
        Get data values for specific edge or lane using a list of data keys. Valid data keys are:
        
        **Edge or Lane**:
        '_vehicle_count_', '_vehicle_ids_', '_vehicle_speed_', '_avg_vehicle_length_', '_halting_no_', '_vehicle_occupancy_',
        '_curr_travel_time_', '_ff_travel_time_', '_emissions_', '_length_', '_max_speed_'
        
        **Edge only**:
        '_connected_edges_', '_incoming_edges_', '_outgoing_edges_', '_street_name_', '_n_lanes_', '_lane_ids_', '_junction_ids_'
        
        **Lane only**: 
        '_edge_id_', '_n_links_', '_allowed_', '_disallowed_', '_left_lc_', '_right_lc_'.
        
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
                desc = "Invalid data_keys given '{0}' (must be [str|(str)], not '{1}').".format(data_keys, type(data_keys).__name__)
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
                        if subscription_key in subscribed_data: vehicle_speed = subscribed_data[subscription_key]
                        else: vehicle_speed = g_class.getLastStepMeanSpeed(geometry_id)

                        data_vals[data_key] = convert_units(vehicle_speed, "m/s", self._speed_unit)

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

    def set_geometry_vals(self, geometry_ids: str|list|tuple, **kwargs) -> None:
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
                            desc = "({0}): Invalid max_speed value '{1}' (must be [int|float], not '{2}').".format(command, value, type(value).__name__)
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
            
    def get_last_step_geometry_vehicles(self, geometry_ids: str|list, vehicle_types: list|None = None, flatten: bool = False) -> dict|list:
        """
        Get the IDs of vehicles on a lane or egde, by geometry ID.
        
        Args:
            `geometry_ids` (str, list):  Edge/lane ID or list of IDs
            `vehicle_types` (list, optional):Included vehicle type IDs
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
    
    def geometry_exists(self, geometry_id: str|int) -> str|None:
        """
        Get geometry type by ID, if geometry with the ID exists.
        
        Args:
            `geometry_id` (str, int): Lane or edge ID
        
        Returns:
            (str, optional):  Geometry type ['_edge_'|'_lane_'], or `None` if it does not exist
        """

        geometry_id = validate_type(geometry_id, str, "geometry_id", self.curr_step)

        if geometry_id in self._all_edges: return "edge"
        elif geometry_id in self._all_lanes: return "lane"
        else: return None

    def detector_exists(self, detector_id: str) -> str|None:
        """
        Get detector type by ID, if a detector with the ID exists.
        
        Args:
            `detector_id` (str): Detector ID
        
        Returns:
            (str, optional):Detector type, or `None` if it does not exist
        """

        detector_id = validate_type(detector_id, str, "detector_id", self.curr_step)

        if detector_id in self.available_detectors.keys():
            return self.available_detectors[detector_id]["type"]
        else: return None

    def route_exists(self, route_id: str) -> str|None:
        """
        Get route edges by ID, if a route with the ID exists.
        
        Args:
            `route_id` (str): Route ID
        
        Returns:
            (str, optional):List of route edges, or `None` if it does not exist.
        """

        route_id = validate_type(route_id, str, "route_id", self.curr_step)

        if route_id in self._all_routes.keys():
            return self._all_routes[route_id]
        else: return None

    def vehicle_type_exists(self, vehicle_type_id: str) -> bool:
        """
        Get whether vehicle type exists by ID.
        
        Args:
            `vehicle_type` (str): Vehicle type ID
        
        Returns:
            bool: Denotes whether vehicle type exists
        """

        vehicle_type_id = validate_type(vehicle_type_id, str, "vehicle_type_id", self.curr_step)

        return vehicle_type_id in list(self._vehicle_types) + list(self._added_vehicle_types)

    def get_event_ids(self, event_statuses: str|list|tuple|None = None) -> list:
        """
        Return event IDs by status, one or more of '_scheduled_', '_active_' or '_completed_'.
        
        Args:
            `event_statuses` (str, list, tuple, optional):Event status type or list of types
        
        Returns:
            list: List of event IDs
        """

        valid_statuses = ["scheduled", "active", "completed"]
        if isinstance(event_statuses, str): event_statuses = [event_statuses]
        elif event_statuses == None: event_statuses = valid_statuses
        elif not isinstance(event_statuses, (list, tuple)):
            desc = "Invalid event_statuses '{0}' type (must be [str|list|tuple], not '{1}').".format(event_statuses, type(event_statuses).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        if len(set(event_statuses) - set(valid_statuses)) != 0:
            desc = "Invalid event_statuses (must only include ['scheduled'|'active'|'completed'])."
            raise_error(KeyError, desc, self.curr_step)

        return self._scheduler.get_event_ids(event_statuses)

    def event_exists(self, event_id: str) -> str|None:
        """
        Get whether event with the given ID exists.
        
        Args:
            `event_id` (str): Event ID
        
        Returns:
            (str, optional):Returns `None` if it does not exist, otherwise status ['_scheduled_'|'_active_'|'_completed_']
        """

        if self._scheduler == None: return None
        else: return self._scheduler.get_event_status(event_id)

    def controller_exists(self, controller_id: str) -> str|None:
        """
        Get whether controller with ID controller_id exists.
        
        Args:
            `controller_id` (str): Controller ID
        
        Returns:
            (str, optional):Returns `None` if it does not exist, otherwise type ['_VSLController_'|'_RGController_']
        """

        if controller_id in self.controllers: return self.controllers[controller_id].__name__()
        else: return None

    def get_controller_ids(self, controller_types: str|list|tuple|None = None) -> list:
        """
        Return list of all controller IDs, or controllers of specified type ('VSLController' or 'RGController').
        
        Args:
            `controller_types` (str, list, tuple, optional):Controller type, defaults to all
        
        Returns:
            list: Controller IDs
        """

        valid_types = ["VSLController", "RGController"]
        if isinstance(controller_types, str): controller_types = [controller_types]
        elif controller_types == None: controller_types = valid_types
        elif not isinstance(controller_types, (list, tuple)):
            desc = "Invalid controller_types '{0}' type (must be [str|list|tuple], not '{1}').".format(controller_types, type(controller_types).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        if len(set(controller_types) - set(valid_types)) != 0:
            desc = "Invalid controller_types (must only include ['VSLController'|'RGController'])."
            raise_error(KeyError, desc, self.curr_step)

        return list([c_id for c_id, c in self.controllers.items() if c.__name__() in controller_types])

    def remove_controllers(self, controller_ids: str|list|tuple) -> None:
        """
        Remove controllers and delete their collected data.
        
        Args:
            `controller_ids` (str, list, tuple): Controller ID or list of IDs
        """

        if isinstance(controller_ids, str): controller_ids = [controller_ids]
        controller_ids = validate_list_types(controller_ids, str, param_name="controller_ids", curr_sim_step=self.curr_step)

        for controller_id in controller_ids:
            if self.controller_exists(controller_id) != None:
                if self.controllers[controller_id].activated:
                    self.controllers[controller_id].deactivate()
                del self.controllers[controller_id]
            else:
                desc = "Controller with ID '{0}' not found.".format(controller_id)
                raise_error(KeyError, desc, self.curr_step)

    def gui_track_vehicle(self, vehicle_id: str, view_id: str|None = None, highlight: bool = True) -> None:
        """
        Sets GUI view to track a vehicle by ID.

        Args:
            `vehicle_id` (str): Vehicle ID
            `view_id` (str, optional): View ID, if `None` uses default
        """
        
        if not self._gui:
            desc = f"Cannot take screenshot (GUI is not active)."
            raise_error(SimulationError, desc, self.curr_step)

        if not self.vehicle_exists(vehicle_id):
            desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)

        if view_id == None: view_id = self._default_view
        if highlight: self.set_vehicle_vals(vehicle_id, highlight=True)

        traci.gui.trackVehicle(view_id, vehicle_id)

        self._gui_veh_tracking[view_id] = vehicle_id

    def gui_stop_tracking(self, view_id: str|None) -> None:
        """
        Stops GUI view from tracking vehicle.

        Args:
            `view_id` (str, optional): View ID (defaults to default view)
        """

        if view_id == None: view_id = self._default_view

        if not self._gui:
            desc = f"Cannot stop vehicle tracking (GUI is not active)."
            raise_error(SimulationError, desc, self.curr_step)
        elif view_id not in self._gui_veh_tracking:
            desc = f"View ID '{view_id}' is not tracking a vehicle."
            raise_error(KeyError, desc, self.curr_step)

        if view_id not in self.get_gui_views():
            desc = f"View ID '{view_id}' not found."
            raise_error(KeyError, desc, self.curr_step)
        
        traci.gui.trackVehicle(view_id, "")

        del self._gui_veh_tracking[view_id]

    def gui_is_tracking(self, view_id: str|None = None) -> bool:
        """
        Returns whether a GUI view is tracking a vehicle.

        Args:
            `view_id` (str, optional): View ID (defaults to default view)
        """
        
        return view_id in self._gui_veh_tracking
    
    def add_gui_view(self, view_id: str, bounds: list|tuple|None = None, zoom: int|float|None = None) -> None:
        """
        Adds a new GUI view.
        
        Args:
            `view_id` (str): View ID
            `bounds` (list, tuple, optional): View bounds coordinates (lower-left, upper-right)
            `zoom` (int, float, optional): Zoom level
        """

        if view_id in self.get_gui_views():
            desc = f"View ID '{view_id}' already exists."
            raise_error(KeyError, desc, self.curr_step)
        
        traci.gui.addView(view_id)
        
        if bounds != None or zoom != None: self.set_view(view_id, bounds, zoom)
        self._gui_views.append(view_id)

    def remove_gui_view(self, view_id: str) -> None:
        """
        Removes a GUI view.
        
        Args:
            `view_id` (str): View ID
        """

        if view_id not in self.get_gui_views():
            desc = f"View ID '{view_id}' not found."
            raise_error(KeyError, desc, self.curr_step)

        elif view_id == self._default_view:
            desc = f"Cannot remove default view '{view_id}'."
            raise_error(ValueError, desc, self.curr_step)

        traci.gui.removeView(view_id)
        self._gui_views.remove(view_id)

    def get_gui_views(self) -> list:
        """
        Returns a list of all GUI view IDs.
        
        Returns:
            list: List of view IDs
        """

        return [self._default_view] + self._gui_views
    
    def get_view_boundaries(self, view_id: str|None = None) -> tuple:
        """
        Returns the boundaries of a view (defaults to default view).

        Args:
            `view_id` (str, optional): View ID (defaults to default view)

        Returns:
            tuple: View boundaries (lower-left, upper-right)
        """

        if not self._gui:
            desc = f"Cannot get view boundaries (GUI is not active)."
            raise_error(SimulationError, desc, self.curr_step)

        if view_id == None: view_id = self._default_view

        elif view_id not in self.get_gui_views():
            desc = f"View ID '{view_id}' not found."
            raise_error(KeyError, desc, self.curr_step)

        return traci.gui.getBoundary(view_id)
    
    def get_view_zoom(self, view_id: str|None = None) -> int:
        """
        Returns the zoom level of a view (defaults to default view).

        Args:
            `view_id` (str, optional): View ID (defaults to default view)

        Returns:
            int: Zoom level percent
        """

        if not self._gui:
            desc = f"Cannot get view boundaries (GUI is not active)."
            raise_error(SimulationError, desc, self.curr_step)

        if view_id == None: view_id = self._default_view

        elif view_id not in self.get_gui_views():
            desc = f"View ID '{view_id}' not found."
            raise_error(KeyError, desc, self.curr_step)

        return traci.gui.getZoom(view_id)
    
    def set_view(self, view_id: str|None = None, bounds: list|tuple|None = None, zoom: int|float|None = None) -> None:
        """
        Sets the bounds and/or zoom level of a GUI view.

        Args:
            `view_id` (str, optional): View ID (defaults to default view)
            `bounds` (list, tuple, optional): View bounds coordinates (lower-left, upper-right)
            `zoom` (int, float, optional): Zoom level
        """

        if not self._gui:
            desc = f"Cannot set view (GUI is not active)."
            raise_error(SimulationError, desc, self.curr_step)

        if view_id == None: view_id = self._default_view

        if bounds == None and zoom == None:
            desc = "Invalid bounds and zoom (both cannot be 'None')."
            raise_error(ValueError, desc, self.curr_step)
        
        if bounds != None:
            _ = validate_list_types(bounds, (tuple, tuple), True, "bounds", self.curr_step)
            traci.gui.setBoundary(view_id, bounds[0][0], bounds[0][1], bounds[1][0], bounds[1][1])
        if zoom != None: traci.gui.setZoom(view_id, zoom)

    def take_screenshot(self, filename: str, view_id: str|None = None, bounds: list|tuple|None = None, zoom: int|float|None = None) -> None:
        """
        Takes a screenshot of a GUI view and saves result to a file.

        Args:
            `filename` (str): Screenshot filename
            `view_id` (str, optional): View ID (defaults to default view)
            `bounds` (list, tuple, optional): View bounds coordinates (lower-left, upper-right) (defaults to current bounds)
            `zoom` (int, float, optional): Zoom level (defaults to current zoom)
        """

        if not self._gui:
            desc = f"Cannot take screenshot (GUI is not active)."
            raise_error(SimulationError, desc, self.curr_step)

        if view_id == None: view_id = self._default_view

        if bounds != None or zoom != None: self.set_view(view_id, bounds, zoom)

        if not filename.endswith(".png"): filename += ".png"

        traci.gui.screenshot(view_id, filename)

    def print_summary(self, save_file=None) -> None:
        """
        Prints a summary of a sim_data file or dictionary, listing
        simulation details, vehicle statistics, detectors, controllers,
        tracked edges/junctions and events. The summary can also be saved
        as a '_.txt_' file.
        
        Args:
            save_file: '_.txt_' filename, if given will be used to save summary
        """
        print_summary(self._all_data, save_file)

    def print_sim_data_struct(self) -> None:
        """
        Prints the structure of the current sim_data dictionary, with keys
        and data types for values. Lists/tuples are displayed at max 2D. '*'
        represents the maximum dimension value if the dimension size is inconsistent,
        and '+' denotes the array dimension is greater than 2.
        """
        
        print_sim_data_struct(self._all_data)

def print_sim_data_struct(sim_data: Simulation|dict|str) -> None:
    """
    Prints the structure of a sim_data dictionary, from a Simulation
    object, dict or filepath, with keys and data types for values. Lists/tuples
    are displayed at max 2D. '*' represents the maximum dimension value if
    the dimension size is inconsistent, and '+' denotes the array dimension is
    greater than 2.

    Args:
        `sim_data` (Simulation, dict, str): Either Simulation object, dictionary or sim_data filepath
    """
    
    sim_data = validate_type(sim_data, (str, dict, Simulation), "sim_data")
    if isinstance(sim_data, Simulation):
        dictionary = sim_data.__dict__()
    elif isinstance(sim_data, dict):
        dictionary = sim_data
    elif isinstance(sim_data, str):
        if sim_data.endswith(".json"): r_class, r_mode = json, "r"
        elif sim_data.endswith(".pkl"): r_class, r_mode = pkl, "rb"
        else:
            caller = "{0}()".format(inspect.stack()[0][3])
            desc = "{0}: sim_data file '{1}' is invalid (must be '.json' or '.pkl').".format(caller, sim_data)
            raise ValueError(desc)

        if os.path.exists(sim_data):
            with open(sim_data, r_mode) as fp:
                dictionary = r_class.load(fp)

    _print_dict({dictionary["scenario_name"]: dictionary})

def _print_dict(dictionary, indent=0, prev_indent="", prev_key=None):
    dict_keys = list(dictionary.keys())
    for key in dict_keys:
        val, is_last = dictionary[key], key == dict_keys[-1]
        curr_indent = _get_indent(indent, is_last, prev_indent)
        if isinstance(val, dict) and prev_key != "trips":
            print(curr_indent+key+":")
            _print_dict(val, indent+1, curr_indent, key)
        else:
            if isinstance(val, (list, tuple, dict)):
                type_str = type(val).__name__ + " " + _get_2d_shape(val)
            else:
                type_str = type(val).__name__
                
            print("{0}{1}: {2}".format(curr_indent, key, type_str))
        prev_indent = curr_indent
    
def _get_indent(indent_no, last, prev_indent, col_width=6):
    if indent_no == 0: return ""
    else:
        v_connector = "|{0}".format(" "*(col_width-1))
        end = "─- "

        connector = "└" if last else "├"
        indent_str = "  "+v_connector*(indent_no-1) + connector + end
        indent_arr, prev_indent = [*indent_str], [*prev_indent]

        for idx, _ in enumerate(indent_arr):
            if idx < len(prev_indent):
                prev_char = prev_indent[idx]
                if prev_char in [" ", "└"]: indent_arr[idx] = " "
                elif prev_char in ["|"]: indent_arr[idx] = "|"
                elif prev_char == "├" and indent_arr[idx] not in ["├", "└"]: indent_arr[idx] = "|"

        indent_str = "".join(indent_arr)
        return indent_str
    
def _get_2d_shape(array):

    x = len(array)
    arrs = []
    deeper = False
    for elem in array:
        if isinstance(elem, (list, tuple)):
            arrs.append(len(elem))
            deeper = deeper or True in [isinstance(elem2, (list, tuple)) for elem2 in elem]

    if len(arrs) > 0:
        if len(set(arrs)) == 1:
            return "({0}x{1})".format(x, arrs[0])
        else:
            return_str = "({0}x{1}*)".format(x, max(arrs))
            if deeper: return_str += "+"
            return return_str
    else: return "(1x{0})".format(x)

def _get_phase_string(curr_phases, masks):

    m_len = len(list(masks.values())[0])
    phase_arr = ['-'] * m_len
    
    for m_key in masks.keys():
        for idx in range(m_len):
            if masks[m_key][idx] == "1":
                phase_arr[idx] = curr_phases[m_key]

    phase_str = "".join(phase_arr)
    return phase_str

class TrackedJunction:
    """ Junction object with automatic data collection. """

    def __init__(self, junc_id: str, simulation: Simulation, junc_params: dict|str=None) -> None:
        """
        Args:
            `junc_id` (str): Junction ID
            `simulation` (Simulation): Simulation object
            `junc_params` (dict, str): Junction parameters dictionary or filepath
        """
        self.id = junc_id
        self.sim = simulation

        self.incoming_edges, self.outgoing_edges = [], []
        for e_id, e_info in self.sim._edge_info.items():
            if self.id == e_info["junction_ids"][0]: self.outgoing_edges.append(e_id)
            elif self.id == e_info["junction_ids"][1]: self.incoming_edges.append(e_id)

        self.position = traci.junction.getPosition(junc_id)
        
        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        self._track_flow = False
        self._measure_queues = False
        self._is_meter = False
        self._has_tl = junc_id in self.sim._all_tls

        self._init_params = junc_params

        if self._has_tl:
            state_str = traci.trafficlight.getRedYellowGreenState(junc_id)
            self._m_len = len(state_str)    
            self._durations = [[] for _ in range(self._m_len)]
            self._avg_green, self._avg_m_red = 0, 0
            self._avg_m_green, self._avg_m_red = [0 for _ in range(self._m_len)], [0 for _ in range(self._m_len)]

        if junc_params != None:
            junc_params = load_params(junc_params, "junc_params", step=self.curr_time)

            valid_params = {"flow_params": dict, "meter_params": dict}
            error, desc = test_input_dict(junc_params, valid_params, "'{0}' junction".format(self.id))
            if error != None: raise_error(error, desc, self.sim.curr_step)

            if "flow_params" in junc_params.keys():
                flow_params = junc_params["flow_params"]

                valid_params = {"inflow_detectors": (list, tuple), "outflow_detectors": (list, tuple), "vehicle_types": list}
                error, desc = test_input_dict(flow_params, valid_params, "'{0}' flow".format(self.id), required=["inflow_detectors", "outflow_detectors"])
                if error != None: raise_error(error, desc, self.sim.curr_step)

                if "inflow_detectors" in flow_params.keys() or "outflow_detectors" in flow_params.keys():
                    if not ("inflow_detectors" in flow_params.keys() and "outflow_detectors" in flow_params.keys()):
                        desc = f"Both 'inflow_detectors' and 'outflow_detectors' are required parameters to track flow (Junction ID: '{self.id}')."
                        raise_error(KeyError, desc, self.sim.curr_step)
                    else:

                        for detector_id in flow_params["inflow_detectors"]:
                            if detector_id not in self.sim.available_detectors.keys():
                                desc = f"Unrecognised detector ID '{detector_id}' given in inflow_detectors (Junction ID: '{self.id}')."
                                raise_error(KeyError, desc, self.sim.curr_step)
                        for detector_id in flow_params["outflow_detectors"]:
                            if detector_id not in self.sim.available_detectors.keys():
                                desc = f"Unrecognised detector ID '{detector_id}' given in outflow_detectors (Junction ID: '{self.id}')."
                                raise_error(KeyError, desc, self.sim.curr_step)

                        self.inflow_detectors = flow_params["inflow_detectors"]
                        self.outflow_detectors = flow_params["outflow_detectors"]

                    if "vehicle_types" in flow_params.keys(): self.flow_vtypes = ["all"] + flow_params["vehicle_types"]
                    else: self.flow_vtypes = ["all"]
                    
                    self._v_in, self._v_out = {vehicle_type: [] for vehicle_type in self.flow_vtypes}, {vehicle_type: [] for vehicle_type in self.flow_vtypes}
                    self._inflows, self._outflows = {vehicle_type: [] for vehicle_type in self.flow_vtypes}, {vehicle_type: [] for vehicle_type in self.flow_vtypes}

                    self._track_flow = True

            if "meter_params" in junc_params.keys():
                meter_params = junc_params["meter_params"]

                valid_params = {"min_rate": (int, float), "max_rate": (int, float), "ramp_edges": (list, tuple),
                                "queue_detector": str, "init_rate": (int, float), "max_queue": int}
                error, desc = test_input_dict(meter_params, valid_params, "'{0}' meter".format(self.id), required=["min_rate", "max_rate"])
                if error != None: raise_error(error, desc, self.sim.curr_step)

                self._is_meter = True
                self.min_rate, self.max_rate = meter_params["min_rate"], meter_params["max_rate"]

                self._metering_rates = []
                self._rate_times = []

                self.queue_detector = None
                self.ramp_edges = None
                self.max_queue = None

                self._queue_lengths, self._queue_delays = [], None

                if "ramp_edges" in meter_params.keys():
                    ramp_edges = meter_params["ramp_edges"]
                    self._ramp_length = 0
                    if not isinstance(ramp_edges, (list, tuple)): ramp_edges = [ramp_edges]
                    validate_list_types(ramp_edges, str, "ramp_edges", self.sim.curr_step)
                    self.ramp_edges = ramp_edges
                    for edge in self.ramp_edges:
                        if edge not in self.sim._all_edges:
                            desc = "Edge ID '{0}' not found.".format(edge)
                            raise_error(KeyError, desc, self.sim.curr_step)
                        else: self._ramp_length += self.sim.get_geometry_vals(edge, "length")

                    self._measure_queues, self.queue_detector = True, None

                    self._queue_delays = []

                if not self._measure_queues:
                    if "queue_detector" in meter_params.keys():
                        self._measure_queues, self.queue_detector = True, meter_params["queue_detector"]

                        if self.queue_detector not in self.sim.available_detectors.keys():
                            desc = f"Unrecognised detector ID given as queue_detector ('{self.queue_detector}')."
                            raise_error(KeyError, desc, self.sim.curr_step)
                        elif self.sim.available_detectors[self.queue_detector]["type"] != "multientryexit":
                            desc = "Only 'multientryexit' detectors can be used to find queue length (not '{0}').".format(self.sim.available_detectors[self.queue_detector]["type"])
                            raise_error(ValueError, desc, self.sim.curr_step)

                if "init_rate" in meter_params.keys(): self.sim.set_tl_metering_rate(self.id, meter_params["init_rate"])
                else: self.sim.set_tl_metering_rate(self.id, self.max_rate)

                if "max_queue" in meter_params.keys(): self.max_queue = meter_params["max_queue"]

            else: self._is_meter = False
    
    def __str__(self): return f"<{self.__name__()}: '{self.id}'>"
    def __name__(self): return "TrackedJunction"

    def __dict__(self) -> dict:

        junc_dict = {"position": self.position, "incoming_edges": self.incoming_edges, "outgoing_edges": self.outgoing_edges,
                     "init_time": self.init_time, "curr_time": self.curr_time}
        
        if self._has_tl: junc_dict["tl"] = {"m_len": self._m_len, "avg_green": self._avg_green, "avg_red": self._avg_red,
                                           "avg_m_green": self._avg_m_green, "avg_m_red": self._avg_m_red, "m_phases": self._durations}

        if self._track_flow:
            junc_dict["flows"] = {"inflow_detectors": self.inflow_detectors, "outflow_detectors": self.outflow_detectors,
                                 "all_inflows": self._inflows, "all_outflows": self._outflows}
            
        if self._is_meter:
            junc_dict["meter"] = {"metering_rates": self._metering_rates, "rate_times": self._rate_times}
            
            if self._measure_queues:
                junc_dict["meter"]["queue_lengths"] = self._queue_lengths
                if self.ramp_edges != None: junc_dict["meter"]["queue_delays"] = self._queue_delays

            if self.max_queue != None: junc_dict["meter"]["max_queue"] = self.max_queue

            junc_dict["meter"]["min_rate"] = self.min_rate
            junc_dict["meter"]["max_rate"] = self.max_rate

        return junc_dict

    def reset(self) -> None:
        """ Resets junction data collection. """
        
        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        if self._has_tl:
            self._durations = [[] for _ in range(self._m_len)]
            self._avg_green, self._avg_red = 0, 0
            self._avg_m_green, self._avg_m_red = [0 for _ in range(self._m_len)], [0 for _ in range(self._m_len)]

        if self._track_flow:
            self._v_in, self._v_out = {vehicle_type: [] for vehicle_type in self.flow_vtypes}, {vehicle_type: [] for vehicle_type in self.flow_vtypes}
            self._inflows, self._outflows = {vehicle_type: [] for vehicle_type in self.flow_vtypes}, {vehicle_type: [] for vehicle_type in self.flow_vtypes}

        if self._is_meter:
            self._metering_rates = []
            self._rate_times = []

            if self._measure_queues:
                self._queue_lengths = []
                if self.ramp_edges != None: self._queue_delays = []

    def update(self, keep_data: bool = True) -> None:
        """
        Update junction object for the current time step.

        Args:
            `keep_data` (bool): Denotes whether to update junction data
        """

        self.curr_time = self.sim.curr_step
        
        if keep_data:
            if self._has_tl:
                curr_state = traci.trafficlight.getRedYellowGreenState(self.id)
                colours = [*curr_state]
                for idx, mc in enumerate(colours):
                    
                    # Phase duration in steps (not seconds)
                    if len(self._durations[idx]) == 0 or mc.upper() != self._durations[idx][-1][0]:
                        self._durations[idx].append([mc.upper(), 1])
                    elif mc.upper() == self._durations[idx][-1][0]:
                        self._durations[idx][-1][1] = self._durations[idx][-1][1] + 1

                    if mc.upper() == 'G':
                        m_green_durs = [val[1] for val in self._durations[idx] if val[0] == 'G']
                        self._avg_m_green[idx] = sum(m_green_durs) / len(m_green_durs)
                    elif mc.upper() == 'R':
                        m_red_durs = [val[1] for val in self._durations[idx] if val[0] == 'R']
                        self._avg_m_red[idx] = sum(m_red_durs) / len(m_red_durs)

                self._avg_green = sum(self._avg_m_green) / len(self._avg_m_green)
                self._avg_red = sum(self._avg_m_red) / len(self._avg_m_red)

            if self._track_flow:
                
                for vehicle_type in self.flow_vtypes:
                    new_v_in = self.sim.get_last_step_detector_vehicles(self.inflow_detectors, vehicle_types=[vehicle_type] if vehicle_type != "all" else None, flatten=True)
                    new_v_in = list(set(new_v_in) - set(self._v_in[vehicle_type]))
                    self._v_in[vehicle_type] += new_v_in
                    self._inflows[vehicle_type].append(len(new_v_in))

                for vehicle_type in self.flow_vtypes:
                    new_v_out = self.sim.get_last_step_detector_vehicles(self.outflow_detectors, vehicle_types=[vehicle_type] if vehicle_type != "all" else None, flatten=True)
                    new_v_out = list(set(new_v_out) - set(self._v_out[vehicle_type]))
                    self._v_out[vehicle_type] += new_v_out
                    self._outflows[vehicle_type].append(len(new_v_out))

            if self._measure_queues:

                if self.ramp_edges != None:
                    queuing_vehicles = self.sim.get_last_step_geometry_vehicles(self.ramp_edges, flatten=True)
                    queuing_vehicles = [veh_id for veh_id in queuing_vehicles if self.sim.vehicle_exists(veh_id)]
                    vehicle_data = self.sim.get_vehicle_vals(queuing_vehicles, ["speed", "allowed_speed"])

                    if len(queuing_vehicles) > 1:
                        speeds = [data["speed"] for data in vehicle_data.values()]
                        allowed_speeds = [data["allowed_speed"] for data in vehicle_data.values()]
                    elif len(queuing_vehicles) == 1:
                        speeds = [vehicle_data["speed"]]
                        allowed_speeds = [vehicle_data["allowed_speed"]]
                    else: speeds, allowed_speeds = [], []

                    self._queue_lengths.append(len([veh_id for veh_id in queuing_vehicles if self.sim.get_vehicle_vals(veh_id, "is_stopped")]))

                    if len(queuing_vehicles) > 0:
                        avg_speed = sum(speeds) / len(speeds)
                        ff_speed = sum(allowed_speeds) / len(allowed_speeds)

                        if avg_speed == 0: self._queue_delays.append(len(speeds))
                        elif avg_speed >= ff_speed: self._queue_delays.append(0)
                        else:
                            ramp_length = convert_units(self._ramp_length, self.sim._l_dist_unit, "metres")
                            flow = (avg_speed * (len(queuing_vehicles) / ramp_length)) * self.sim.step_length
                            self._queue_delays.append(flow * ((ramp_length / avg_speed) - (ramp_length / ff_speed)))

                    else: self._queue_delays.append(0)
                
                elif self.queue_detector != None:
                    queuing_vehicles = self.sim.get_last_step_detector_vehicles(self.queue_detector, flatten=True)
                    queuing_vehicles = [veh_id for veh_id in queuing_vehicles if self.sim.vehicle_exists(veh_id)]
                    num_stopped = len([veh_id for veh_id in queuing_vehicles if self.sim.get_vehicle_vals(veh_id, "is_stopped")])
                    self._queue_lengths.append(num_stopped)

                else:
                    desc = f"Cannot update meter '{self.id}' queue length (no detector or entry/exit edges)"
                    raise_error(KeyError, desc, self.sim.curr_step)

    def set_metering_rate(self, metering_rate: int|float, g_time: int|float = 1, y_time: int|float = 1, min_red: int|float = 1, vehs_per_cycle: int|None = None, control_interval: int|float = 60):
        """
        Set ramp metering rate of a meter at a junction. Uses a one-car-per-green policy with a default
        1s green and yellow time, with red phase duration changed to set flow. All phase durations must
        be larger than the simulation step length.
        
        Args:
            `metering_rate` (int, float): On-ramp inflow in veh/hr (from all lanes)
            `g_time` (int, float): Green phase duration (s), defaults to 1
            `y_time` (int, float): Yellow phase duration (s), defaults to 1
            `min_red` (int, float): Minimum red phase duration (s), defaults to 1
            `vehs_per_cycle` (int, optional):Number of vehicles released with each cycle, defaults to the number of lanes
            `control_interval` (int, float): Ramp meter control interval (s)
        """

        if not self._is_meter:
            desc = f"Cannot set metering rate (Junction ID '{self.id}' is not a metered junction)."
            raise_error(SimulationError, desc, self.sim.curr_step)
        
        self.sim.set_tl_metering_rate(self.id, metering_rate, g_time, y_time, min_red, vehs_per_cycle, control_interval)

class TrackedEdge:
    """ Edge object with automatic data collection. """

    def __init__(self, edge_id: str, simulation: Simulation) -> None:
        """
        Args:
            `edge_id` (str): Edge ID
            `simulation` (Simulation): Simulation object
        """
        self.id = edge_id
        self.sim = simulation

        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        self.linestring = self.sim._edge_info[self.id]["linestring"]
        self.length = self.sim._edge_info[self.id]["length"]
        self.from_node, self.to_node = self.sim._edge_info[self.id]["junction_ids"]

        self.n_lanes = self.sim.get_geometry_vals(self.id, "n_lanes")

        self.step_vehicles = []
        self.flows = []
        self.speeds = []
        self.densities = []
        self.occupancies = []

        self.sim.add_geometry_subscriptions(self.id, "vehicle_ids")

    def __str__(self): return "<{0}: '{1}'>".format(self.__name__, self.id)
    def __name__(self): return "TrackedEdge"

    def __dict__(self) -> dict:

        edge_dict = {"linestring": self.linestring,
                     "length": self.length,
                     "to_node": self.to_node,
                     "from_node": self.from_node,
                     "n_lanes": self.n_lanes,
                     "step_vehicles": self.step_vehicles,
                     "flows": self.flows,
                     "speeds": self.speeds,
                     "densities": self.densities,
                     "occupancies": self.occupancies,
                     "init_time": self.init_time,
                     "curr_time": self.curr_time}
        
        return edge_dict

    def reset(self) -> None:
        """ Resets edge data collection. """

        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        self.step_vehicles = []
        self.flows = []
        self.speeds = []
        self.densities = []
        self.occupancies = []

    def update(self, keep_data: bool = True) -> None:
        """
        Update edge object for the current time step.

        Args:
            `keep_data` (bool): Denotes whether to update edge data
        """
        
        self.curr_time = self.sim.curr_step
        if keep_data:
            last_step_vehs = self.sim.get_last_step_geometry_vehicles(self.id, flatten=True)

            veh_data, total_speed = [], 0
            for veh_id in last_step_vehs:
                vals = self.sim.get_vehicle_vals(veh_id, ["speed", "position", "lane_idx"])
                pos = self._get_distance_on_road(vals["position"])
                if self.sim.units in ['IMPERIAL']: pos *= 0.0006213712
                veh_data.append((veh_id, pos, vals["speed"], vals["lane_idx"]))
                total_speed += vals["speed"]
            
            self.step_vehicles.append(veh_data)

            n_vehicles = len(veh_data)

            occupancy = self.sim.get_geometry_vals(self.id, "vehicle_occupancy")
            speed = -1 if n_vehicles == 0 else total_speed / n_vehicles
            if speed != -1 and self.sim.units == "UK": speed = convert_units(speed, "mph", "kmph")
            density = -1 if n_vehicles == 0 else n_vehicles / self.length
            flow = -1 if n_vehicles == 0 else speed * density

            self.flows.append(flow)
            self.speeds.append(speed)
            self.densities.append(density)
            self.occupancies.append(occupancy)
            
    def _get_distance_on_road(self, veh_coors):
        line = LineString(self.linestring)
        p = Point(veh_coors)
        p2 = line.interpolate(line.project(p))
        x_val = line.line_locate_point(p2, False)
        x_pct = x_val/line.length
        return x_pct

def print_summary(sim_data: dict|str, save_file: str|None=None, tab_width: int=58):
    """
    Prints a summary of a sim_data file or dictionary, listing
    simulation details, vehicle statistics, detectors, controllers,
    tracked edges/junctions and events.
    
    Args:
        `sim_data` (dict, str):  Simulation data dictionary or filepath
        `save_file` (str, optional):'_.txt_' filename, if given will be used to save summary
        `tab_width` (int): Table width
    """
    caller = "{0}()".format(inspect.stack()[0][3])
    if save_file != None:
        save_file = validate_type(save_file, str, "save_file")
        if not save_file.endswith(".txt"): save_file += ".txt"
    old_stdout = sys.stdout
    sys.stdout = buffer = io.StringIO()
    
    sim_data = validate_type(sim_data, (str, dict), "sim_data")
    if isinstance(sim_data, str):
        if sim_data.endswith(".json"): r_class, r_mode = json, "r"
        elif sim_data.endswith(".pkl"): r_class, r_mode = pkl, "rb"
        else:
            desc = f"{caller}: sim_data file '{sim_data}' is invalid (must be '.json' or '.pkl')."
            raise ValueError(desc)

        if os.path.exists(sim_data):
            with open(sim_data, r_mode) as fp:
                sim_data = r_class.load(fp)
        else:
            desc = "{0}: sim_data file '{1}' not found.".format(caller, sim_data)
            raise FileNotFoundError(desc)
    elif len(sim_data.keys()) == 0 or "data" not in sim_data.keys():
        desc = "{0}: Invalid sim_data (no data found)."
        raise ValueError(desc)
    
    name = sim_data["scenario_name"]
    if math.floor((tab_width-len(name))/2) != math.ceil((tab_width-len(name))/2):
        tab_width += 1
    
    primary_delineator = " *"+"="*(tab_width+2)+"*"
    secondary_delineator = " *"+"-"*(tab_width+2)+"*"
    tertiary_delineator = " * "+"-"*tab_width+" *"
    
    print(primary_delineator)
    _table_print("TUD-SUMO v{0}".format(sim_data["tuds_version"]), tab_width)

    print(primary_delineator)
    _table_print(sim_data["scenario_name"], tab_width)
    if "scenario_desc" in sim_data.keys():
        desc = sim_data["scenario_desc"]
        print(primary_delineator)
        if tab_width - len("Description: "+desc) > 0:
            _table_print("Description: "+desc, tab_width)
        else:
            _table_print("Description:", tab_width)
            desc_lines = _add_linebreaks(desc, tab_width)
            for line in desc_lines: _table_print(line, tab_width)
    print(primary_delineator)
    
    start_step, end_step = sim_data["start"], sim_data["end"]
    start_time, end_time = datetime.strptime(sim_data["sim_start"], datetime_format), datetime.strptime(sim_data["sim_end"], datetime_format)
    sim_duration, sim_duration_steps = end_time - start_time, end_step - start_step
    if start_time.date() == end_time.date():
        _table_print(f"Simulation Run: {start_time.strftime(date_format)}", tab_width)
        _table_print(f"{start_time.strftime(time_format)} - {end_time.strftime(time_format)} ({sim_duration})", tab_width)
    else:
        _table_print(f"Simulation Run: ({sim_duration})", tab_width)
        _table_print([start_time.strftime(date_format), end_time.strftime(date_format)], tab_width, centre_cols=True)
        _table_print([start_time.strftime(time_format), end_time.strftime(time_format)], tab_width, centre_cols=True)
    
    print(secondary_delineator)
    _table_print(["Number of Steps:", f"{sim_duration_steps} ({start_step}-{end_step})"], tab_width)
    _table_print(["Step Length:", f"{sim_data['step_len']}s"], tab_width)
    _table_print(["Avg. Step Duration:", f"{sim_duration.total_seconds() / sim_duration_steps}s"], tab_width)
    _table_print(["Units Type:", unit_desc[sim_data["units"]]], tab_width)
    _table_print(["Seed:", sim_data["seed"]], tab_width)
    
    print(primary_delineator)
    _table_print("Data", tab_width)
    print(primary_delineator)
    _table_print("Vehicle Data", tab_width)
    print(secondary_delineator)

    labels = {"no_vehicles": ["No. Vehicles", "Overall TTS"],
              "no_waiting": ["No. Waiting Vehicles", "Overall TWT"],
              "delay": ["Vehicle Delay", "Cumulative Delay"],
              "to_depart": ["Vehicles to Depart", None]}

    for key, label in labels.items():
        data = sim_data["data"]["vehicles"][key]
        unit = "s" if key == "delay" else ""
        _table_print(f"{label[0]}:", tab_width)
        _table_print(["Average:", f"{round(sum(data)/len(data), 2)}{unit}"], tab_width)
        _table_print(["Peak:", f"{round(max(data), 2)}{unit}"], tab_width)
        _table_print(["Final:", f"{round(data[-1], 2)}{unit}"], tab_width)
        if label[1] != None: _table_print([f"{label[1]}:", f"{round(sum(data), 2)}s"], tab_width)
        print(tertiary_delineator)

    _table_print(["Individual Data:", "Yes" if "all_vehicles" in sim_data["data"].keys() else "No"], tab_width)

    print(secondary_delineator)
    _table_print("Trip Data", tab_width)
    print(secondary_delineator)
    n_inc, n_com = len(sim_data["data"]["trips"]["incomplete"]), len(sim_data["data"]["trips"]["completed"])
    _table_print(["Incomplete Trips:", f"{n_inc} ({round(100 * n_inc / (n_inc + n_com), 2)}%)"], tab_width)
    _table_print(["Completed Trips:", f"{n_com} ({round(100 * n_com / (n_inc + n_com), 1)}%)"], tab_width)

    print(secondary_delineator)
    _table_print("Detectors", tab_width)
    print(secondary_delineator)
    if "detectors" not in sim_data["data"].keys() or len(sim_data["data"]["detectors"]) == 0:
        _table_print("No detectors found.")
    else:
        ils, mees, unknown, labels = [], [], [], ["Induction Loop Detectors", "Multi-Entry-Exit Detectors", "Unknown Type"]
        for det_id, det_info in sim_data["data"]["detectors"].items():
            if det_info["type"] == "inductionloop": ils.append(det_id)
            elif det_info["type"] == "multientryexit": mees.append(det_id)
            else: unknown.append(det_id)
        
        add_spacing = False
        for ids, label in zip([ils, mees, unknown], labels):
            if len(ids) > 0:
                if add_spacing: _table_print(tab_width=tab_width)
                _table_print(label+": ({0})".format(len(ids)), tab_width)
                id_lines = _add_linebreaks(", ".join(ids), tab_width)
                for line in id_lines: _table_print(line, tab_width)
                add_spacing = True
 
    print(secondary_delineator)
    _table_print("Tracked Edges", tab_width)
    print(secondary_delineator)
    if "edges" not in sim_data["data"].keys() or len(sim_data["data"]["edges"]) == 0:
        _table_print("No tracked edges found.")
    else:
        id_lines = _add_linebreaks(", ".join(sim_data["data"]["edges"].keys()), tab_width)
        for line in id_lines: _table_print(line, tab_width)
        
    print(secondary_delineator)
    _table_print("Tracked Junctions", tab_width)
    print(secondary_delineator)
    if "junctions" not in sim_data["data"].keys() or len(sim_data["data"]["junctions"]) == 0:
        _table_print("No tracked junctions found.")
    else:
        for junc_id, junc_info in sim_data["data"]["junctions"].items():
            j_arr = []
            if "tl" in junc_info.keys(): j_arr.append("Signalised")
            if "meter" in junc_info.keys(): j_arr.append("Metered")
            if len(j_arr) == 0: j_arr.append("Uncontrolled")
            _table_print("{0} ({1})".format(junc_id, ", ".join(j_arr)), tab_width)

    if "controllers" in sim_data["data"].keys():
        print(secondary_delineator)
        _table_print("Controllers", tab_width)
        print(secondary_delineator)

        rgs, vsls, unknown, labels = [], [], [], ["Route Guidance", "Variable Speed Limits", "Unknown Type"]
        for cont_id, cont_info in sim_data["data"]["controllers"].items():
            if cont_info["type"] == "RG": rgs.append(cont_id)
            elif cont_info["type"] == "VSL": vsls.append(cont_id)
            else: unknown.append(cont_id)
        
        add_spacing = False
        for ids, label in zip([rgs, vsls, unknown], labels):
            if len(ids) > 0:
                if add_spacing: _table_print(tab_width=tab_width)
                _table_print(label+": ({0})".format(len(ids)), tab_width)
                id_lines = _add_linebreaks(", ".join(ids), tab_width)
                for line in id_lines: _table_print(line, tab_width)
                add_spacing = True
        

    if "events" in sim_data["data"].keys():
        event_statuses = []
        for s in ["scheduled", "active", "completed"]:
            if s in sim_data["data"]["events"].keys() and len(sim_data["data"]["events"][s]) > 0:
                event_statuses.append(s)

        if len(event_statuses) >= 0:
            print(secondary_delineator)
            _table_print("Event IDs & Statuses", tab_width)
            print(secondary_delineator)

            for event_status in event_statuses:
                event_ids = list(sim_data["data"]["events"][event_status].keys())
                event_str = "{0}: {1}".format(event_status.title(), ", ".join(event_ids))
                event_lines = _add_linebreaks(event_str, tab_width)
                for line in event_lines: _table_print(line, tab_width)

    print(secondary_delineator)
    
    sys.stdout = old_stdout
    summary = buffer.getvalue()

    if save_file != None:
        with open(save_file, "w") as fp:
            fp.write(summary)
    else:
        print(summary)

def _table_print(strings=None, tab_width=58, side=" | ", padding=" ", centre_cols=False):
    
    if strings == None: strings = ""
    print_str = None
    if isinstance(strings, str):
        print_str = side+_centre_str(strings, tab_width)+side
    elif isinstance(strings, (list, tuple)):
        if centre_cols:
            col_spacing = padding*math.floor((tab_width - sum([len(str(string)) for string in strings])) / (len(strings) + 1))
            print_str = col_spacing+col_spacing.join([str(string) for string in strings])+col_spacing
            print_str = side+_centre_str(print_str, tab_width)+side
        else:
            col_spacing = padding*math.floor((tab_width - sum([len(str(string)) for string in strings])) / (len(strings) - 1))
            print_str = side+col_spacing.join([str(string) for string in strings])+side
    else:
        desc = "_table_print(): Invalid type (must be [str|list|tuple], not '{0}').".format(type(strings).__name__)
        raise TypeError(desc)
    
    if print_str != None:
        print(print_str)
    else: exit()

def _centre_str(string, width, padding=" "):
    if len(string) > width: return string
    else: return padding*math.floor((width-len(string))/2)+string+padding*math.ceil((width-len(string))/2)

def _add_linebreaks(string, width):
    lines = []
    while string != "":
        if " " not in string or len(string) < width:
            lines.append(string)
            break
        elif " " in string and len(string) > width:
            max_len_str = string[:width]
            line_break_index = max_len_str.rfind(" ")
            lines.append(max_len_str[:line_break_index])
            string = string[line_break_index+1:]
    return lines

class SimulationError(Exception):
    pass