import os, sys, traci, sumolib
import numpy as np
from tqdm import tqdm
from random import seed as set_seed
from .events import EventScheduler, Event, _cause_incident, _add_weather
from .demand import _add_demand_vehicles
from shapely.geometry import LineString
from .helpers import print_sim_data_struct, print_summary
from .utils import *

from . import _io, _get_set_funcs, _vehicles, _traffic_signals, _gui, objects, controllers, _routing, _subscriptions

class Simulation:
    """ Main simulation interface."""

    def __init__(self, scenario_name: str | None = None, scenario_desc: str | None = None, *, verbose: bool = True) -> None:
        """
        Args:
            `scenario_name` (str, optional): Scenario label saved to simulation object (defaults to name of '_.sumocfg_')
            `scenario_desc` (str, optional): Simulation scenario description, saved along with all files
            `verbose` (bool): Denotes whether to print simulation information
        """

        # Should ideally be changed to use @dataclass for groups of variables (particularly hidden ones) 

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

        self._last_step_delay = {}
        self._last_step_flow = {}
        self._last_step_density = {}
        self._lane_to_edges = {}

        self._include_insertion_delay = False

        self._get_fc_data = True
        self._all_curr_vehicle_ids = set([])
        self._all_loaded_vehicle_ids = set([])
        self._all_added_vehicles = set([])
        self._all_removed_vehicles = set([])
        self._vehicle_types = set([])
        self._added_vehicle_types = set([])
        self._known_vehicles = {}
        self._stopped_vehicles = set([])
        self._trips = {"incomplete": {}, "completed": {}}

        self._v_in_funcs = []
        self._v_out_funcs = []
        self._v_func_params = {}
        self._valid_v_func_params = ["simulation", "curr_step", "vehicle_id", "route_id",
                                     "vehicle_type", "departure", "origin", "destination"] 
        
        self._default_view = 'View #0'
        self._gui_views = []
        self._gui_veh_tracking = None
        self._recorder = None

        self._weather_events = set([])

        self._closed_lanes = None

        from .__init__ import __version__
        self._tuds_version = __version__

    def __str__(self):
        if self.scenario_name != None:
            return "<{0}: '{1}'>".format(self.__name__, self.scenario_name)
        else: return "<{0}>".format(self.__name__)

    def __name__(self): return "Simulation"

    def __dict__(self): return {} if self._all_data == None else self._all_data


    # --- SIMULATION ---

    def start(self, 
              config_file: str | None = None,
              *, 
              net_file: str | None = None,
              route_file: str | None = None,
              add_file: str | None = None,
              gui_file: str | None = None,
              cmd_options: list | None = None,
              units: str | int = 1,
              get_fc_data: bool = True,
              include_insertion_delay: bool = False,
              automatic_subscriptions: bool = True,
              suppress_warnings: bool = False,
              suppress_traci_warnings: bool = True,
              suppress_pbar: bool = False,
              seed: str = "random",
              gui: bool = False,
              sumo_home: str | None = None
             ) -> None:
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
            `get_fc_data` (bool): Denotes whether to collect floating car data (set to `False` to improve performance)
            `include_insertion_delay` (bool): Denotes whether to include insertion delay (delay of vehicles waiting to be inserted into the simulation) in network-wide delay calculations
            `automatic_subscriptions` (bool): Denotes whether to automatically subscribe to commonly used vehicle data (speed and position, defaults to `True`)
            `suppress_warnings` (bool): Suppress simulation warnings
            `suppress_traci_warnings` (bool): Suppress warnings from TraCI
            `suppress_pbar` (bool): Suppress automatic progress bar when not using the GUI
            `seed` (bool): Either int to be used as seed, or `random.random()`/`random.randint()`, where a random seed is used
            `gui` (bool): Bool denoting whether to run GUI
            `sumo_home` (str, optional): SUMO base directory, if the `$SUMO_HOME` variable is not already set within the environment
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

        if self._junc_phases != None: _traffic_signals._update_lights(self)

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

        self._get_fc_data = get_fc_data
        self._include_insertion_delay = include_insertion_delay
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

    def step_through(self,
                     n_steps: int | None = None,
                     *,
                     end_step: int | None = None,
                     n_seconds: int | None = None,
                     vehicle_types: list | tuple | None = None,
                     keep_data: bool = True,
                     append_data: bool = True,
                     pbar_max_steps: int | None = None
                    ) -> dict:
        """
        Step through simulation from the current time until end_step, aggregating data during this period.
        
        Args:
            `n_steps` (int, optional): Perform n steps of the simulation (defaults to 1)
            `end_step` (int, optional): End point for stepping through simulation (given instead of `n_steps` or `n_seconds`)
            `n_seconds` (int, optional): Simulation duration in seconds (given instead of `n_steps` or `end_step`)
            `vehicle_types` (list, tuple, optional): Vehicle type(s) to collect data of (type ID or list of IDs, defaults to all)
            `keep_data` (bool): Denotes whether to store and process data collected during this run (defaults to `True`)
            `append_data` (bool): Denotes whether to append simulation data to that of previous runs or overwrite previous data (defaults to `True`)
            `pbar_max_steps` (int, optional): Max value for progress bar (persistent across calls) (negative values remove the progress bar)
        
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
                if self._get_fc_data: all_data["data"]["fc_data"] = []
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

            last_step_data, all_v_data = self._step(vehicle_types=vehicle_types, keep_data=keep_data)

            if keep_data:
                if self._get_fc_data: all_data["data"]["fc_data"].append(all_v_data)

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
            
    def _step(self, *, vehicle_types: list | None = None, keep_data: bool = True) -> dict:
        """
        Increment simulation by one time step, updating light state. Use `Simulation.step_through()` to run the simulation.
        
        Args:
            `vehicle_types` (list, optional): Vehicle type(s) to collect data of (type ID or list of IDs, defaults to all)
            `keep_data` (bool): Denotes whether to store and process data collected during this run (defaults to `True`)

        Returns:
            dict: Simulation data
        """

        self.curr_step += 1
        self.curr_time += self.step_length

        # First, implement the demand in the demand table (if exists)
        if self._manual_flow:
            _add_demand_vehicles(self)

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
            if self._get_fc_data: subscriptions += ["acceleration", "heading", "altitude"]
            else: subscriptions.append("position")

            self.add_vehicle_subscriptions(list(all_vehicles_in), subscriptions)

        # Process all vehicles entering/exiting the simulation
        _vehicles._vehicles_in(self, all_vehicles_in)
        _vehicles._vehicles_out(self, all_vehicles_out)

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

            _traffic_signals._update_lights(self, update_junc_lights)

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

            total_data, all_v_data = _get_set_funcs._get_all_vehicle_data(self, vehicle_types=vehicle_types)
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

    def reset_data(self, *, reset_juncs: bool = True, reset_edges: bool = True, reset_controllers: bool = True, reset_trips: bool = True) -> None:
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

    
    # --- SUBSCRIPTIONS ---

    def add_vehicle_subscriptions(self, vehicle_ids: str | list | tuple, data_keys: str | list | tuple) -> None:
        """
        Creates a new subscription for certain variables for **specific vehicles**. Valid data keys are;
        '_speed_', '_is_stopped_', '_max_speed_', '_acceleration_', '_position_', '_altitude_', '_heading_',
        '_edge_id_', '_lane_idx_', '_route_id_', '_route_idx_', '_route_edges_'.

        Args:
            `vehicle_ids` (str, list, tuple): Vehicle ID or list of IDs
            `data_keys` (str, list, tuple): Data key or list of keys
        """

        _subscriptions._add_vehicle_subscriptions(self, vehicle_ids, data_keys)

    def remove_vehicle_subscriptions(self, vehicle_ids: str | list | tuple) -> None:
        """
        Remove **all** active subscriptions for a vehicle or list of vehicles.
        
        Args:
            `vehicle_ids` (str, list, tuple): Vehicle ID or list of IDs
        """

        _subscriptions._remove_vehicle_subscriptions(self, vehicle_ids)
    
    def add_detector_subscriptions(self, detector_ids: str | list | tuple, data_keys: str | list | tuple) -> None:
        """
        Creates a new subscription for certain variables for **specific detectors**. Valid data keys are;
        '_vehicle_count_', '_vehicle_ids_', '_speed_', '_halting_no_', '_occupancy_', '_last_detection_'.
        
        Args:
            `detector_id` (str, list, tuple): Detector ID or list of IDs
            `data_keys` (str, list, tuple): Data key or list of keys
        """

        _subscriptions._add_detector_subscriptions(self, detector_ids, data_keys)

    def remove_detector_subscriptions(self, detector_ids: str | list | tuple) -> None:
        """
        Remove **all** active subscriptions for a detector or list of detectors.
        
        Args:
            `detector_ids` (str, list, tuple): Detector ID or list of IDs
        """

        _subscriptions._remove_detector_subscriptions(self, detector_ids)
    
    def add_geometry_subscriptions(self, geometry_ids: str | list | tuple, data_keys: str | list | tuple) -> None:
        """
        Creates a new subscription for geometry (edge/lane) variables. Valid data keys are;
        '_vehicle_count_', '_vehicle_ids_', '_vehicle_speed_', '_halting_no_', '_occupancy_'.
        
        Args:
            `geometry_ids` (str, list, tuple): Geometry ID or list of IDs
            `data_keys` (str, list, tuple): Data key or list of keys
        """

        _subscriptions._add_geometry_subscriptions(self, geometry_ids, data_keys)

    def remove_geometry_subscriptions(self, geometry_ids: str | list | tuple) -> None:
        """
        Remove **all** active subscriptions for a geometry object or list of geometry objects.
        
        Args:
            `geometry_ids` (str, list, tuple): Geometry ID or list of IDs
        """

        _subscriptions._remove_detector_subscriptions(self, geometry_ids)


    # --- INPUT/OUTPUT ---

    def save_objects(self, filename: str | None = None, *, overwrite: bool = True, json_indent: int = 4) -> None:
        """
        Save parameters of all TUD-SUMO objects created for the simulation (tracked edges/junctions, phases, controllers, events, demand, routes).

        Args:
            `filename` (str, optional): Output filename ('_.json_' or '_.pkl_')
            `overwrite` (bool): Denotes whether to allow overwriting previous outputs
            `json_indent` (int, optional): Indent used when saving JSON files
        """

        _io._save_objects(self, filename, overwrite, json_indent)

    def load_objects(self, object_parameters: str | dict) -> None:
        """
        Load parameters of all TUD-SUMO objects created for the simulation (tracked edges/junctions, phases, controllers, events, demand, routes).
        
        Args:
            `object_parameters` (str, dict): Either dict containing object parameters or '_.json_'/'_.pkl_' filepath
        """
        
        _io._load_objects(self, object_parameters)

    def load_demand_profiles(self, demand_profiles: str | dict | list | tuple) -> list:
        """
        Load demand profile(s) into the simulation.

        Args:
            `demand_profiles` (str, list, tuple): Either a filename to a previously saved DemandProfile object, or list of filenames

        Returns:
            list: List of DemandProfile objects
        """

        return _io._load_demand_profiles(self, demand_profiles)

    def save_data(self, filename: str | None = None, *, overwrite: bool = True, json_indent: int | None = 4) -> None:
        """
        Save all vehicle, detector and junction data in a JSON or pickle file.
        
        Args:
            `filename` (str, optional): Output filepath (defaults to '_./{scenario_name}.pkl_')
            `overwrite` (bool): Prevent previous outputs being overwritten
            `json_indent` (int, optional): Indent used when saving JSON files
        """

        _io._save_data(self, filename, overwrite, json_indent)
            
    def save_fc_data(self, filename: str | None = None, *, overwrite: bool = True, json_indent: int | None = 4) -> None:
        """
        Save all floating car data in a JSON or pickle file.

        Args:
            `filename` (str, optional): Output filepath (defaults to '_./{scenario_name}_fc_data.pkl_')
            `overwrite` (bool): Prevent previous outputs being overwritten
            `json_indent` (int, optional): Indent used when saving JSON files
        """

        _io._save_fc_data(self, filename, overwrite, json_indent)


    # --- SIMPLE GETTERS/CHECKERS ---

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
    
    def get_junction_ids(self) -> list:
        """
        Return list of all junctions in the network.
        
        Returns:
            list: All junction IDs
        """

        return list(self._all_juncs)
    
    def junction_exists(self, junction_id: str) -> bool:
        """
        Tests if a junction exists in the network.
        
        Returns:
            bool: `True` if ID in list of all junction IDs 
        """

        junction_id = validate_type(junction_id, str, "junction_id", self.curr_step)

        return junction_id in self._all_juncs
    
    def get_geometry_ids(self, geometry_types: str | list | tuple | None = None) -> list:
        """
        Return list of IDs for all edges and lanes in the network.
        
        Args:
            `geometry_types` (str, list, tuple, optional): Geometry type ['_edge_' | '_lane_'] or list of types
        
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
    
    def geometry_exists(self, geometry_id: str | int) -> str | None:
        """
        Get geometry type by ID, if geometry with the ID exists.
        
        Args:
            `geometry_id` (str, int): Lane or edge ID
        
        Returns:
            (str, optional):  Geometry type ['_edge_' | '_lane_'], or `None` if it does not exist
        """

        geometry_id = validate_type(geometry_id, str, "geometry_id", self.curr_step)

        if geometry_id in self._all_edges: return "edge"
        elif geometry_id in self._all_lanes: return "lane"
        else: return None

    def get_detector_ids(self, detector_types: str | list | tuple | None = None) -> list:
        """
        Return list of IDs for all edges and lanes in the network.
        
        Args:
            `detector_types` (str, list, tuple, optional): Detector type ['_multientryexit_' | '_inductionloop_'] or list of types
        
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
    
    def detector_exists(self, detector_id: str) -> str | None:
        """
        Get detector type by ID, if a detector with the ID exists.
        
        Args:
            `detector_id` (str): Detector ID
        
        Returns:
            (str, optional): Detector type, or `None` if it does not exist
        """

        detector_id = validate_type(detector_id, str, "detector_id", self.curr_step)

        if detector_id in self.available_detectors.keys():
            return self.available_detectors[detector_id]["type"]
        else: return None


    # --- ADVANCED GETTERS/SETTERS ---

    # - NETWORK -

    def get_interval_network_data(self, data_keys: list | tuple, n_steps: int, *, interval_end: int = 0, get_avg: bool=False) -> float | dict:
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
        
        return _get_set_funcs._get_interval_network_data(self, data_keys, n_steps, interval_end, get_avg)
    

    # - DETECTORS -

    def get_last_step_detector_vehicles(self, detector_ids: str | list | tuple, *, vehicle_types: list | None = None, flatten: bool = False) -> dict | list:
        """
        Get the IDs of vehicles that passed over the specified detectors.
        
        Args:
            `detector_ids` (str, list, tuple): Detector ID or list of IDs (defaults to all)
            `vehicle_types` (list, optional): Included vehicle types
            `flatten` (bool): If true, all IDs are returned in a 1D array, else a dict with vehicles for each detector
        
        Returns:
            (dict, list): Dict or list containing all vehicle IDs
        """

        return _get_set_funcs._get_last_step_detector_vehicles(self, detector_ids, vehicle_types, flatten)
    
    def get_detector_vals(self, detector_ids: list | tuple | str, data_keys: str | list) -> int | float | dict:
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

        return _get_set_funcs._get_detector_vals(self, detector_ids, data_keys)
    
    def get_interval_detector_data(self,
                                   detector_ids: str | list | tuple,
                                   data_keys: str | list,
                                   n_steps: int,
                                   *, 
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

        return _get_set_funcs._get_interval_detector_data(self, detector_ids, data_keys, n_steps, interval_end, avg_step_vals, avg_det_vals, sum_counts)


    # - VEHICLES -

    def get_vehicle_vals(self, vehicle_ids: str | list | tuple, data_keys: str | list) -> dict | str | int | float | list:
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

        return _get_set_funcs._get_vehicle_vals(self, vehicle_ids, data_keys)

    def set_vehicle_vals(self, vehicle_ids: list | tuple | str, **kwargs) -> None:
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

        _get_set_funcs._set_vehicle_vals(self, vehicle_ids, **kwargs)

    def get_vehicle_data(self, vehicle_ids: str | list | tuple, *, refresh: bool = False) -> dict | None:
        """
        Get data for specified vehicle(s).
        
        Args:
            `vehicle_ids` (str, list, tuple): Vehicle ID or list of IDs
            `refresh` (bool): Denotes whether to update static vehicle data
        
        Returns:
            (dict, optional): Vehicle data dictionary, returns None if does not exist in simulation
        """

        return _get_set_funcs._get_vehicle_data(self, vehicle_ids, refresh)


    # - VEHICLE TYPES -

    def get_vehicle_type_vals(self, vehicle_types: str | list | tuple, data_keys: str | list) -> dict | str | float | tuple:
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
                
        return _get_set_funcs._get_vehicle_type_vals(self, vehicle_types, data_keys)
    
    def set_vehicle_type_vals(self, vehicle_types: list | tuple | str, **kwargs) -> None:
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

        _get_set_funcs._set_vehicle_type_vals(self, vehicle_types, **kwargs)


    # - GEOMETRY -

    def get_geometry_vals(self, geometry_ids: str | list | tuple, data_keys: str | list) -> dict | str | int | float | list:
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

        return _get_set_funcs._get_geometry_vals(self, geometry_ids, data_keys)

    def set_geometry_vals(self, geometry_ids: str | list | tuple, **kwargs) -> None:
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
        
        _get_set_funcs._set_geometry_vals(self, geometry_ids, **kwargs)

    def close_road(self, geometry_ids: str | list | tuple) -> None:
        """
        Close a edge/lane indefinitely from its ID.

        Args:
            `geometry_ids` (str, list, tuple): Edge/lane ID or list of edge/lane IDs
        """

        _get_set_funcs._close_road(self, geometry_ids)

    def open_road(self, geometry_ids: str | list | tuple) -> None:
        """
        Open a closed a edge/lane indefinitely from its ID.

        Args:
            `geometry_ids` (str, list, tuple): Edge/lane ID or list of edge/lane IDs
        """

        _get_set_funcs._open_road(self, geometry_ids)

    def get_last_step_geometry_vehicles(self, geometry_ids: str | list, *, vehicle_types: list | None = None, flatten: bool = False) -> dict | list:
        """
        Get the IDs of vehicles on a lane or egde, by geometry ID.
        
        Args:
            `geometry_ids` (str, list):  Edge/lane ID or list of IDs
            `vehicle_types` (list, optional): Included vehicle type IDs
            `flatten` (bool): If `True`, all IDs are returned in a 1D array, else a dict with vehicles for each edge/lane
        
        Returns:
            (dict, list): List containing all vehicle IDs or dictionary containing IDs by edge/lane
        """

        return _get_set_funcs._get_last_step_geometry_vehicles(self, geometry_ids, vehicle_types, flatten)


    # --- VEHICLES ---

    def add_vehicle(self,
                    vehicle_id: str,
                    vehicle_type: str,
                    routing: str | list | tuple,
                    *,
                    initial_speed: str | int | float = "max",
                    origin_lane: str | int = "best",
                    origin_pos: str | int | float = "base") -> None:
        """
        Add a new vehicle into the simulation.
        
        Args:
            `vehicle_id` (str): ID for new vehicle, **must be unique**
            `vehicle_type` (str): Vehicle type ID for new vehicle
            `routing` (str, list, tuple): Either route ID or (2x1) list of edge IDs for origin-destination pair
            `initial_speed` (str, int, float): Initial speed at insertion, either ['_max_' | '_random_'] or number > 0
            `origin_lane` (str, int, float): Lane for insertion at origin, either ['_random_' | '_free_' | '_allowed_' | '_best_' | '_first_'] or lane index
            `origin_pos` (str, int): Longitudinal position at insertion, either ['_random_' | '_free_' | '_random_free_' | '_base_' | '_last_' | '_stop_' | '_splitFront_'] or offset
        """

        _vehicles._add_vehicle(self, vehicle_id, vehicle_type, routing, initial_speed, origin_lane, origin_pos)

    def remove_vehicles(self, vehicle_ids: str | list | tuple) -> None:
        """
        Remove a vehicle or list of vehicles from the simulation.
        
        Args:
            `vehicle_ids` (str, list, tuple): List of vehicle IDs or single ID
        """
        
        _vehicles._remove_vehicles(self, vehicle_ids)

    def add_vehicle_type(self,
                         vehicle_type_id: str,
                         *,
                         vehicle_class: str = "passenger",
                         colour: str | list | tuple | None = None,
                         length: int | float | None = None,
                         width: int | float | None = None,
                         height: int | float | None = None,
                         max_speed: int | float | None = None,
                         speed_factor: int | float | None = None,
                         speed_dev: int | float | None = None,
                         min_gap: int | float | None = None,
                         max_acceleration: int | float | None = None,
                         max_deceleration: int | float | None = None,
                         headway: int | float | None = None,
                         imperfection: int | float | None = None,
                         max_lateral_speed: int | float | None = None,
                         emission_class: str | None = None,
                         gui_shape: str | None = None
                        ) -> None:
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
            `max_acceleration` (int, float, optional): Maximum vehicle acceleration
            `max_deceleration` (int, float, optional): Maximum vehicle deceleration
            `headway` (int, float, optional): Desired minimum time headway in seconds
            `imperfection` (int, float, optional): Driver imperfection (0 denotes perfect driving)
            `max_lateral_speed` (int, float, optional): Maximum lateral speed when lane changing
            `emission_class` (str, optional): Vehicle emissions class ID
            `gui_shape` (str, optional): Vehicle shape in GUI (defaults to vehicle class name)
        """
        
        _vehicles._add_vehicle_type(self, vehicle_type_id, vehicle_class, colour, length, width, height, max_speed, speed_factor,
                                         speed_dev, min_gap, max_acceleration, max_deceleration, headway, imperfection, max_lateral_speed,
                                         emission_class, gui_shape)
        
    def stop_vehicle(self, vehicle_id: str, duration: int | float | None = None, *, lane_idx: int | None = None, pos: int | float | None = None) -> None:
        """
        Stops a vehicle at a given/random position along the next edge. The vehicle will try to stay
        in the same lane when stopping if available. If not, the vehicle will stop in the outermost lane.
        An error is thrown if the vehicle has no next edge ID (i.e. it is at the end of its route).

        Args:
            `vehicle_id` (str): Vehicle ID
            `duration` (int, float, optional): Duration to stop for in seconds (if not given, the vehicle will stop indefinitely)
            `lane_idx` (int, optional): Lane index to stop in (if not given, the vehicle will try to stop in its current lane)
            `pos` (int, float, optional): Position to stop at as a percent of the edge's length (if not given, a random position is chosen)
        """

        _vehicles._stop_vehicle(self, vehicle_id, duration, lane_idx, pos)

    def resume_vehicle(self, vehicle_id: str) -> None:
        """
        Resumes a previously stopped vehicle.
        
        Args:
            `vehicle_id` (str): Vehicle ID
        """

        _vehicles._resume_vehicle(self, vehicle_id)

    def get_vehicle_ids(self, vehicle_types: str | list | tuple | None = None) -> list:
        """
        Return list of IDs for all vehicles currently in the simulation.
        
        Args:
            `vehicle_types` (str, list, tuple, optional): Vehicle type ID or list of IDs (defaults to all)

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
        
    def vehicle_exists(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle exists in the network and has departed.
        
        Returns:
            bool: `True` if ID in list of current vehicle IDs 
        """

        vehicle_id = validate_type(vehicle_id, str, "vehicle_id", self.curr_step)

        return vehicle_id in self._all_curr_vehicle_ids
        
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
    
    def get_vehicle_type_ids(self) -> list:
        """
        Return list of all valid vehicle type IDs.
        
        Returns:
            list: All vehicle type IDs
        """

        return list(self._vehicle_types) + list(self._added_vehicle_types)
    
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
        
    
    # - VEHICLE FUNCTIONS -
    
    def add_vehicle_in_functions(self, functions, parameters: dict | None = None) -> None:
        """
        Add a function (or list of functions) that will are called with each new vehicle that enters the simulation.
        Valid TUD-SUMO parameters are '_simulation_', '_curr_step_', '_vehicle_id_', '_route_id_', '_vehicle_type_',
        '_departure_', '_origin_', '_destination_'. These values are collected from the simulation with each call. Extra
        parameters for any function can be given in the parameters dictionary.
        
        Args:
            `functions` (function, list): Function or list of functions
            `parameters` (dict): Dictionary containing values for extra custom parameters
        """

        _vehicles._add_v_func(self, functions, parameters, self._v_in_funcs, self._valid_v_func_params)

    def remove_vehicle_in_functions(self, functions) -> None:
        """
        Stop a function called with each new vehicle that enters the simulation.
        
        Args:
            functions (function, list): Function (or function name) or list of functions
        """
        
        self._remove_v_func(functions, "in")

    def add_vehicle_out_functions(self, functions, parameters: dict | None = None) -> None:
        """
        Add a function (or list of functions) that will are called with each vehicle that exits the simulation.
        Valid TUD-SUMO parameters are '_simulation_', '_curr_step_' and '_vehicle_id_'. These values are collected
        from the simulation with each call. Extra parameters for any function can be given in the parameters dictionary.
        
        Args:
            `functions` (function, list): Function or list of functions
            `parameters` (dict, optional): Dictionary containing values for extra custom parameters
        """

        _vehicles._add_v_func(self, functions, parameters, self._v_out_funcs, self._valid_v_func_params[:3])

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

    
    # --- TRAFFIC SIGNALS ---

    def set_tl_colour(self, junction_id: str | int, colour_str: str) -> None:
        """
        Sets a junction to a colour for an indefinite amount of time. Can be used when tracking phases separately (ie. not within TUD-SUMO).
        
        Args:
            `junction_id` (str, int): Junction ID
            `colour_str` (str): Phase colour string (valid characters are ['_G_' | '_g_' | '_y_' | '_r_' | '-'])
        """
        
        _traffic_signals._set_tl_colour(self, junction_id, colour_str)
        
    def set_phases(self, junction_phases: dict, *, start_phase: int = 0, overwrite: bool = True) -> None:
        """
        Sets the phases for the simulation, starting at the next simulation step.
        
        Args:
            `junction_phases` (dict): Dictionary containing junction phases and times
            `start_phase` (int): Phase number to start at, defaults to 0
            `overwrite` (bool): If `True`, the `junc_phases` dict is overwitten with `junction_phases`. If `False`, only specific junctions are overwritten.
        """

        _traffic_signals._set_phases(self, junction_phases, start_phase, overwrite)

    def set_m_phases(self, junction_phases: dict, *, start_phase: int = 0, overwrite: bool = True) -> None:
        """
        Sets the traffic light phases for the simulation based on movements, starting at the next simulation step.
        
        Args:
            `junction_phases` (dict): Dictionary containing junction phases, times and masks for different movements
            `start_phase` (int): Phase number to start at, defaults to 0
            `overwrite` (bool): If `True`, the `junc_phases` dict is overwitten with `junction_phases`. If `False`, only specific junctions are overwritten.
        """

        _traffic_signals._set_m_phases(self, junction_phases, start_phase, overwrite)

    def set_tl_metering_rate(self,
                             rm_id: str,
                             metering_rate: int | float,
                             *,
                             g_time: int | float = 1,
                             y_time: int | float = 1,
                             min_red: int | float = 1,
                             vehs_per_cycle: int | None = None,
                             control_interval: int | float = 60
                            ) -> dict:
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
            `vehs_per_cycle` (int, optional): Number of vehicles released with each cycle, defaults to the number of lanes
            `control_interval` (int, float): Ramp meter control interval (s)
        
        Returns:
            dict: Resulting phase dictionary
        """
        
        return _traffic_signals._set_tl_metering_rate(self, rm_id, metering_rate, g_time, y_time, min_red,
                                                      vehs_per_cycle, control_interval)

    def change_phase(self, junction_id: str | int, phase_number: int) -> None:
        """
        Change to a different phase at the specified junction_id.
        
        Args:
            `junction_id` (str, int): Junction ID
            `phase_number` (int): Phase number
        """
        
        _traffic_signals._change_phase(self, junction_id, phase_number)


# --- OBJECTS ---

    def add_tracked_junctions(self, junctions: str | list | tuple | dict | None = None) -> dict:
        """
        Initalise junctions and start tracking states and flows. Defaults to all junctions with traffic lights.
        
        Args:
            `junctions` (str, list, tuple, dict, optional): Junction IDs or list of IDs, or dict containing junction(s) parameters
        
        Returns:
            dict | TrackedJunction: Dictionary of added junctions or single TrackedJunction  object
        """

        return objects._add_tracked_junctions(self, junctions)
    
    def get_tracked_junction_ids(self) -> list:
        """
        Return list of all tracked junctions in the network.
        
        Returns:
            list: All tracked junction IDs
        """

        return list(self.tracked_junctions.keys())
    
    def tracked_junction_exists(self, junction_id: str) -> bool:
        """
        Tests if a tracked junction exists in the network.
        
        Returns:
            bool: `True` if ID in list of tracked junction IDs
        """

        junction_id = validate_type(junction_id, str, "junction_id", self.curr_step)

        return junction_id in self.tracked_junctions
    
    def add_tracked_edges(self, edge_ids: str | list | None = None) -> None:
        """
        Initalise edges and start collecting data.
        
        Args:
            `edge_ids` (str, list, optional): List of edge IDs or single ID, defaults to all
        """

        objects._add_tracked_edges(self, edge_ids)
    
    def get_tracked_edge_ids(self) -> list:
        """
        Return list of all tracked edges in the network.
        
        Returns:
            list: All tracked edges IDs
        """

        return list(self.tracked_edges.keys())
    
    def tracked_edge_exists(self, edge_id: str) -> bool:
        """
        Tests if a tracked junction exists in the network.
        
        Returns:
            bool: `True` if ID in list of tracked junction IDs
        """

        edge_id = validate_type(edge_id, str, "edge_id", self.curr_step)

        return edge_id in self.tracked_edges
           
    def add_controllers(self, controller_params: str | dict) -> dict:
        """
        Add controllers from parameters in a dictionary/JSON file.
        
        Args:
            `controller_params` (str, dict): Controller parameters dictionary or filepath

        Returns:
            dict: Dictionary of controller objects by their ID
        """

        return controllers._add_controllers(self, controller_params)
    
    def remove_controllers(self, controller_ids: str | list | tuple) -> None:
        """
        Remove controllers and delete their collected data.
        
        Args:
            `controller_ids` (str, list, tuple): Controller ID or list of IDs
        """

        controllers._remove_controllers(self, controller_ids)

    def get_controller_ids(self, controller_types: str | list | tuple | None = None) -> list:
        """
        Return list of all controller IDs, or controllers of specified type ('VSLController' or 'RGController').
        
        Args:
            `controller_types` (str, list, tuple, optional): Controller type, defaults to all
        
        Returns:
            list: Controller IDs
        """

        valid_types = ["VSLController", "RGController"]
        if isinstance(controller_types, str): controller_types = [controller_types]
        elif controller_types == None: controller_types = valid_types
        elif not isinstance(controller_types, (list, tuple)):
            desc = "Invalid controller_types '{0}' type (must be [str | list | tuple], not '{1}').".format(controller_types, type(controller_types).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        if len(set(controller_types) - set(valid_types)) != 0:
            desc = "Invalid controller_types (must only include ['VSLController' | 'RGController'])."
            raise_error(KeyError, desc, self.curr_step)

        return list([c_id for c_id, c in self.controllers.items() if c.__name__() in controller_types])
    
    def controller_exists(self, controller_id: str) -> str | None:
        """
        Get whether controller with ID controller_id exists.
        
        Args:
            `controller_id` (str): Controller ID
        
        Returns:
            (str, optional): Returns `None` if it does not exist, otherwise type ['_VSLController_' | '_RGController_']
        """

        if controller_id in self.controllers: return self.controllers[controller_id].__name__()
        else: return None


    # --- EVENTS ---

    def add_events(self, event_params: Event | str | list | dict) -> None:
        """
        Add events and event scheduler.
        
        Args:
            `event_parms` (Event, str, list, dict): Event parameters [Event | [Event] | dict | filepath]
        """

        if self._scheduler == None:
            self._scheduler = EventScheduler(self)
        self._scheduler.add_events(event_params)

    def remove_events(self, event_ids: str | list | tuple | None = None) -> None:
        """
        Remove event(s) from the simulation. Scheduled and completed events are deleted
        from the event scheduler, whilst active events are terminated early. Defaults
        to removing **_all currently active_** events.

        Args:
            `event_ids` (str, list, tuple, optional): List of event IDs
        """
        
        all_events = self._scheduler.get_event_ids()

        if isinstance(event_ids, str): event_ids = [event_ids]
        if event_ids == None: event_ids = self._scheduler.get_event_ids("active")

        if len(event_ids) > 0:
            for event_id in event_ids:
                if event_id in all_events: self._scheduler.remove_event(event_id)
        else:
            desc = "Invalid event_ids '[]' (given empty list)."
            raise_error(ValueError, desc, self.curr_step)

    def get_event_ids(self, event_statuses: str | list | tuple | None = None) -> list:
        """
        Return event IDs by status, one or more of '_scheduled_', '_active_' or '_completed_'.
        
        Args:
            `event_statuses` (str, list, tuple, optional): Event status type or list of types
        
        Returns:
            list: List of event IDs
        """

        valid_statuses = ["scheduled", "active", "completed"]
        if isinstance(event_statuses, str): event_statuses = [event_statuses]
        elif event_statuses == None: event_statuses = valid_statuses
        elif not isinstance(event_statuses, (list, tuple)):
            desc = "Invalid event_statuses '{0}' type (must be [str | list | tuple], not '{1}').".format(event_statuses, type(event_statuses).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        if len(set(event_statuses) - set(valid_statuses)) != 0:
            desc = "Invalid event_statuses (must only include ['scheduled' | 'active' | 'completed'])."
            raise_error(KeyError, desc, self.curr_step)

        return self._scheduler.get_event_ids(event_statuses)

    def event_exists(self, event_id: str) -> str | None:
        """
        Get whether event with the given ID exists.
        
        Args:
            `event_id` (str): Event ID
        
        Returns:
            (str, optional): Returns `None` if it does not exist, otherwise status ['_scheduled_' | '_active_' | '_completed_']
        """

        if self._scheduler == None: return None
        else: return self._scheduler.get_event_status(event_id)

    def get_event(self, event_id: str) -> Event:
        """
        Get event object by its ID.

        Args:
            `event_id` (str): Event ID
        
        Returns:
            Event: Event object
        """
        return self._scheduler.get_event(event_id)

    def cause_incident(self,
                       duration: int,
                       *,
                       vehicle_ids: str | list | tuple | None = None,
                       geometry_ids: str | list | tuple = None,
                       n_vehicles: int = 1,
                       vehicle_separation: float = 0,
                       assert_n_vehicles: bool = False,
                       edge_speed: int | float | None = -1,
                       position: int | float | None = None,
                       highlight_vehicles: bool = True,
                       incident_id: str | None = None
                      ) -> bool:
        """
        Simulates an incident by stopping vehicle(s) on the following edge in their route for a
        period of time, before removing them from the simulation. Vehicle(s) can either be specified
        using `vehicle_ids`, chosen randomly based location using `geometry_ids`, or vehicles can
        be chosen randomly throughout the network if neither `vehicle_ids` or `geometry_ids` are given.
        
        Args:
            `duration` (int): Duration of incident (in seconds)
            `vehicle_ids` (str, list, tuple, optional): Vehicle ID or list of IDs to include in the incident
            `geometry_ids` (str, list, tuple, optional): Geometry ID or list of IDs to randomly select vehicles from
            `n_vehicles` (int): Number of vehicles in the incident, if randomly chosen
            `vehicle_separation` (float): Factor denoting how separated randomly chosen vehicles are (0.1-1)
            `assert_n_vehicles` (bool): Denotes whether to throw an error if the correct number of vehicles cannot be found
            `edge_speed` (int, float, None, optional): New max speed for edges where incident vehicles are located (defaults to 15km/h or 10mph). Set to `None` to not change speed.
            `position` (int, float, optional): Distance along edge where vehicle will stop [0 (start) - 1 (end)]
            `highlight_vehicles` (bool): Denotes whether to highlight vehicles in the SUMO GUI
            `incident_id` (str, optional): Incident event ID used in the simulation data file (defaults to '_incident_{n}_')

        Returns:
            bool: Denotes whether incident was successfully created
        """
        
        return _cause_incident(self, duration, vehicle_ids, geometry_ids, n_vehicles, vehicle_separation, assert_n_vehicles, edge_speed,
                               position, highlight_vehicles, incident_id)
    
    def add_weather(self,
                    duration: int | float,
                    strength: float = 0.2,
                    locations: list | tuple | None = None,
                    *,
                    weather_id: str | None = None,
                    headway_increase: int | float | None = None,
                    imperfection_increase: int | float | None = None,
                    acceleration_reduction: int | float | None = None,
                    speed_f_reduction: float | int | None = None
                   ) -> str:
        """
        Starts simulating weather effects in the next time step. Both desired time headway and driver imperfection
        are increased, whilst acceleration/deceleration and driver speed factor are reduced. The increase/reduction
        can either be defined individually or using the `strength` parameter, which is otherwise used as the default
        increase/reduction.

        Weather can be localised by setting `locations` to a list of geometry IDs, or can be made network-wide by
        omitting `locations`.

        Args:
            `duration` (int, float): Duration of active weather effects in seconds
            `strength` (float): Used as the default reduction/increase value when not given (defaults to 0.2)
            `locations` (list, tuple, optional): List of edge/lane IDs where effects will be active (defaults to network-wide effects)
            `weather_id` (str, optional): Event ID (defaults to 'weather_x', where x is the )
            `headway_increase` (int, float, optional): Percent increase to vehicle type desired time headway (tau)
            `imperfection_increase` (int, float, optional): Percent increase to vehicle type imperfection value (sigma)
            `acceleration_reduction` (int, float, optional): Percent reduction to vehicle type maximum acceleration/deceleration
            `speed_f_reduction` (int, float, optional): Percent reduction to vehicle type speed factor, used to calculate vehicle speed based on speed limit

        Returns:
            str: Weather event ID
        """

        return _add_weather(self, duration, strength, locations, weather_id, headway_increase, imperfection_increase,
                            acceleration_reduction, speed_f_reduction)
    
    def get_active_weather(self) -> tuple:
        
        if self._scheduler != None:
            active_events = self._scheduler.get_event_ids("active")
            return tuple(e_id for e_id in active_events if e_id in self._weather_events)
        
        else: return tuple()
    

    # --- ROUTING ---

    def add_route(self, routing: list | tuple, route_id: str | None = None, *, assert_new_id: bool = True) -> None:
        """
        Add a new route. If only 2 edge IDs are given, vehicles calculate
        optimal route at insertion, otherwise vehicles take specific edges.
        
        Args:
            `routing` (list, tuple): List of edge IDs
            `route_id` (str, optional): Route ID, if not given, generated from origin-destination
            `assert_new_id` (bool): If True, an error is thrown for duplicate route IDs
        """
        
        _routing._add_route(self, routing, route_id, assert_new_id)

    def get_path_edges(self, origin: str, destination: str, *, curr_optimal: bool = True) -> list | None:
        """
        Find an optimal route between 2 edges (using the A* algorithm).
        
        Args:
            `origin` (str): Origin edge ID
            `destination` (str): Destination edge ID
            `curr_optimal` (str): Denotes whether to find current optimal route (ie. whether to consider current conditions)
        
        Returns:
            (None, (list, float)): List of edge IDs & travel time (s) (based on curr_optimal), or None if no route exists
        """
        
        return _routing._get_path_edges(self, origin, destination, curr_optimal)
    
    def get_path_travel_time(self, edge_ids: list | tuple, *, curr_tt: bool = True, unit: str = "seconds") -> float:
        """
        Calculates the travel time for a route.
        
        Args:
            `edge_ids` (list, tuple): List of edge IDs
            `curr_tt` (bool): Denotes whether to find the current travel time (ie. whether to consider current conditions)
            `unit` (str): Time unit (either ['_steps_' | '_seconds_' | '_minutes_' | '_hours_']) (defaults to seconds)
        
        Returns:
            float: Travel time in specified unit
        """
        
        edge_ids = validate_list_types(edge_ids, str, param_name="edge_ids", curr_sim_step=self.curr_step)
        
        tt_key = "curr_travel_time" if curr_tt else "ff_travel_time"
        total_tt = sum([self.get_geometry_vals(edge_id, tt_key) for edge_id in edge_ids])

        return convert_units(total_tt, "hours", unit, self.step_length)

    def is_valid_path(self, edge_ids: list | tuple) -> bool:
        """
        Checks whether a list of edges is a valid connected path. If two disconnected
        edges are given, it returns whether there is a path between them.
        
        Args:
            `edge_ids` (list, tuple): List of edge IDs
        
        Returns:
            bool: Denotes whether it is a valid path
        """

        return _routing._is_valid_path(self, edge_ids)

    def route_exists(self, route_id: str) -> str | None:
        """
        Get route edges by ID, if a route with the ID exists.
        
        Args:
            `route_id` (str): Route ID
        
        Returns:
            (str, optional): List of route edges, or `None` if it does not exist.
        """

        route_id = validate_type(route_id, str, "route_id", self.curr_step)

        if route_id in self._all_routes.keys():
            return self._all_routes[route_id]
        else: return None


    # --- GUI ---

    def gui_track_vehicle(self, vehicle_id: str, view_id: str | None = None, *, highlight: bool = True) -> None:
        """
        Sets GUI view to track a vehicle by ID.

        Args:
            `vehicle_id` (str): Vehicle ID
            `view_id` (str, optional): View ID, if `None` uses default
        """
        
        _gui._track_vehicle(self, vehicle_id, view_id, highlight)

    def gui_stop_tracking(self, view_id: str | None) -> None:
        """
        Stops GUI view from tracking vehicle.

        Args:
            `view_id` (str, optional): View ID (defaults to default view)
        """

        _gui._stop_tracking(self, view_id)

    def gui_is_tracking(self, view_id: str | None = None) -> bool:
        """
        Returns whether a GUI view is tracking a vehicle.

        Args:
            `view_id` (str, optional): View ID (defaults to default view)
        """
        
        if isinstance(self._gui_veh_tracking, dict): return view_id in self._gui_veh_tracking
        return False      

    def add_gui_view(self, view_id: str, bounds: list | tuple | None = None, zoom: int | float | None = None) -> None:
        """
        Adds a new GUI view.
        
        Args:
            `view_id` (str): View ID
            `bounds` (list, tuple, optional): View bounds coordinates (lower-left, upper-right)
            `zoom` (int, float, optional): Zoom level
        """

        _gui._add_view(self, view_id, bounds, zoom)

    def remove_gui_view(self, view_id: str) -> None:
        """
        Removes a GUI view.
        
        Args:
            `view_id` (str): View ID
        """

        _gui._remove_view(self, view_id)

    def get_gui_views(self) -> list:
        """
        Returns a list of all GUI view IDs.
        
        Returns:
            list: List of view IDs
        """

        return [self._default_view] + self._gui_views
    
    def get_view_boundaries(self, view_id: str | None = None) -> tuple:
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
    
    def get_view_zoom(self, view_id: str | None = None) -> int:
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
    
    def set_view(self, view_id: str | None = None, bounds: list | tuple | None = None, zoom: int | float | None = None) -> None:
        """
        Sets the bounds and/or zoom level of a GUI view.

        Args:
            `view_id` (str, optional): View ID (defaults to default view)
            `bounds` (list, tuple, optional): View bounds coordinates (lower-left, upper-right)
            `zoom` (int, float, optional): Zoom level
        """

        _gui._set_view(self, view_id, bounds, zoom)
    
    def take_screenshot(self, filename: str, view_id: str | None = None, bounds: list | tuple | None = None, zoom: int | float | None = None) -> None:
        """
        Takes a screenshot of a GUI view and saves result to a file.

        Args:
            `filename` (str): Screenshot filename
            `view_id` (str, optional): View ID (defaults to default view)
            `bounds` (list, tuple, optional): View bounds coordinates (lower-left, upper-right) (defaults to current bounds)
            `zoom` (int, float, optional): Zoom level (defaults to current zoom)
        """

        _gui._take_screenshot(self, filename, view_id, bounds, zoom)  


    # --- HELPERS ---

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
