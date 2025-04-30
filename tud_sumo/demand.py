import csv, datetime, copy, pickle, os, string, random as rnd
import xml.etree.ElementTree as et  
from .utils import *

class DemandProfile:

    def __init__(self, simulation=None, step_length=1):
        from .simulation import Simulation

        self.id = None
        self._sim = None
    
        if simulation != None:
            self.add_to_simulation(simulation)
            self.step_length = self._sim.step_length
        elif step_length != None:
            self.step_length = step_length
        else:
            desc = "No Simulation object or step length given."
            raise_error(ValueError, desc)

        self._demand_headers = ["routing", "step_range", "veh/hour", "vehicle_types", "vehicle_type_dists",
                                "init_speed", "origin_lane", "origin_pos", "insertion_sd", "colour"]
        self._demand_arrs = []

        self._vehicle_types = {}

        self.active = True

    def __name__(self): return "DemandProfile"

    def _generate_id(self, n: int=10): 
        """ Generates random ID of length 'n'."""

        return ''.join(rnd.choice(string.ascii_uppercase + string.digits) for _ in range(n))

    def activate(self) -> None:
        """ Activate the demand profile. """
        self.active = True

    def deactivate(self) -> None:
        """ Deactivate the demand profile. """
        self.active = False

    def add_to_simulation(self, simulation) -> None:
        """
        Adds the profile to a simulation object.
        
        Args:
            `simulation` (Simulation): Simulation object to add the profile to
        """

        if self._sim != None: self.remove()

        self._sim = simulation
    
        while self.id == None or self.id in self._sim._demand_profiles:
            self.id = self._generate_id()

        self._sim._demand_profiles[self.id] = self

        if len(self._vehicle_types) > 0:
            for vehicle_type_id, vehicle_type_data in self._vehicle_types.items():
                self._sim.add_vehicle_type(vehicle_type_id, **vehicle_type_data)

    def remove(self) -> None:
        """ Removes the profile from its corresponding simulation. """

        if self._sim != None:
            del self._sim._demand_profiles[self.id]
            self._sim = None

    def is_complete(self) -> bool:
        """ Returns whether all scheduled vehicles have been added to the simulation. """
        
        if self._sim == None: return False
        elif len(self._demand_arrs) == 0: return True
        else:
            end_times = [arr[1][1] for arr in self._demand_arrs]
            return max(end_times) > self._sim.curr_step
        
    def plot_demand(self, routing: str|list|tuple|None=None, save_fig: str|None=None) -> None:
        """
        Plots demand within a demand profile. Alternatively, use `Plotter.plot_demand()`.

        Args:
            `routing` (str, list, tuple, optional): Either string (route ID or 'all'), OD pair eg. ('A', 'B') or None (defaulting to all)
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self._sim == None:
            desc = "Cannot plot demand (no simulation object found)."
            raise_error(ValueError, desc)

        from .plot import Plotter

        plt = Plotter(self._sim, time_unit="hours")
        plt.plot_demand(routing, self, save_fig=save_fig)
        del plt

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

        vehicle_type_data = {name: locals()[name] for name in valid_vehicle_type_val_keys if locals()[name] != None}

        if self._sim != None: self._sim.add_vehicle_type(vehicle_type_id, **vehicle_type_data)
        self._vehicle_types[vehicle_type_id] = vehicle_type_data
        
    def create_route_file(self, filename: str) -> None:
        """
        Create a SUMO '.rou.xml' file from the demand added to this profile.

        Args:
            `filename` (str): Filename for route file
        """

        if len(self._vehicle_types) == 0 and len(self._demand_arrs) == 0:
            desc = "Cannot create route file (no vehicle types or demand data found)."
            raise_error(KeyError, desc)
        
        if not filename.endswith(".rou.xml"): filename += ".rou.xml"

        root = et.Element("routes", attrib={"xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance",
                                             "xsi:noNamespaceSchemaLocation": "http://sumo.dlr.de/xsd/routes_file.xsd"})

        if len(self._vehicle_types) > 0:

            sumo_names = {'vehicle_class': 'vClass', 'colour': 'color', 'length': 'length', 'height': 'height', 'mass': 'mass',
                          'speed_factor': 'speedFactor', 'speed_dev': 'speedDev', 'min_gap': 'minGap', 'acceleration': 'accel',
                          'deceleration': 'decel', 'tau': 'tau', 'max_lateral_speed': 'maxSpeedLat',  'emission_class': "emissionClass",
                          'gui_shape': 'guiShape'}

            root.append(et.Comment(" VTypes "))

            for vehicle_type_id, vehicle_type_data in self._vehicle_types.items():
                
                attributes = {"id": vehicle_type_id}
                sumo_attributes = {sumo_names[name]: value for name, value in vehicle_type_data.items() if value != None}

                if "color" in sumo_attributes:
                    if sumo_attributes["color"] not in sumo_colours:
                        colour = colour_to_rgba(sumo_attributes["color"])
                        if isinstance(colour, (list, tuple)): colour = ",".join([str(val) for val in colour])
                        sumo_attributes["color"] = colour

                attributes.update(sumo_attributes)
                root.append(et.Element("vType", attrib=attributes))

        if len(self._demand_arrs) > 0:
            root.append(et.Comment(" Vehicles, persons and containers (sorted by depart) "))

            idx = 0
            for demand_arr in self._demand_arrs:
                
                vehs_per_hour = demand_arr[2]
                if vehs_per_hour <= 0: continue

                origin, destination = demand_arr[0]
                start, end = demand_arr[1]
                vehicle_types = demand_arr[3]
                vehicle_type_dists = demand_arr[4]

                attributes = {"id": "", "type": "", "begin": str(start), "from": origin, "to": destination, "end": str(end),
                            "vehsPerHour": 0, "departLane": str(demand_arr[6]), "departPos": str(demand_arr[7]), "departSpeed": str(demand_arr[5])}
                
                if demand_arr[9] != None: 
                    if demand_arr[9] not in sumo_colours:
                        colour = colour_to_rgba(demand_arr[9])
                        if isinstance(colour, (list, tuple)): colour = ",".join([str(val) for val in colour])
                    else: colour = demand_arr[9]
                    attributes["color"] = colour
                    
                if isinstance(vehicle_types, str): vehicle_types = [vehicle_types]
                if vehicle_type_dists == None: vehicle_type_dists = [1] * len(vehicle_types)
                
                for vehicle_type, type_dist in zip(vehicle_types, vehicle_type_dists):
                    attributes["id"] = f"flow_{idx}"
                    if vehicle_type != "DEFAULT_VEHTYPE": attributes["type"] = vehicle_type
                    else: del attributes["type"]
                    attributes["vehsPerHour"] = str(vehs_per_hour * type_dist)

                    root.append(et.Element("flow", attrib=attributes))
                    idx += 1

        _save_xml(root, filename)

    def load_demand(self, csv_file: str) -> None:
        """
        Loads OD demand from a '_.csv_' file. The file must contain an 'origin/destination' or 'route_id' column(s), 'start_time/end_time' or 'start_step/end_step' columns(s) and a 'demand/number' column.
        
        Args:
            `csv_file` (str): Demand file location
        """

        csv_file = validate_type(csv_file, str, "demand file")
        if csv_file.endswith(".csv"):
            if os.path.exists(csv_file):
                with open(csv_file, "r") as fp:

                    valid_cols = ["origin", "destination", "route_id", "start_time", "end_time", "start_step", "end_step", "demand", "number",
                                  "vehicle_types", "vehicle_type_dists", "initial_speed", "origin_lane", "origin_pos", "insertion_sd", "colour"]
                    demand_idxs = {}
                    reader = csv.reader(fp)
                    for idx, row in enumerate(reader):

                        # First, get index of (valid) columns to read data correctly and
                        # store in demand_idxs dict
                        if idx == 0:

                            if len(set(row) - set(valid_cols)) != 0:
                                desc = "Invalid demand file (unknown columns '{0}').".format("', '".join(list(set(row) - set(valid_cols))))
                                raise_error(KeyError, desc)

                            if "route_id" in row: demand_idxs["route_id"] = row.index("route_id")
                            elif "origin" in row and "destination" in row:
                                demand_idxs["origin"] = row.index("origin")
                                demand_idxs["destination"] = row.index("destination")
                            else:
                                desc = "Invalid demand file (no routing values, must contain 'route_id' or 'origin/destination')."
                                raise_error(KeyError, desc)
                            
                            if "start_time" in row and "end_time" in row:
                                demand_idxs["start_time"] = row.index("start_time")
                                demand_idxs["end_time"] = row.index("end_time")
                            elif "start_step" in row and "end_step" in row:
                                demand_idxs["start_step"] = row.index("start_step")
                                demand_idxs["end_step"] = row.index("end_step")
                            else:
                                desc = "Invalid demand file (no time values, must contain 'start_time/end_time' or 'start_step/end_step')."
                                raise_error(KeyError, desc)

                            if "demand" in row: demand_idxs["demand"] = row.index("demand")
                            elif "number" in row: demand_idxs["number"] = row.index("number")
                            else:
                                desc = "Invalid demand file (no demand values, must contain 'demand/number')."
                                raise_error(KeyError, desc)

                            if "vehicle_types" in row:
                                demand_idxs["vehicle_types"] = row.index("vehicle_types")
                            
                            if "vehicle_type_dists" in row and "vehicle_types" in row:
                                demand_idxs["vehicle_type_dists"] = row.index("vehicle_type_dists")

                            if "initial_speed" in row:
                                demand_idxs["initial_speed"] = row.index("initial_speed")

                            if "origin_lane" in row:
                                demand_idxs["origin_lane"] = row.index("origin_lane")

                            if "origin_pos" in row:
                                demand_idxs["origin_pos"] = row.index("origin_pos")

                            if "insertion_sd" in row:
                                demand_idxs["insertion_sd"] = row.index("insertion_sd")

                            if "colour" in row:
                                demand_idxs["colour"] = row.index("colour")

                        else:

                            # Use demand_idx dict to get all demand data from the correct indices

                            if "route_id" in demand_idxs: routing = row[demand_idxs["route_id"]]
                            else: routing = (row[demand_idxs["origin"]], row[demand_idxs["destination"]])

                            if "start_time" in demand_idxs: 
                                step_range = (int(row[demand_idxs["start_time"]]) / self.step_length, int(row[demand_idxs["end_time"]]) / self.step_length)
                            else: step_range = (int(row[demand_idxs["start_step"]]), int(row[demand_idxs["end_step"]]))

                            step_range = [int(val) for val in step_range]

                            # Convert to flow in vehicles/hour if using 'number'
                            if "number" in demand_idxs: demand = int(row[demand_idxs["number"]]) / convert_units(step_range[1] - step_range[0], "steps", "hours", self.step_length)
                            else: demand = float(row[demand_idxs["demand"]])

                            if "vehicle_types" in demand_idxs:
                                vehicle_types = row[demand_idxs["vehicle_types"]].split(",")
                                if len(vehicle_types) == 1: vehicle_types = vehicle_types[0]
                            else:
                                # If vehicle types not defined, use SUMO default vehicle type (standard car)
                                vehicle_types = "DEFAULT_VEHTYPE"
                                if "vehicle_type_dists" in demand_idxs:
                                    desc = "vehicle_type_dists given without vehicle_types."
                                    raise_error(ValueError, desc)

                            if isinstance(vehicle_types, (list, tuple)):
                                if "vehicle_type_dists" in demand_idxs:
                                    vehicle_type_dists = row[demand_idxs["vehicle_type_dists"]].split(",")
                                    if len(vehicle_type_dists) != len(vehicle_types):
                                        desc = "Invalid vehicle_type_dists '[{0}]' (must be same length as vehicle_types '{1}').".format(", ".join(vehicle_type_dists), len(vehicle_types))
                                        raise_error(ValueError, desc)
                                    else: vehicle_type_dists = [float(val) for val in vehicle_type_dists]
                                else: vehicle_type_dists = 1 if isinstance(vehicle_types, str) else [1 / len(vehicle_types)]*len(vehicle_types)
                            else:
                                vehicle_type_dists = None

                            if "initial_speed" in demand_idxs:
                                initial_speed = row[demand_idxs["initial_speed"]]
                                if initial_speed.isdigit(): initial_speed = float(initial_speed)
                            else: initial_speed = "max"

                            if "origin_lane" in demand_idxs:
                                origin_lane = row[demand_idxs["origin_lane"]]
                                if origin_lane.isdigit(): origin_lane = int(origin_lane)
                            else: origin_lane = "best"

                            if "origin_pos" in demand_idxs:
                                origin_pos = row[demand_idxs["origin_pos"]]
                            else: origin_pos = "base"

                            if "insertion_sd" in demand_idxs:
                                insertion_sd = float(row[demand_idxs["insertion_sd"]])
                            else: insertion_sd = 0.333

                            if "colour" in demand_idxs:
                                colour = row[demand_idxs["colour"]]
                            else: colour = None

                            self.add_demand(routing, step_range, demand, vehicle_types, vehicle_type_dists, initial_speed, origin_lane, origin_pos, insertion_sd, colour)
                            
            else:
                desc = "Demand file '{0}' not found.".format(csv_file)
                raise_error(FileNotFoundError, desc)
        else:
            desc = "Invalid demand file '{0}' format (must be '.csv').".format(csv_file)
            raise_error(ValueError, desc)

    def add_demand(self, routing: str|list|tuple, step_range: list|tuple, demand: int|float, vehicle_types: str|list|tuple|None = None, vehicle_type_dists: list|tuple|None = None, initial_speed: str|int|float = "max", origin_lane: str|int|float = "best", origin_pos: str|int = "base", insertion_sd: float = 0.333, colour: str|list|tuple|None = None) -> None:
        """
        Adds traffic flow demand for a specific route and time.
        
        Args:
            `routing` (str, list, tuple): Either a route ID or OD pair of edge IDs
            `step_range` (str, list, tuple): (2x1) list or tuple denoting the start and end steps of the demand
            `demand` (int, float): Generated flow in vehicles/hour
            `vehicle_types` (str, list, tuple, optional):List of vehicle type IDs
            `vehicle_type_dists` (list, tuple, optional):Vehicle type distributions used when generating flow
            `initial_speed` (str, int, float): Initial speed at insertion, either ['_max_'|'_random_'] or number > 0
            `origin_lane` (str, int): Lane for insertion at origin, either ['_random_'|'_free_'|'_allowed_'|'_best_'|'_first_'] or lane index
            `origin_pos` (str, int): Longitudinal position at insertion, either ['_random_'|'_free_'|'_random_free_'|'_base_'|'_last_'|'_stop_'|'_splitFront_'] or offset
            `insertion_sd` (float): Vehicle insertion number standard deviation, at each step
            `colour` (str, list, tuple, optional): Vehicle colour, either hex code, list of rgb/rgba values or valid SUMO colour string
        """

        if self._sim != None:
            routing = validate_type(routing, (str, list, tuple), "routing", self._sim.curr_step)
            if isinstance(routing, str) and not self._sim.route_exists(routing):
                desc = "Unknown route ID '{0}'.".format(routing)
                raise_error(KeyError, desc, self._sim.curr_step)
            elif isinstance(routing, (list, tuple)):
                routing = validate_list_types(routing, (str, str), True, "routing", self._sim.curr_step)
                if not self._sim.is_valid_path(routing):
                    desc = "No route between edges '{0}' and '{1}'.".format(routing[0], routing[1])
                    raise_error(ValueError, desc, self._sim.curr_step)

            step_range = validate_list_types(step_range, ((int), (int)), True, "step_range", self._sim.curr_step)
            if step_range[1] < step_range[0] or step_range[1] < self._sim.curr_step:
                desc = "Invalid step_range '{0}' (must be valid range and end > current step)."
                raise_error(ValueError, desc, self._sim.curr_step)

            if vehicle_types != None:
                vehicle_types = validate_type(vehicle_types, (str, list, tuple), param_name="vehicle_types", curr_sim_step=self._sim.curr_step)
                if isinstance(vehicle_types, (list, tuple)):
                    vehicle_types = validate_list_types(vehicle_types, str, param_name="vehicle_types", curr_sim_step=self._sim.curr_step)
                    for type_id in vehicle_types:
                        if not self._sim.vehicle_type_exists(type_id):
                            desc = "Unknown vehicle type ID '{0}' in vehicle_types.".format(type_id)
                            raise_error(KeyError, desc, self._sim.curr_step)
                elif not self._sim.vehicle_type_exists(vehicle_types):
                    desc = "Unknown vehicle_types ID '{0}' given.".format(vehicle_types)
                    raise_error(KeyError, desc, self._sim.curr_step)
            else: vehicle_types = "DEFAULT_VEHTYPE"
            
            if vehicle_type_dists != None and vehicle_types == None:
                desc = "vehicle_type_dists given, but no vehicle types."
                raise_error(ValueError, desc, self._sim.curr_step)
            elif vehicle_type_dists != None and isinstance(vehicle_types, str):
                desc = "Invalid vehicle_type_dists (vehicle_types is a single type ID, so no distribution)."
                raise_error(ValueError, desc, self._sim.curr_step)
            elif vehicle_type_dists != None:
                vehicle_type_dists = validate_list_types(vehicle_type_dists, float, param_name="vehicle_type_dists", curr_sim_step=self._sim.curr_step)
                if len(vehicle_type_dists) != len(vehicle_types):
                    desc = "Invalid vehicle_type_dists (must be same length as vehicle_types, {0} != {1}).".format(len(vehicle_type_dists), len(vehicle_types))
                    raise_warning(ValueError, desc, self._sim.curr_step)

            insertion_sd = validate_type(insertion_sd, (int, float), "insertion_sd", self._sim.curr_step)

            self._sim._manual_flow = True
        self._demand_arrs.append([routing, step_range, demand, vehicle_types, vehicle_type_dists, initial_speed, origin_lane, origin_pos, insertion_sd, colour])

    def add_demand_function(self, routing: str|list|tuple, step_range: list|tuple, demand_function, parameters: dict|None = None, vehicle_types: str|list|tuple|None = None, vehicle_type_dists: list|tuple|None = None, initial_speed: str|int|float = "max", origin_lane: str|int|float = "best", origin_pos: str|int = "base", insertion_sd: float = 0.333, colour: str|list|tuple|None = None) -> None:
        """
        Adds traffic flow demand calculated for each step using a 'demand_function'. 'step' is the only required parameter of the function.

        Args:
            `routing` (str, list, tuple): Either a route ID or OD pair of edge IDs
            `step_range` (list, tuple): (2x1) list or tuple denoting the start and end steps of the demand
            `demand_function` (function): Function used to calculate flow (vehicles/hour)
            `parameters` (dict, optional):Dictionary containing extra parameters for the demand function
            `vehicle_types` (str, list, tuple, optional): List of vehicle type IDs
            `vehicle_type_dists` (list, tuple, optional): Vehicle type distributions used when generating flow
            `initial_speed` (str, int, float): Initial speed at insertion, either ['_max_'|'_random_'] or number > 0
            `origin_lane` (str, int, float): Lane for insertion at origin, either ['_random_'|'_free_'|'_allowed_'|'_best_'|'_first_'] or lane index
            `origin_pos` (str, int): Longitudinal position at insertion, either ['_random_'|'_free_'|'_random_free_'|'_base_'|'_last_'|'_stop_'|'_splitFront_'] or offset
            `insertion_sd` (float): Vehicle insertion number standard deviation, at each step
            `colour` (str, list, tuple, optional): Vehicle colour, either hex code, list of rgb/rgba values or valid SUMO colour string
        """
        
        step_range = validate_list_types(step_range, ((int), (int)), True, "step_range")
        if step_range[1] < step_range[0] or (self._sim != None and step_range[1] < self._sim.curr_step):
            desc = "Invalid step_range '{0}' (must be valid range and end > current step)."
            raise_error(ValueError, desc)
        
        # Step through start -> end, calculate flow value 
        for step_no in range(step_range[0], step_range[1]):
            
            params = {"step": step_no}
            if parameters != None: params.update(parameters)
            demand_val = demand_function(**params)

            # Outputs must be a number
            if not isinstance(demand_val, (int, float)):
                desc = "Invalid demand function (output must be type 'int', not '{0}').".format(type(demand_val).__name__)
                raise_error(TypeError, desc)
            
            # Skip if equal to or less than 0
            if demand_val <= 0: continue

            self.add_demand(routing, (step_no, step_no), demand_val, vehicle_types, vehicle_type_dists, initial_speed, origin_lane, origin_pos, insertion_sd, colour)

    def remove_demand(self, start_step: int, end_step: int) -> None:
        """
        Removes all demand from the profile in the range [start_step, end_step] (inclusive).

        Args:
            `start_step` (int): Start step of the removal period (seconds)
            `end_step` (int): End step of the removal period (seconds)
        """

        rm_start, rm_end = start_step * self.step_length, end_step * self.step_length

        new_arrs = []
        for demand_arr in self._demand_arrs:

            arr_start, arr_end = demand_arr[1][0], demand_arr[1][1]
            
            if arr_start < rm_start and arr_end <= rm_end:
                demand_arr[1] = (demand_arr[1][0], min(demand_arr[1][1], rm_start - self.step_length))
            
            elif arr_start >= rm_start and arr_end > rm_end:
                demand_arr[1] = (max(demand_arr[1][0], rm_end + self.step_length), demand_arr[1][1])

            elif arr_start >= rm_start and arr_end <= rm_end:
                continue

            elif arr_start < rm_start and arr_end > rm_end:

                cp1 = copy.deepcopy(demand_arr)
                cp1[1][1] = rm_start - self.step_length
                new_arrs.append(cp1)

                cp2 = copy.deepcopy(demand_arr)
                cp2[1][0] = rm_end + self.step_length
                new_arrs.append(cp2)

                continue

            new_arrs.append(copy.deepcopy(demand_arr))

        self._demand_arrs = copy.deepcopy(new_arrs)

    def save(self, filename: str, overwrite: bool = True) -> None:
        """
        Saves the demand profile to a serialised file.

        Args:
            `filename` (str): '.pkl' filename
            `overwrite` (bool): Denotes whether to allow overwriting previous outputs
        """
        
        if not filename.endswith('.pkl'): filename += ".pkl"

        if os.path.exists(filename):
            if overwrite and self._sim != None and not self._sim._suppress_warnings:
                raise_warning(f"File '{filename}' already exists and will be overwritten.", self._sim.curr_step)
            elif not overwrite:
                desc = f"File '{filename}' already exists and cannot be overwritten."
                raise_error(FileExistsError, desc, self.curr_step)
        
        dp_dict = self.__dict__.copy()
        del dp_dict["sim"]
        del dp_dict["id"]
        del dp_dict["step_length"]

        with open(filename, "wb") as fp:
            pickle.dump(dp_dict, fp)

def _save_xml(data, filename) -> None:
    tree = et.ElementTree(data)
    et.indent(tree, space="    ")

    xml_str = et.tostring(data, encoding="utf-8").decode()
    with open(filename, "w") as fp:
        from .__init__ import __version__
        now = datetime.now()
        fp.write('<?xml version="1.0" encoding="UTF-8"?>\n\n')
        fp.write(f'<!-- generated on {now.strftime("%Y-%m-%d %H:%M:%S")} by TUD-SUMO Version {__version__} -->\n\n')
        fp.write(xml_str)
