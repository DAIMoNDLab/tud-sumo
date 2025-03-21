import csv, datetime
import xml.etree.ElementTree as gfg  
from .utils import *

class DemandProfile:

    def __init__(self, simulation, name: str|None = None):
        from .simulation import Simulation

        self.sim = simulation
        self.sim._demand_profiles.append(self)

        self._demand_headers = ["routing", "step_range", "veh/step", "vehicle_types", "vehicle_type_dists", "init_speed", "origin_lane", "origin_pos", "insertion_sd"]
        self._demand_arrs = []
        self._demand_files = []

        self._vehicle_types = {}

        self.name = name

    def __name__(self): return "DemandProfile"

    def is_complete(self) -> bool:
        if len(self._demand_arrs) == 0: return True
        else:
            end_times = [arr[1][1] for arr in self._demand_arrs]
            return max(end_times) > self.sim.curr_step
        
    def add_vehicle_type(self, vehicle_type_id: str, vehicle_class: str="passenger", colour: str|list|tuple|None = None, length: int|float|None = None, width: int|float|None = None, height: int|float|None = None, mass: int|float|None = None, speed_factor: int|float|None = None, speed_dev: int|float|None = None, min_gap: int|float|None = None, acceleration: int|float|None = None, deceleration: int|float|None = None, tau: int|float|None = None, max_lateral_speed: int|float|None = None, gui_shape: str|None = None) -> None:
        """
        Adds a new vehicle type to the simulation.

        Args:
            `vehicle_type_id` (str): ID for the new vehicle type
            `vehicle_class` (str): Vehicle class (defaults to passenger)
            `colour` (str, list, tuple, optional): Vehicle type colour, either hex code or list of rgb/rgba values
            `length` (int, float, optional): Length of vehicle in metres/feet
            `width` (int, float, optional): Width of vehicle in metres/feet
            `height` (int, float, optional): Height of vehicle in metres/feet
            `mass` (int, float, optional): Mass of vehicle in kilograms
            `speed_factor` (int, float, optional): Vehicle type multiplier for lane speed limits
            `speed_dev` (int, float, optional): Vehicle type deviation of the speed factor
            `min_gap` (int, float, optional): Minimum gap after leader (m)
            `acceleration` (int, float, optional): Vehicle type acceleration ability (m/s^2)
            `deceleration` (int, float, optional): Vehicle type deceleration ability (m/s^2)
            `tau` (int, float, optional): Car following model parameter
            `max_lateral_speed` (int, float, optional): Maximum lateral speed (m/s)
            `gui_shape` (str, optional): Vehicle shape in GUI (defaults to vehicle class name)
        """

        values = [vehicle_class, colour, length, height, mass, speed_factor, speed_dev, min_gap, acceleration, deceleration, tau, max_lateral_speed, gui_shape]
        labels = ['vClass', 'color', 'length', 'height', 'mass', 'speedFactor', 'speedDev', 'minGap', 'accel', 'decel', 'tau', 'maxSpeedLat', 'guiShape']
        
        vehicle_type_data = {label: value for label, value in zip(labels, values) if value != None}

        self.sim.add_vehicle_type(vehicle_type_id, vehicle_class, colour, length,
                                  width, height, mass, speed_factor, speed_dev,
                                  min_gap, acceleration, deceleration, tau,
                                  max_lateral_speed, gui_shape)
        
        if "color" in vehicle_type_data:
            if vehicle_type_data["color"] not in sumo_colours:
                colour = colour_to_rgba(vehicle_type_data["color"], self.sim.curr_step)
                if isinstance(colour, (list, tuple)): colour = ",".join([str(val) for val in colour])
                vehicle_type_data["color"] = colour

        self._vehicle_types[vehicle_type_id] = vehicle_type_data
        
    def create_route_file(self, filename: str|None = None, add_to_cfg: bool = True) -> None:
        """
        Create a SUMO '.rou.xml' file from the demand added to this profile.

        Args:
            `filename` (str, optional): Filename for route file (uses profile name if not given)
            `add_to_cfg` (bool): Denotes whether to add route file to the SUMO config file (if used)
        """

        if len(self._vehicle_types) == 0 and len(self._demand_arrs) == 0:
            desc = "Cannot create route file (no vehicle types or demand data found)."
            raise_error(KeyError, desc, self.sim.curr_step)
        
        if filename == None:
            if self.name == None:
                desc = "Cannot create route file (no DemandProfile name or filename given)."
                raise_error(ValueError, desc, self.sim.curr_step)
            else: filename = self.name
        
        if not filename.endswith(".rou.xml"): filename += ".rou.xml"

        root = gfg.Element("routes", attrib={"xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance",
                                             "xsi:noNamespaceSchemaLocation": "http://sumo.dlr.de/xsd/routes_file.xsd"})

        if len(self._vehicle_types) > 0:
            root.append(gfg.Comment(" VTypes "))

            for vehicle_type_id, vehicle_type_data in self._vehicle_types.items():
                attributes = {"id": vehicle_type_id}
                attributes.update(vehicle_type_data)
                root.append(gfg.Element("vType", attrib=attributes))

        if len(self._demand_arrs) > 0:
            root.append(gfg.Comment(" Vehicles, persons and containers (sorted by depart) "))

            idx = 0
            for demand_arr in self._demand_arrs:
                
                origin, destination = demand_arr[0]
                start, end = demand_arr[1]
                vehs_per_hour = demand_arr[2]
                vehicle_types = demand_arr[3]
                vehicle_type_dists = demand_arr[4]

                attributes = {"id": "", "type": "", "begin": str(start), "from": origin, "to": destination, "end": str(end),
                            "vehsPerHour": 0, "departLane": str(demand_arr[6]), "departPos": str(demand_arr[7]), "departSpeed": str(demand_arr[5])}

                if isinstance(vehicle_types, str): vehicle_types = [vehicle_types]
                if vehicle_type_dists == None: vehicle_type_dists = [1] * len(vehicle_types)
                
                for vehicle_type, type_dist in zip(vehicle_types, vehicle_type_dists):
                    attributes["id"] = f"{self.name}_{idx}" if self.name != None else f"flow_{idx}"
                    attributes["type"] = vehicle_type
                    attributes["vehsPerHour"] = str(vehs_per_hour * type_dist)

                    root.append(gfg.Element("flow", attrib=attributes))
                    idx += 1

        _save_xml(root, filename)

    def load_demand(self, csv_file: str) -> None:
        """
        Loads OD demand from a '_.csv_' file. The file must contain an 'origin/destination' or 'route_id' column(s), 'start_time/end_time' or 'start_step/end_step' columns(s) and a 'demand/number' column.
        
        Args:
            `csv_file` (str): Demand file location
        """

        csv_file = validate_type(csv_file, str, "demand file", self.sim.curr_step)
        if csv_file.endswith(".csv"):
            if os.path.exists(csv_file):
                with open(csv_file, "r") as fp:

                    valid_cols = ["origin", "destination", "route_id", "start_time", "end_time", "start_step", "end_step",
                                  "demand", "number", "vehicle_types", "vehicle_type_dists", "initial_speed", "origin_lane", "origin_pos", "insertion_sd"]
                    demand_idxs = {}
                    reader = csv.reader(fp)
                    for idx, row in enumerate(reader):

                        # First, get index of (valid) columns to read data correctly and
                        # store in demand_idxs dict
                        if idx == 0:

                            if len(set(row) - set(valid_cols)) != 0:
                                desc = "Invalid demand file (unknown columns '{0}').".format("', '".join(list(set(row) - set(valid_cols))))
                                raise_error(KeyError, desc, self.sim.curr_step)

                            if "route_id" in row: demand_idxs["route_id"] = row.index("route_id")
                            elif "origin" in row and "destination" in row:
                                demand_idxs["origin"] = row.index("origin")
                                demand_idxs["destination"] = row.index("destination")
                            else:
                                desc = "Invalid demand file (no routing values, must contain 'route_id' or 'origin/destination')."
                                raise_error(KeyError, desc, self.sim.curr_step)
                            
                            if "start_time" in row and "end_time" in row:
                                demand_idxs["start_time"] = row.index("start_time")
                                demand_idxs["end_time"] = row.index("end_time")
                            elif "start_step" in row and "end_step" in row:
                                demand_idxs["start_step"] = row.index("start_step")
                                demand_idxs["end_step"] = row.index("end_step")
                            else:
                                desc = "Invalid demand file (no time values, must contain 'start_time/end_time' or 'start_step/end_step')."
                                raise_error(KeyError, desc, self.sim.curr_step)

                            if "demand" in row: demand_idxs["demand"] = row.index("demand")
                            elif "number" in row: demand_idxs["number"] = row.index("number")
                            else:
                                desc = "Invalid demand file (no demand values, must contain 'demand/number')."
                                raise_error(KeyError, desc, self.sim.curr_step)

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

                        else:

                            # Use demand_idx dict to get all demand data from the correct indices

                            if "route_id" in demand_idxs: routing = row[demand_idxs["route_id"]]
                            else: routing = (row[demand_idxs["origin"]], row[demand_idxs["destination"]])

                            if "start_time" in demand_idxs: 
                                step_range = (int(row[demand_idxs["start_time"]]) / self.sim.step_length, int(row[demand_idxs["end_time"]]) / self.sim.step_length)
                            else: step_range = (int(row[demand_idxs["start_step"]]), int(row[demand_idxs["end_step"]]))

                            step_range = [int(val) for val in step_range]

                            # Convert to flow in vehicles/hour if using 'number'
                            if "number" in demand_idxs: demand = int(row[demand_idxs["number"]]) / convert_units(step_range[1] - step_range[0], "steps", "hours", self.sim.step_length)
                            else: demand = float(row[demand_idxs["demand"]])

                            if "vehicle_types" in demand_idxs:
                                vehicle_types = row[demand_idxs["vehicle_types"]].split(",")
                                if len(vehicle_types) == 1: vehicle_types = vehicle_types[0]
                            else:
                                # If vehicle types not defined, use SUMO default vehicle type (standard car)
                                vehicle_types = "DEFAULT_VEHTYPE"
                                if "vehicle_type_dists" in demand_idxs:
                                    desc = "vehicle_type_dists given without vehicle_types."
                                    raise_error(ValueError, desc, self.sim.curr_step)


                            if isinstance(vehicle_types, (list, tuple)):
                                if "vehicle_type_dists" in demand_idxs:
                                    vehicle_type_dists = row[demand_idxs["vehicle_type_dists"]].split(",")
                                    if len(vehicle_type_dists) != len(vehicle_types):
                                        desc = "Invalid vehicle_type_dists '[{0}]' (must be same length as vehicle_types '{1}').".format(", ".join(vehicle_type_dists), len(vehicle_types))
                                        raise_error(ValueError, desc, self.sim.curr_step)
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

                            self.add_demand(routing, step_range, demand, vehicle_types, vehicle_type_dists, initial_speed, origin_lane, origin_pos, insertion_sd)
                            
            else:
                desc = "Demand file '{0}' not found.".format(csv_file)
                raise_error(FileNotFoundError, desc, self.sim.curr_step)
        else:
            desc = "Invalid demand file '{0}' format (must be '.csv').".format(csv_file)
            raise_error(ValueError, desc, self.sim.curr_step)

        self._demand_files.append(csv_file)

    def add_demand(self, routing: str|list|tuple, step_range: list|tuple, demand: int|float, vehicle_types: str|list|tuple|None = None, vehicle_type_dists: list|tuple|None = None, initial_speed: str|int|float = "max", origin_lane: str|int|float = "best", origin_pos: str|int = "base", insertion_sd: float = 0.333) -> None:
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
        """

        routing = validate_type(routing, (str, list, tuple), "routing", self.sim.curr_step)
        if isinstance(routing, str) and not self.sim.route_exists(routing):
            desc = "Unknown route ID '{0}'.".format(routing)
            raise_error(KeyError, desc, self.sim.curr_step)
        elif isinstance(routing, (list, tuple)):
            routing = validate_list_types(routing, (str, str), True, "routing", self.sim.curr_step)
            if not self.sim.is_valid_path(routing):
                desc = "No route between edges '{0}' and '{1}'.".format(routing[0], routing[1])
                raise_error(ValueError, desc, self.sim.curr_step)

        step_range = validate_list_types(step_range, ((int), (int)), True, "step_range", self.sim.curr_step)
        if step_range[1] < step_range[0] or step_range[1] < self.sim.curr_step:
            desc = "Invalid step_range '{0}' (must be valid range and end > current step)."
            raise_error(ValueError, desc, self.sim.curr_step)

        if vehicle_types != None:
            vehicle_types = validate_type(vehicle_types, (str, list, tuple), param_name="vehicle_types", curr_sim_step=self.sim.curr_step)
            if isinstance(vehicle_types, (list, tuple)):
                vehicle_types = validate_list_types(vehicle_types, str, param_name="vehicle_types", curr_sim_step=self.sim.curr_step)
                for type_id in vehicle_types:
                    if not self.sim.vehicle_type_exists(type_id):
                        desc = "Unknown vehicle type ID '{0}' in vehicle_types.".format(type_id)
                        raise_error(KeyError, desc, self.sim.curr_step)
            elif not self.sim.vehicle_type_exists(vehicle_types):
                desc = "Unknown vehicle_types ID '{0}' given.".format(vehicle_types)
                raise_error(KeyError, desc, self.sim.curr_step)
        else: vehicle_types = "DEFAULT_VEHTYPE"
        
        if vehicle_type_dists != None and vehicle_types == None:
            desc = "vehicle_type_dists given, but no vehicle types."
            raise_error(ValueError, desc, self.sim.curr_step)
        elif vehicle_type_dists != None and isinstance(vehicle_types, str):
            desc = "Invalid vehicle_type_dists (vehicle_types is a single type ID, so no distribution)."
            raise_error(ValueError, desc, self.sim.curr_step)
        elif vehicle_type_dists != None:
            vehicle_type_dists = validate_list_types(vehicle_type_dists, float, param_name="vehicle_type_dists", curr_sim_step=self.sim.curr_step)
            if len(vehicle_type_dists) != len(vehicle_types):
                desc = "Invalid vehicle_type_dists (must be same length as vehicle_types, {0} != {1}).".format(len(vehicle_type_dists), len(vehicle_types))
                raise_warning(ValueError, desc, self.sim.curr_step)

        insertion_sd = validate_type(insertion_sd, (int, float), "insertion_sd", self.sim.curr_step)

        self.sim._manual_flow = True
        self._demand_arrs.append([routing, step_range, demand, vehicle_types, vehicle_type_dists, initial_speed, origin_lane, origin_pos, insertion_sd])

        self._demand_arrs = sorted(self._demand_arrs, key=lambda row: row[1][0])

    def add_demand_function(self, routing: str|list|tuple, step_range: list|tuple, demand_function, parameters: dict|None = None, vehicle_types: str|list|tuple|None = None, vehicle_type_dists: list|tuple|None = None, initial_speed: str|int|float = "max", origin_lane: str|int|float = "best", origin_pos: str|int = "base", insertion_sd: float = 0.333) -> None:
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
        """
        
        step_range = validate_list_types(step_range, ((int), (int)), True, "step_range", self.sim.curr_step)
        if step_range[1] < step_range[0] or step_range[1] < self.sim.curr_step:
            desc = "Invalid step_range '{0}' (must be valid range and end > current step)."
            raise_error(ValueError, desc, self.sim.curr_step)
        
        # Step through start -> end, calculate flow value 
        for step_no in range(step_range[0], step_range[1]):
            
            params = {"step": step_no}
            if parameters != None: params.update(parameters)
            demand_val = demand_function(**params)

            # Outputs must be a number
            if not isinstance(demand_val, (int, float)):
                desc = "Invalid demand function (output must be type 'int', not '{0}').".format(type(demand_val).__name__)
                raise_error(TypeError, desc, self.sim.curr_step)
            
            # Skip if equal to or less than 0
            if demand_val <= 0: continue

            self.add_demand(routing, (step_no, step_no), demand_val, vehicle_types, vehicle_type_dists, initial_speed, origin_lane, origin_pos, insertion_sd)

def _save_xml(data, filename) -> None:
    tree = gfg.ElementTree(data)
    gfg.indent(tree, space="    ")

    xml_str = gfg.tostring(data, encoding="utf-8").decode()
    with open(filename, "w") as fp:
        from .__init__ import __version__
        now = datetime.now()
        fp.write('<?xml version="1.0" encoding="UTF-8"?>\n\n')
        fp.write(f'<!-- generated on {now.strftime("%Y-%m-%d %H:%M:%S")} by TUD-SUMO Version {__version__} -->\n\n')
        fp.write(xml_str)
