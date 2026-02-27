import traci
from random import random
from .utils import *

def _add_vehicle(self,
                vehicle_id: str,
                vehicle_type: str,
                routing: str | list | tuple,
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

    if self.vehicle_exists(vehicle_id):
        desc = "Invalid vehicle_id given '{0}' (must be unique).".format(vehicle_id)
        raise_error(ValueError, desc, self.curr_step)
    
    origin_lane = validate_type(origin_lane, (str, int), "origin_lane", self.curr_step)
    initial_speed = validate_type(initial_speed, (str, int, float), "initial_speed", self.curr_step)
    if isinstance(initial_speed, str) and initial_speed not in ["max", "random"]:
        desc = "Invalid initial_speed string given '{0}' (must be ['_max_' | '_random_']).".format(initial_speed, type(initial_speed).__name__)
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
            _vehicles_in(self, vehicle_id)

        else:
            desc = "Invalid routing given '[{0}]' (must have shape (2x1)).".format(",".join(routing))
            raise_error(TypeError, desc, self.curr_step)

def _remove_vehicles(self, vehicle_ids: str | list | tuple) -> None:
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
            _vehicles_out(self, vehicle_id, is_removed=True)
        else:
            desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)

def _add_vehicle_type(self,
                        vehicle_type_id: str,
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
    
    if self.vehicle_type_exists(vehicle_type_id):
        desc = f"Cannot create vehicle type (ID '{vehicle_type_id}' already exists)."
        raise_error(KeyError, desc, self.curr_step)

    traci.vehicletype.copy('DEFAULT_VEHTYPE', vehicle_type_id)
    self._added_vehicle_types.add(vehicle_type_id)

    if self._gui: gui_shape = vehicle_class

    self.set_vehicle_type_vals(vehicle_type_id, vehicle_class=vehicle_class, colour=colour,
                                length=length, width=width, height=height, max_speed=max_speed,
                                speed_factor=speed_factor, speed_dev=speed_dev, min_gap=min_gap,
                                max_acceleration=max_acceleration, max_deceleration=max_deceleration,
                                headway=headway, imperfection=imperfection, max_lateral_speed=max_lateral_speed,
                                emission_class=emission_class, gui_shape=gui_shape)

def _stop_vehicle(self, vehicle_id: str, duration: int | float | None = None, lane_idx: int | None = None, pos: int | float | None = None) -> None:
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

    data = self.get_vehicle_vals(vehicle_id, ["next_edge_id", "lane_idx"])
    if data["next_edge_id"] == None:
        desc = f"Vehicle '{vehicle_id}' has no next edge ID and cannot stop."
        raise_error(ValueError, desc, self.curr_step)

    edge_data = self.get_geometry_vals(data["next_edge_id"], ["n_lanes", "length"]) 

    if pos == None: pos = random()
    elif not isinstance(pos, (int, float)):
        desc = f"Invalid pos value '{pos}' (must be [int | float], not '{type(pos).__name__}')."
        raise_error(TypeError, desc, self.curr_step)
    elif pos < 0 or pos > 1:
        desc = f"Invalid pos value '{pos}' (must be 0 <= pos < 1)."
        raise_error(ValueError, desc, self.curr_step)

    if lane_idx == None:
        lane_idx = min(data["lane_idx"], edge_data["n_lanes"] - 1)
    elif not isinstance(lane_idx, int):
        desc = f"Invalid lane_idx value '{lane_idx}' (must be int, not '{type(lane_idx).__name__}')."
        raise_error(TypeError, desc, self.curr_step)
    elif lane_idx < 0 or lane_idx >= edge_data["n_lanes"]:
        desc = f"Invalid lane_idx value '{lane_idx}' (must be 0 <= lane_idx < {edge_data['n_lanes']})."
        raise_error(ValueError, desc, self.curr_step)

    pos = pos * convert_units(edge_data["length"], self._l_dist_unit, "metres")
    if duration == None: traci.vehicle.setStop(vehicle_id, data["next_edge_id"], pos=pos, laneIndex=lane_idx)
    else: traci.vehicle.setStop(vehicle_id, data["next_edge_id"], pos=pos, laneIndex=lane_idx, duration=float(duration))
    
    self._stopped_vehicles.add(vehicle_id)

def _resume_vehicle(self, vehicle_id: str) -> None:
    """
    Resumes a previously stopped vehicle.
    
    Args:
        `vehicle_id` (str): Vehicle ID
    """

    if not self.vehicle_exists(vehicle_id):
        desc = f"Vehicle with ID '{vehicle_id}' not found."
        raise_error(KeyError, desc, self.curr_step)

    if vehicle_id not in self._stopped_vehicles and not self._suppress_warnings:
        raise_warning(f"Vehicle '{vehicle_id}' is not stopped.")

    else:
        if self.get_vehicle_vals(vehicle_id, "is_stopped"):
            traci.vehicle.resume(vehicle_id)
            self._stopped_vehicles.remove(vehicle_id)
        
        elif not self._suppress_warnings:
            raise_warning(f"Vehicle '{vehicle_id}' has not stopped yet (cannot resume).")


# --- VEHICLE FUNCTIONS ---

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

def _vehicles_in(self, vehicle_ids: str | list | tuple, is_added: bool = False) -> None:
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
        
def _vehicles_out(self, vehicle_ids: str | list | tuple, is_removed: bool = False) -> None:
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