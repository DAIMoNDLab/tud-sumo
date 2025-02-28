from random import random
from .utils import *

class VSLController:
    """ Variable Speed Limit (VSL) controller object. """

    def __init__(self, vsl_id: str, vsl_params: dict, simulation):
        """
        Args:
            `vsl_id` (str): VSL controller ID
            `vsl_params` (dict): VSL controller parameters
            `simulation` (Simulation): Simulation object
        """

        from .simulation import Simulation

        self.id = vsl_id
        self.c_type = Controller(1)

        self.sim = simulation
        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        self.activated = False
        self.activation_times = []

        self._init_params = vsl_params

        valid_params = {"type": (str, int), "geometry_ids": (list, tuple), "default_limit": (int, float)}
        error, desc = test_input_dict(vsl_params, valid_params, "VSL controller", required=["type", "geometry_ids"])
        if error != None: raise_error(error, desc, self.sim.curr_step)

        if "geometry_ids" not in vsl_params.keys():
            desc = "Geometry ID(s) are a required input for VSL controllers (key: 'geometry_ids')."
            raise_error(KeyError, desc, self.sim.curr_step)
        else: self.geometry_ids = vsl_params["geometry_ids"]

        self.geometry = {}
        for geometry_id in self.geometry_ids:
            g_type = self.sim.geometry_exists(geometry_id)
            if g_type in ["edge", "lane"]:
                self.geometry[geometry_id] = {"g_type": g_type, "nc_speed_limit": self.sim.get_geometry_vals(geometry_id, "max_speed"), "avg_speeds": []}
                self.sim.add_geometry_subscriptions(geometry_id, ["vehicle_count", "vehicle_speed"])
            else:
                desc = "Geometry ID '{0}' not found.".format(geometry_id)
                raise_error(KeyError, desc, self.sim.curr_step)

        self.speed_limit = None if "default_limit" not in vsl_params.keys() else vsl_params["default_limit"]

    def __str__(self): return "<{0}: '{1}'>".format(self.__name__, self.id)
    def __name__(self): return "VSLController"

    def __dict__(self) -> dict:

        vsl_dict = {"type": "VSL", "geometry_data": self.geometry, "init_time": self.init_time, "curr_time": self.curr_time,
                    "activation_times": self.activation_times}
        return vsl_dict
    
    def set_speed_limit(self, speed_limit: int|float|None = None) -> None:
        """
        Activates the VSL controller and sets a speed limit.
        
        Args:
            `speed_limt` (int, float, optional): New speed limit, if not given, speed limit is set to last setting
        """
        
        if self.speed_limit == None and speed_limit == None:
            desc = "Cannot change speed limit as no value given."
            raise_error(ValueError, desc, self.sim.curr_step)
        
        else:
            if speed_limit != None: self.speed_limit = speed_limit

            if self.speed_limit <= 0:
                self.deactivate()
            else:
                self.activation_times.append((self.speed_limit, self.sim.curr_step))
                for geometry_id in self.geometry.keys(): self.sim.set_geometry_vals(geometry_id, max_speed=self.speed_limit)
    
    def deactivate(self) -> None:
        """ Deactivates VSL and resets speed limits to default (defined by network). """
        
        self.activated = False
        self.speed_limit = -1
        for geometry_id in self.geometry.keys():
            self.sim.set_geometry_vals(geometry_id, max_speed=self.geometry[geometry_id]["nc_speed_limit"])
        self.activation_times.append((-1, self.sim.curr_step))
    
    def reset(self) -> None:
        """ Resets controller data. """
        
        self.speed_limits = []
        self.times = []

        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        for g_data in self.geometry.values():
            g_data["avg_speeds"] = []
    
    def update(self, keep_data: bool = True) -> None:
        """
        Updates controller at time step.

        Args:
            `keep_data` (bool): Denotes whether to update controller data
        """

        self.curr_time = self.sim.curr_step
        if keep_data:
            for g_id, g_data in self.geometry.items():
                if self.sim.get_geometry_vals(g_id, "vehicle_count") > 0:
                    g_data["avg_speeds"].append(self.sim.get_geometry_vals(g_id, "vehicle_speed"))
                else: g_data["avg_speeds"].append(-1)

class RGController:
    """ Dynamic Route Guidance (RG) controller object. """

    def __init__(self, rg_id: str|int, rg_params: dict, simulation):
        """
        Args:
            `rg_id` (str): RG controller ID
            `rg_params` (dict): RG controller parameters
            `simulation` (Simulation): Simulation object
        """

        from .simulation import Simulation

        self.id = rg_id
        self.c_type = Controller(2)

        self.sim = simulation
        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        self.activated = False
        self.activation_times = []

        self._init_params = rg_params

        valid_params = {"type": (str, int), "old_destination": str, "new_destination": str, "diversion_pct": (int, float),
                        "vehicle_type": (str, list, tuple), "detector_ids": (list, tuple), "highlight": (str, list, tuple)}
        
        error, desc = test_input_dict(rg_params, valid_params, "RG controller", required=["type", "detector_ids"])
        if error != None: raise_error(error, desc, self.sim.curr_step)
        
        self.old_target = None if "old_destination" not in rg_params.keys() else rg_params["old_destination"]
        self.target = None if "new_destination" not in rg_params.keys() else rg_params["new_destination"]

        self.g_type = None

        self.n_diverted, self.total_vehs = [], []
        self.diverted_vehs = set([])
        self.diversion_pct = 1.0 if "diversion_pct" not in rg_params.keys() else rg_params["diversion_pct"]

        if "vehicle_type" in rg_params.keys():
            self.diversion_vtypes = rg_params["vehicle_type"]
            if not isinstance(self.diversion_vtypes, (list, tuple)):
                self.diversion_vtypes = [self.diversion_vtypes]
        else: self.diversion_vtypes = None

        self.highlight_colour = None if "highlight" not in rg_params.keys() else rg_params["highlight"]
        
        if "detector_ids" not in rg_params.keys():
            desc = "Detector ID(s) are a required input for RG controllers (key: 'detector_ids')."
            raise_error(KeyError, desc, self.sim.curr_step)
        else: self.detector_ids = rg_params["detector_ids"]
        
        if not isinstance(self.detector_ids, (list, tuple)): 
            self.detector_ids = [self.detector_ids]

        self.detector_info = {}
        for detector_id in self.detector_ids:
            d_type = self.sim.detector_exists(detector_id)
            if d_type == None:
                desc = "Detector ID '{0}' not found.".format(detector_id)
                raise_error(KeyError, desc, self.sim.curr_step)
            elif d_type != 'inductionloop':
                desc = "Invalid RG detector type (must be 'inductionloop' not '{0}')".format(d_type)
                raise_error(KeyError, desc, self.sim.curr_step)
            else: self.detector_info[detector_id] = {"location": self.sim.get_detector_vals(detector_id, "position")}

    def __str__(self): return "<{0}: '{1}'>".format(self.__name__, self.id)
    def __name__(self): return "RGController"

    def __dict__(self) -> dict:
        rg_dict = {"type": "RG", "detector_ids": self.detector_ids, "init_time": self.init_time, "curr_time": self.curr_time}

        rg_dict["activation_times"] = self.activation_times
        rg_dict["n_diverted"] = self.n_diverted
        
        return rg_dict
    
    def activate(self, old_target: str|int|None = None, new_target: str|int|None = None, diversion_pct: float|None = None, highlight_colour: str|None = None) -> None:
        """
        Activates route guidance for drivers.
        
        Args:
            `old_target` (str, int, optional): If given, only drivers going to `old_target` are diverted
            `new_target` (str, int, optional): New target for driver, either edge ID or route
            `diversion_pct` (float, optional): Used to only divert a percent of drivers (selected randomly)
            `highlight_colour` (str, optional): If given, diverted drivers are coloured
        """
        
        if self.target == None and new_target == None:
            desc = "Cannot activate as no target given."
            raise_error(ValueError, desc, self.sim.curr_step)
        else:
            if not self.activated:
                if new_target != None: self.target = new_target

                if self.sim.geometry_exists(self.target) == "edge": self.g_type = "edge"
                elif self.sim.route_exists(self.target) != None: self.g_type = "route"
                else:
                    desc = "Route or edge ID '{1}' not found".format(self.target)
                    raise_error(KeyError, desc, self.sim.curr_step)

                self.activated = True
            
                if diversion_pct != None: self.diversion_pct = diversion_pct
                if old_target != None: self.old_target = old_target

                if self.old_target != None: activation_data = ((self.old_target, self.target), self.diversion_pct, self.curr_time)
                else: activation_data = (self.target, self.diversion_pct, self.curr_time)
                self.activation_times.append(activation_data)

                if highlight_colour != None: self.highlight_colour = highlight_colour
            elif not self.sim._suppress_warnings: raise_warning("RG controller '{0}' is already activated.".format(self.id), self.sim.curr_step)

    def deactivate(self) -> None:
        """ Deactivate the controller and stop diverting drivers. """
        
        if not self.activated:
            if not self.sim._suppress_warnings: raise_warning("Controller already deactivated.", self.sim.curr_step)
        else:
            self.activated = False
            self.activation_times.append((-1, -1, self.curr_time))
    
    def reset(self) -> None:
        """ Resets controller data. """
                
        self.n_diverted, self.total_vehs = [], []
        self.diverted_vehs = set([])
        self.activation_times = []

        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step
    
    def update(self, keep_data: bool = True) -> None:
        """
        Performs route guidance on vehicles passing over the detector.

        Args:
            `keep_data` (bool): Denotes whether to update RG data
        """

        self.curr_time = self.sim.curr_step
        n_diverted, total_vehs = 0, 0

        if self.activated:
            all_vehs = self.sim.get_last_step_detector_vehicles(self.detector_ids, vehicle_types=self.diversion_vtypes, flatten=True)
            for veh_id in all_vehs:
                total_vehs += 1
                if veh_id not in self.diverted_vehs and random() <= self.diversion_pct:
                    if self.old_target == None or self.sim.get_vehicle_vals("new_destination") == self.old_target:
                        if self.g_type == "edge":
                            self.sim.set_vehicle_vals(veh_id, destination=self.target, colour=self.highlight_colour)
                        elif self.g_type == "route":
                            self.sim.set_vehicle_vals(veh_id, route_id=self.target, colour=self.highlight_colour)

                        n_diverted += 1
                        self.diverted_vehs.add(veh_id)
        if keep_data:
            self.total_vehs.append(total_vehs)
            self.n_diverted.append(n_diverted)