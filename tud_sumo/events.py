import json, math, os, pickle as pkl
from copy import deepcopy
from random import random, choices, choice
from .utils import *

class EventScheduler:
    """ Event manager that stores, tracks and implements events. """
    def __init__(self, sim):
        """
        Args:
            `sim` (Simulation): Simulation object
        """
        from .simulation import Simulation

        self.sim = sim
        self.scheduled_events = {}
        self.active_events = {}
        self.completed_events = {}

    def __dict__(self):
        schedule_dict = {}

        for status, event_dict in zip(["scheduled", "active", "completed"], [self.scheduled_events, self.active_events, self.completed_events]):
            if len(event_dict) > 0:
                schedule_dict[status] = {}
                for event_obj in event_dict.values():
                    event_dict = event_obj.__dict__()
                    del event_dict["id"]
                    schedule_dict[status][event_obj.id] = event_dict

        return schedule_dict
    
    def __str__(self): return "<{0}>".format(self.__name__)
    def __name__(self): return "EventScheduler"

    def get_event_ids(self, event_statuses: str | list | tuple | None = None) -> list:
        """
        Gets all event IDs, or those of a specific status.

        Args:
            `event_statuses` (str, list, tuple, optional): Event status or list of statuses from ['_scheduled_' | '_active_' | '_completed_'] (defaults to all)

        Returns:
            list: List of event IDs
        """

        if event_statuses == None: event_statuses = ["scheduled", "active", "completed"]
        elif isinstance(event_statuses, str): event_statuses = [event_statuses]

        event_ids = []
        if "scheduled" in event_statuses:
            event_ids += list(self.scheduled_events.keys())
        if "active" in event_statuses:
            event_ids += list(self.active_events.keys())
        if "completed" in event_statuses:
            event_ids += list(self.completed_events.keys())

        return event_ids
    
    def get_events(self, event_statuses: str | list | tuple | None = None) -> list:
        """
        Gets all events, or those of a specific status.

        Args:
            `event_statuses` (str, list, tuple, optional): Event status or list of statuses from ['_scheduled_' | '_active_' | '_completed_'] (defaults to all)

        Returns:
            list: List of events
        """
                
        if event_statuses == None: event_statuses = ["scheduled", "active", "completed"]
        elif isinstance(event_statuses, str): event_statuses = [event_statuses]

        events = []
        if "scheduled" in event_statuses:
            events += list(self.scheduled_events.values())
        if "active" in event_statuses:
            events += list(self.active_events.values())
        if "completed" in event_statuses:
            events += list(self.completed_events.values())

        return events
    
    def get_event(self, event_id):

        if event_id in self.scheduled_events:
            return self.scheduled_events[event_id]
        elif event_id in self.active_events:
            return self.active_events[event_id]
        elif event_id in self.completed_events:
            return self.completed_events[event_id]
        else:
            desc = f"Unrecognised event ID '{event_id}'."
            raise_error(KeyError, desc, self.sim.curr_step)

    def remove_event(self, event_id):

        if event_id in self.scheduled_events:
            del self.scheduled_events[event_id]
        elif event_id in self.active_events:
            event = self.get_event(event_id)
            event.terminate()
        elif event_id in self.completed_events:
            del self.completed_events[event_id]
        else:
            desc = f"Unrecognised event ID '{event_id}'."
            raise_error(KeyError, desc, self.sim.curr_step)
    
    def add_events(self, events) -> None:
        """
        Add events to the schedule.

        Args:
            `events` (Event, str, list, tuple, dict): Event, list of events, dictionary of event parameters or path to parameters file
        """

        if isinstance(events, Event): self.scheduled_events[events.id] = events

        else:
            if isinstance(events, dict):
                for event_id, event_params in events.items():
                    if isinstance(event_params, Event): self.scheduled_events[event_id] = event_params
                    elif isinstance(event_params, dict): self.scheduled_events[event_id] = Event(event_id, event_params, self.sim)
                    else:
                        desc = "Invalid event_params dict (must contain [dict | Event], not '{0}').".format(type(event_params).__name__)
                        raise_error(TypeError, desc, self.sim.curr_step)

            elif isinstance(events, str):
                if events.endswith(".json"): r_class, r_mode = json, "r"
                elif events.endswith(".pkl"): r_class, r_mode, = pkl, "rb"
                else:
                    desc = "Event junction parameters file '{0}' (must be '.json' file).".format(events)
                    raise_error(ValueError, desc, self.sim.curr_step)

                if os.path.exists(events):
                    with open(events, r_mode) as fp:
                        events = r_class.load(fp)
                    for event_id, event_params in events.items():
                        self.scheduled_events[event_id] = Event(event_id, event_params, self.sim)
                else:
                    desc = "Event parameters file '{0}' not found.".format(events)
                    raise_error(FileNotFoundError, desc, self.sim.curr_step)
                
            elif hasattr(events, "__iter__") and all([isinstance(event, Event) for event in events]):
                for event in events: self.scheduled_events[event.id] = event
            
            else:
                desc = "Invalid event_params (must be [dict | filepath | (Event)], not '{0}').".format(type(events).__name__)
                raise_error(TypeError, desc, self.sim.curr_step)

    def update_events(self):
        """ Update & implement all events. """
        
        scheduled_event_ids = list(self.scheduled_events.keys())
        for event_id in scheduled_event_ids:
            event = self.scheduled_events[event_id]
            if event.start_time <= self.sim.curr_step:
                event.start()
                del self.scheduled_events[event_id]
                self.active_events[event_id] = event

        active_events = list(self.active_events.keys())
        for event_id in active_events:
            event = self.active_events[event_id]
            if event.is_active():
                event.run()
            else:
                del self.active_events[event_id]
                self.completed_events[event_id] = event

    def get_event_status(self, event_id: str) -> str | None:
        """
        Get the status of an event, by its ID.
        
        Args:
            `event_id` (str): Event ID
        
        Returns:
            (str, optional): Event status ['_scheduled_' | '_active_' | '_completed_'], or `None` if it does not exist
        """

        if event_id in self.scheduled_events.keys(): return "scheduled"
        elif event_id in self.active_events.keys(): return "active"
        elif event_id in self.completed_events.keys(): return "completed"
        else: return None

class Event:
    """ A scheduled event, where effects are carried out for a specified amount of time. """

    def __init__(self, event_id: str, event_params: str | dict, simulation):
        """
        Args:
            `event_id` (str): Event ID
            `event_params` (str, dict): Event parameters dictionary or path to parameters file
            `simulation` (Simulation): Simulation object
        """

        self.id = event_id
        self.sim = simulation

        self._init_params = event_params

        if not isinstance(event_params, dict) and not isinstance(event_params, str):
            desc = "Invalid event_params (must be [dict | filepath (str)], not '{0}').".format(type(event_params).__name__)
            raise_error(TypeError, desc, self.sim.curr_step)
        elif isinstance(event_params, str):
            if event_params.endswith(".json"): r_class, r_mode = json, "r"
            elif event_params.endswith(".pkl"): r_class, r_mode = pkl, "rb"
            else:
                desc = "Event junction parameters file '{0}' (must be '.json' or '.pkl' file).".format(event_params)
                raise_error(ValueError, desc, self.sim.curr_step)

            if os.path.exists(event_params):
                with open(event_params, r_mode) as fp:
                    event_params = r_class.load(fp)
            else:
                desc = "Event parameters file '{0}' not found.".format(event_params)
                raise_error(FileNotFoundError, desc, self.sim.curr_step)

        valid_params = {"start_time": (int, float), "start_step": (int, float), "end_time": (int, float),
                        "end_step": (int, float), "edges": dict, "vehicles": dict}
        error, desc = test_input_dict(event_params, valid_params, "event")
        if error != None: raise_error(error, desc, self.sim.curr_step)

        if "start_time" in event_params.keys(): self.start_time = event_params["start_time"] / self.sim.step_length
        elif "start_step" in event_params.keys(): self.start_time = event_params["start_step"]
        else:
            desc = "Event 'start_time' or 'start_step' are not given and one is required."
            raise_error(KeyError, desc, self.sim.curr_step)

        if "end_time" in event_params.keys():
            self.end_time = event_params["end_time"] / self.sim.step_length
        elif "end_step" in event_params.keys():
            self.end_time = event_params["end_step"]
        else:
            self.end_time = math.inf

        #if "edges" not in event_params.keys() and "vehicles" not in event_params.keys():
        #    desc = "Neither 'edges' or 'vehicles' parameters are given and one or both is required."
        #    raise_error(KeyError, desc, self.sim.curr_step)
        
        if "edges" in event_params.keys():
            edge_params = event_params["edges"]
            valid_params = {"actions": dict, "edge_ids": (list, tuple), "r_effects": bool}
            error, desc = test_input_dict(edge_params, valid_params, dict_name="edge", required=["actions", "edge_ids"])
            if error != None: raise_error(error, desc, self.sim.curr_step)
                                          
            self.edge_ids = edge_params["edge_ids"]
            self.e_actions, self.e_base = edge_params["actions"], {}

            if "r_effects" in edge_params:
                self.e_r_effects = edge_params["r_effects"]
            else: self.e_r_effects = False

        else: self.e_actions = None

        if "vehicles" in event_params.keys():
            veh_params = event_params["vehicles"]
            valid_params = {"actions": dict, "locations": (list, tuple), "vehicle_ids": (list, tuple), "effect_duration": (int, float),
                            "vehicle_types": (list, tuple), "effect_probability": (int, float), "vehicle_limit": int, "highlight": bool,
                            "remove_affected_vehicles": bool, "speed_safety_checks": bool, "lc_safety_checks": bool, "r_effects": bool,
                            "location_only": bool, "force_end": bool}
            
            error, desc = test_input_dict(veh_params, valid_params, "vehicle")
            if error != None: raise_error(error, desc, self.sim.curr_step)

            self.locations, self.vehicle_ids = None, None
            if "locations" in veh_params:
                self.locations = veh_params["locations"]
            elif "vehicle_ids" in veh_params:
                self.vehicle_ids = veh_params["vehicle_ids"]
            else:
                desc = "Event 'locations' or 'vehicle_ids' are not given and one is required."
                raise_error(KeyError, desc, self.sim.curr_step)

            self.v_effect_dur = math.inf if "effect_duration" not in veh_params.keys() else veh_params["effect_duration"] / self.sim.step_length
            self.v_actions, self.v_base, self.affected_vehicles = {} if "actions" not in veh_params else veh_params["actions"], {}, {}

            if "vehicle_types" in veh_params.keys():
                self.vehicle_types = veh_params["vehicle_types"]
            else: self.vehicle_types = None

            self.v_prob = 1 if "effect_probability" not in veh_params.keys() else veh_params["effect_probability"]
            self.ignored_vehs = []
            
            if "vehicle_limit" in veh_params.keys():
                self.vehicle_limit, self.total_affected_vehicles = veh_params["vehicle_limit"], 0
            else:
                self.vehicle_limit, self.total_affected_vehicles = math.inf, 0

            self.highlight = None if "highlight" not in veh_params.keys() else veh_params["highlight"]

            if "remove_affected_vehicles" in veh_params:
                self.remove_affected_vehicles = veh_params["remove_affected_vehicles"]
            else: self.remove_affected_vehicles = False

            if "speed_safety_checks" in veh_params:
                self.assert_speed_safety = veh_params["speed_safety_checks"]
            else: self.assert_speed_safety = True

            if "lc_safety_checks" in veh_params:
                self.assert_lc_safety = veh_params["lc_safety_checks"]
            else: self.assert_lc_safety = True

            for data_key in ["acceleration", "lane_idx"]:
                if data_key in self.v_actions.keys():
                    self.v_actions[data_key] = (self.v_actions[data_key], self.v_effect_dur)

            if "r_effects" in veh_params:
                self.v_r_effects = veh_params["r_effects"]
            else: self.v_r_effects = False

            if "location_only" in veh_params:
                self.location_only = veh_params["location_only"]
            else: self.location_only = False

            if "force_end" in veh_params:
                self.force_end = veh_params["force_end"]
            else: self.force_end = False

        else: self.v_actions = None

    def __dict__(self):

        event_dict = {"id": self.id, "start_time": self.start_time, "end_time": self.end_time}
        if self.e_actions != None:
            event_dict["edges"] = {"edge_ids": self.edge_ids, "actions": self.e_actions}
        if self.v_actions != None:
            event_dict["vehicles"] = {"locations": self.locations, "actions": self.v_actions,
                                     "effect_duration": self.v_effect_dur, "n_affected": self.total_affected_vehicles}
            
        return event_dict
    
    def __str__(self): return "<{0}: '{1}'>".format(self.__name__, self.id)
    def __name__(self): return "Event"

    def start(self):
        """ Start the scheduled event. """

        if self.e_actions != None:
            for edge_id in self.edge_ids:

                base_vals = self.sim.get_geometry_vals(edge_id, list(self.e_actions.keys()))
                if len(list(self.e_actions.keys())) == 1:
                    action_key = list(self.e_actions.keys())[0]
                    self.e_base[edge_id] = {}
                    self.e_base[edge_id][action_key] = base_vals
                else:
                    self.e_base[edge_id] = base_vals

                if self.e_r_effects: action_dict = {a_key: self.e_base[edge_id][a_key] * self.e_actions[a_key] if isinstance(self.e_actions[a_key], (int, float)) else self.e_actions[a_key] for a_key in self.e_actions.keys()}
                else: action_dict = self.e_actions

                self.sim.set_geometry_vals(edge_id, **action_dict)

                self.e_effects_active = True
                if "allowed" in self.e_base[edge_id].keys():
                    self.e_base[edge_id]["disallowed"] = self.e_actions["allowed"]
                    del self.e_base[edge_id]["allowed"]
                if "disallowed" in self.e_base[edge_id].keys():
                    self.e_base[edge_id]["allowed"] = self.e_actions["disallowed"]
                    del self.e_base[edge_id]["disallowed"]

        self.e_effects_active = self.e_actions != None
        self.v_effects_active = self.v_actions != None

    def run(self):
        """ Implement any event effects. """

        # Reset affected edges
        if self.e_actions != None:
            if self.end_time <= self.sim.curr_step and self.e_effects_active:
                for edge_id in self.edge_ids:
                    self.sim.set_geometry_vals(edge_id, **self.e_base[edge_id])
                self.e_effects_active = False

        if self.v_actions != None:
            if self.end_time > self.sim.curr_step:
                if self.locations != None:
                    new_vehicles = []
                    for location in self.locations:
                        if self.sim.geometry_exists(location) != None:
                            new_vehicles += self.sim.get_last_step_geometry_vehicles(location, vehicle_types = self.vehicle_types, flatten = True)
                        elif self.sim.detector_exists(location) != None:
                            new_vehicles += self.sim.get_last_step_detector_vehicles(location, vehicle_types = self.vehicle_types, flatten = True)
                        else:
                            desc = "Object '{0}' has unknown type (must be detector or geometry ID).".format(location)
                            raise_error(KeyError, desc, self.sim.curr_step)
                            
                    new_vehicles = list(set(new_vehicles))
                    new_vehicles = [vehicle_id for vehicle_id in new_vehicles if vehicle_id not in self.affected_vehicles.keys()]
                
                elif self.vehicle_ids != None:

                    new_vehicles = list(set(self.vehicle_ids) - set(self.affected_vehicles.keys()))

                else:
                    desc = "Event '{0}' has no 'location' or list of vehicle IDs.".format(self.id)
                    raise_error(KeyError, desc, self.sim.curr_step)

                for vehicle_id in new_vehicles:
                    if self.total_affected_vehicles < self.vehicle_limit and vehicle_id not in self.ignored_vehs:

                        # Vehicle will be permanently ignored according to the effect probability
                        if random() > self.v_prob:
                            self.ignored_vehs.append(vehicle_id)
                            continue

                        base_vals = self.sim.get_vehicle_vals(vehicle_id, list(self.v_actions.keys()))
                        if len(list(self.v_actions.keys())) == 1:
                            action_key = list(self.v_actions.keys())[0]
                            self.v_base[vehicle_id] = {}
                            self.v_base[vehicle_id][action_key] = base_vals
                        else:
                            self.v_base[vehicle_id] = base_vals

                        if "speed" in self.v_base[vehicle_id].keys(): self.v_base[vehicle_id]["speed"] = -1

                        # If using relative effects (r_effects == True), the values in self.v_actions are
                        # used as the relative change to each variable (ie. 0.8 will result in 20% reduction)
                        if self.v_r_effects: action_dict = {a_key: self.v_base[vehicle_id][a_key] * self.v_actions[a_key] if isinstance(self.v_actions[a_key], (int, float)) else self.v_actions[a_key] for a_key in self.v_actions.keys()}
                        else: action_dict = self.v_actions

                        self.sim.set_vehicle_vals(vehicle_id, **action_dict)
                        # If self.v_effect_dur is a number, this is used as effect duration, else if it is "EVENT",
                        # all vehicle effects will be stopped once the event is over.
                        if isinstance(self.v_effect_dur, str) and self.v_effect_dur.upper() == "EVENT":
                            self.affected_vehicles[vehicle_id] = {"start_effect": self.sim.curr_step, "end_effect": self.end_time}
                        else:
                            self.affected_vehicles[vehicle_id] = {"start_effect": self.sim.curr_step, "end_effect": self.sim.curr_step + self.v_effect_dur}

                        if self.highlight != None:
                            self.sim.set_vehicle_vals(vehicle_id, colour=self.highlight)

                        if not self.assert_speed_safety: self.sim.set_vehicle_vals(vehicle_id, speed_safety_checks=False)
                        if not self.assert_lc_safety: self.sim.set_vehicle_vals(vehicle_id, lc_safety_checks=False)

                        self.total_affected_vehicles += 1

            affected_vehicles_ids = list(self.affected_vehicles.keys())
            for vehicle_id in affected_vehicles_ids:
                remove_effects = False
                if not self.sim.vehicle_exists(vehicle_id):
                    del self.affected_vehicles[vehicle_id]
                    continue
                
                elif self.affected_vehicles[vehicle_id]["end_effect"] <= self.sim.curr_step: remove_effects = True

                elif self.location_only:
                    l = self.sim.get_vehicle_vals(vehicle_id, ["edge_id", "lane_id"])
                    if l["edge_id"] not in self.locations and l["lane_id"] not in self.locations: remove_effects = True

                if self.force_end and self.sim.curr_step > self.end_time: remove_effects = True

                if remove_effects: self._remove_v_effects(vehicle_id)

    def terminate(self):
        """ Terminate active event before scheduled end, removing any vehicle/edge effects. """

        if self.e_effects_active or self.v_effects_active:
            self.end_time = self.sim.curr_step

            for vehicle_id in list(self.affected_vehicles.keys()):
                self._remove_v_effects(vehicle_id)

            if self.e_actions != None:
                for edge_id in self.edge_ids:
                    self.sim.set_geometry_vals(edge_id, **self.e_base[edge_id])
                self.e_effects_active = False

            self.e_effects_active = False
            self.v_effects_active = False

    def is_active(self):
        """
        Returns whether the event is currently active.

        Returns:
            bool: Denotes whether the event is active
        """
        return self.e_effects_active or self.v_effects_active
    
    def _remove_v_effects(self, vehicle_id):
                        
        if "acceleration" in self.v_base[vehicle_id].keys():
            del self.v_base[vehicle_id]["acceleration"]
        if "lane_idx" in self.v_base[vehicle_id].keys():
            del self.v_base[vehicle_id]["lane_idx"]
        if self.highlight != None:
            self.v_base[vehicle_id]["highlight"] = None

        self.sim.set_vehicle_vals(vehicle_id, **self.v_base[vehicle_id])
        if not self.assert_speed_safety: self.sim.set_vehicle_vals(vehicle_id, speed_safety_checks=True)
        if not self.assert_lc_safety: self.sim.set_vehicle_vals(vehicle_id, lc_safety_checks=True)

        del self.affected_vehicles[vehicle_id]

        if len(self.affected_vehicles.keys()) == 0 and self.end_time <= self.sim.curr_step:
            self.v_effects_active = False

        if self.remove_affected_vehicles:
            self.sim.remove_vehicles(vehicle_id)


def _cause_incident(self,
                    duration: int,
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
    
    if self._scheduler == None:
        self._scheduler = EventScheduler(self)
        
    if incident_id == None:
        id_idx = 1
        while self._scheduler.get_event_status("incident_{0}".format(id_idx)) != None: id_idx += 1
        incident_id = "incident_{0}".format(id_idx)

    event_dict = {"start_time": (self.curr_step + 1) * self.step_length, "end_time": (self.curr_step * self.step_length) + duration}
    
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
        vehicle_ids = choices(all_geometry_vehicles, k=min(n_vehicles, len(all_geometry_vehicles)))

    # Neither location or vehicles specified - an error is thrown
    elif geometry_ids != None and vehicle_ids != None:
        desc = "Invalid inputs (cannot use both vehicle_ids and geometry_ids)."
        raise_error(ValueError, desc, self.curr_step)
        
    if check_n_vehicles:
        if len(vehicle_ids) != n_vehicles:
            if assert_n_vehicles:
                desc = f"Incident could not be started (could not find enough vehicles, {len(vehicle_ids)} != {n_vehicles})."
                raise_error(SimulationError, desc, self.curr_step)
            else:
                if not self._suppress_warnings: raise_warning(f"Incident could not be started (could not find enough vehicles, {len(vehicle_ids)} != {n_vehicles}).")
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
                self.stop_vehicle(vehicle_id, duration=duration, pos=position)

        if "vehicles" in event_dict: event_dict["vehicles"]["vehicle_ids"] = vehicle_ids
    
        if edge_speed != None:
            if edge_speed < 0: edge_speed = 15 if self.units.name == "METRIC" else 10
            if geometry_ids == None: geometry_ids = [self.get_vehicle_vals(veh_id, "next_edge_id") for veh_id in vehicle_ids]
            event_dict["edges"] = {"edge_ids": geometry_ids, "actions": {"max_speed": edge_speed}}

    self.add_events({incident_id: event_dict})
    return True

def _add_weather(self,
                duration: int | float,
                strength: float = 0.2,
                locations: list | tuple | None = None,
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

    # internal lanes?

    if locations != None and isinstance(locations, (list, tuple)):
        for edge in locations:
            if self.geometry_exists(edge) == None:
                desc = f"Unrecognised geometry ID '{edge}'."
                raise_error(KeyError, desc, self.curr_step)
    else: locations = self._all_edges

    if weather_id == None: weather_id = f"weather_{len(self._weather_events) + 1}"
    if weather_id in self._weather_events:
        desc = f"Invalid weather event ID '{weather_id}' (already exists)."
        raise_error(KeyError, desc, self.curr_step)

    w_effects = {}
    w_effects["headway"] = 1 + strength if headway_increase == None else 1 + headway_increase
    w_effects["imperfection"] = 1 + strength if imperfection_increase == None else 1 + imperfection_increase
    w_effects["max_acceleration"] = 1 - strength if acceleration_reduction == None else 1 - acceleration_reduction
    w_effects["max_deceleration"] = 1 - strength if acceleration_reduction == None else 1 - acceleration_reduction
    w_effects["speed_factor"] = 1 - strength if speed_f_reduction == None else 1 - speed_f_reduction

    w_effects = {key: val for key, val in w_effects.items() if val != 1}

    if len(w_effects) == 0:
        desc = "Could not add weather effects (no changes set - increase strength)."
        raise_error(ValueError, desc, self.curr_step)

    event_dict = {weather_id: {
                    "start_step": self.curr_step + 1,
                    "end_step": self.curr_step + ((duration + 1) / self.step_length),
                    "vehicles": {
                        "locations": locations,
                        "actions": w_effects,
                        "effect_probability": 1,
                        "remove_affected_vehicles": False,
                        "r_effects": True,
                        "location_only": True,
                        "force_end": True
                        }
                    }}
    
    self.add_events(event_dict)
    self._weather_events.add(weather_id)
    return weather_id