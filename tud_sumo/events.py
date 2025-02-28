import json, math, os, pickle as pkl
from copy import deepcopy
from random import random
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

    def get_event_ids(self, event_statuses: str|list|tuple|None=None) -> list:
        """
        Gets all event IDs, or those of a specific status.

        Args:
            `event_statuses` (str, list, tuple, optional): Event status or list of statuses from ['_scheduled_'|'_active_'|'_completed_'] (defaults to all)

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
    
    def get_events(self, event_statuses: str|list|tuple|None=None) -> list:
        """
        Gets all events, or those of a specific status.

        Args:
            `event_statuses` (str, list, tuple, optional): Event status or list of statuses from ['_scheduled_'|'_active_'|'_completed_'] (defaults to all)

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
                        desc = "Invalid event_params dict (must contain [dict|Event], not '{0}').".format(type(event_params).__name__)
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
                desc = "Invalid event_params (must be [dict|filepath|(Event)], not '{0}').".format(type(events).__name__)
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

    def get_event_status(self, event_id: str) -> str|None:
        """
        Get the status of an event, by its ID.
        
        Args:
            `event_id` (str): Event ID
        
        Returns:
            (str, optional): Event status ['_scheduled_'|'_active_'|'_completed_'], or `None` if it does not exist
        """

        if event_id in self.scheduled_events.keys(): return "scheduled"
        elif event_id in self.active_events.keys(): return "active"
        elif event_id in self.completed_events.keys(): return "completed"
        else: return None

class Event:
    """ A scheduled event, where effects are carried out for a specified amount of time. """

    def __init__(self, event_id: str, event_params: str|dict, simulation):
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
            desc = "Invalid event_params (must be [dict|filepath (str)], not '{0}').".format(type(event_params).__name__)
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

        if "edges" not in event_params.keys() and "vehicles" not in event_params.keys():
            desc = "Neither 'edges' or 'vehicles' parameters are given and one or both is required."
            raise_error(KeyError, desc, self.sim.curr_step)
        
        if "edges" in event_params.keys():
            edge_params = event_params["edges"]
            valid_params = {"actions": dict, "edge_ids": (list, tuple)}
            error, desc = test_input_dict(edge_params, valid_params, dict_name="edge", required=True)
            if error != None: raise_error(error, desc, self.sim.curr_step)
                                          
            self.edge_ids = edge_params["edge_ids"]
            self.e_actions, self.e_base = edge_params["actions"], {}

        else: self.e_actions = None

        if "vehicles" in event_params.keys():
            veh_params = event_params["vehicles"]
            valid_params = {"actions": dict, "locations": (list, tuple), "vehicle_ids": (list, tuple), "effect_duration": (int, float),
                            "vehicle_types": (list, tuple), "effect_probability": (int, float), "vehicle_limit": int, "highlight": bool,
                            "remove_affected_vehicles": bool, "speed_safety_checks": bool, "lc_safety_checks": bool}
            
            error, desc = test_input_dict(veh_params, valid_params, "vehicle")
            if error != None: raise_error(error, desc, self.sim.curr_step)

            if "actions" not in veh_params.keys():
                desc = "Vehicle events require 'actions' parameters."
                raise_error(KeyError, desc, self.sim.curr_step)

            self.locations, self.vehicle_ids = None, None
            if "locations" in veh_params:
                self.locations = veh_params["locations"]
            elif "vehicle_ids" in veh_params:
                self.vehicle_ids = veh_params["vehicle_ids"]
            else:
                desc = "Event 'locations' or 'vehicle_ids' are not given and one is required."
                raise_error(KeyError, desc, self.sim.curr_step)

            self.v_effect_dur = math.inf if "effect_duration" not in veh_params.keys() else veh_params["effect_duration"]
            self.v_actions, self.v_base, self.affected_vehicles = veh_params["actions"], {}, {}

            if "vehicle_types" in veh_params.keys():
                self.vehicle_types = veh_params["vehicle_types"]
            else: self.vehicle_types = None

            self.v_prob = 1 if "effect_probability" not in veh_params.keys() else veh_params["effect_probability"]
            
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

                self.sim.set_geometry_vals(edge_id, **self.e_actions)
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
                    if self.total_affected_vehicles < self.vehicle_limit and random() < self.v_prob:
                        base_vals = self.sim.get_vehicle_vals(vehicle_id, list(self.v_actions.keys()))
                        if len(list(self.v_actions.keys())) == 1:
                            action_key = list(self.v_actions.keys())[0]
                            self.v_base[vehicle_id] = {}
                            self.v_base[vehicle_id][action_key] = base_vals
                        else:
                            self.v_base[vehicle_id] = base_vals

                        if "speed" in self.v_base[vehicle_id].keys(): self.v_base[vehicle_id]["speed"] = -1
                        self.sim.set_vehicle_vals(vehicle_id, **self.v_actions)

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
                if not self.sim.vehicle_exists(vehicle_id):
                    del self.affected_vehicles[vehicle_id]
                    continue
                
                elif self.affected_vehicles[vehicle_id]["end_effect"] <= self.sim.curr_step:
                    
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

    def is_active(self):
        """
        Returns whether the event is currently active.

        Returns:
            bool: Denotes whether the event is active
        """
        return self.e_effects_active or self.v_effects_active