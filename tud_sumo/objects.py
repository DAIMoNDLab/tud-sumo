import traci
from shapely.geometry import LineString, Point
from .utils import *

class TrackedJunction:
    """ Junction object with automatic data collection. """

    def __init__(self, junc_id: str, simulation, junc_params: dict | str=None) -> None:
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

                    self._queue_lengths.append(len([veh_id for veh_id in queuing_vehicles if self.sim.get_vehicle_vals(veh_id, "is_stopped")]))

                    queue_delay = 0
                    if len(queuing_vehicles) > 0:
                        for ramp_edge in self.ramp_edges:
                            queue_delay += self.sim.get_geometry_vals(ramp_edge, "vehicle_delay")
                    self._queue_delays.append(queue_delay)

                elif self.queue_detector != None:
                    queuing_vehicles = self.sim.get_last_step_detector_vehicles(self.queue_detector, flatten=True)
                    queuing_vehicles = [veh_id for veh_id in queuing_vehicles if self.sim.vehicle_exists(veh_id)]
                    num_stopped = len([veh_id for veh_id in queuing_vehicles if self.sim.get_vehicle_vals(veh_id, "is_stopped")])
                    self._queue_lengths.append(num_stopped)

                else:
                    desc = f"Cannot update meter '{self.id}' queue length (no detector or entry/exit edges)"
                    raise_error(KeyError, desc, self.sim.curr_step)

    def set_metering_rate(self,
                          metering_rate: int | float,
                          *,
                          g_time: int | float = 1,
                          y_time: int | float = 1,
                          min_red: int | float = 1,
                          vehs_per_cycle: int | None = None,
                          control_interval: int | float = 60):
        """
        Set ramp metering rate of a meter at a junction. Uses a one-car-per-green policy with a default
        1s green and yellow time, with red phase duration changed to set flow. All phase durations must
        be larger than the simulation step length.
        
        Args:
            `metering_rate` (int, float): On-ramp inflow in veh/hr (from all lanes)
            `g_time` (int, float): Green phase duration (s), defaults to 1
            `y_time` (int, float): Yellow phase duration (s), defaults to 1
            `min_red` (int, float): Minimum red phase duration (s), defaults to 1
            `vehs_per_cycle` (int, optional): Number of vehicles released with each cycle, defaults to the number of lanes
            `control_interval` (int, float): Ramp meter control interval (s)
        """

        if not self._is_meter:
            desc = f"Cannot set metering rate (Junction ID '{self.id}' is not a metered junction)."
            raise_error(SimulationError, desc, self.sim.curr_step)
        
        self.sim.set_tl_metering_rate(self.id, metering_rate, g_time=g_time, y_time=y_time, min_red=min_red, vehs_per_cycle=vehs_per_cycle, control_interval=control_interval)

class TrackedEdge:
    """ Edge object with automatic data collection. """

    def __init__(self, edge_id: str, simulation) -> None:
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
                pos = _get_distance_on_road(vals["position"], self.linestring)
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

def _get_distance_on_road(veh_coors, linestring):
    line = LineString(linestring)
    p = Point(veh_coors)
    p2 = line.interpolate(line.project(p))
    x_val = line.line_locate_point(p2, False)
    x_pct = x_val/line.length
    return x_pct

def _add_tracked_junctions(self, junctions: str | list | tuple | dict | None = None) -> dict:
    """
    Initalise junctions and start tracking states and flows. Defaults to all junctions with traffic lights.
    
    Args:
        `junctions` (str, list, tuple, dict, optional): Junction IDs or list of IDs, or dict containing junction(s) parameters
    
    Returns:
        dict | TrackedJunction: Dictionary of added junctions or single TrackedJunction  object
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

def _add_tracked_edges(self, edge_ids: str | list | None = None):
    """
    Initalise edges and start collecting data.
    
    Args:
        `edge_ids` (str, list, optional): List of edge IDs or single ID, defaults to all
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