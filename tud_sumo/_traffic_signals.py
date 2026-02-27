import traci
from copy import deepcopy
from .utils import *

def _set_tl_colour(self, junction_id: str | int, colour_str: str) -> None:
    """
    Sets a junction to a colour for an indefinite amount of time. Can be used when tracking phases separately (ie. not within TUD-SUMO).
    
    Args:
        `junction_id` (str, int): Junction ID
        `colour_str` (str): Phase colour string (valid characters are ['_G_' | '_g_' | '_y_' | '_r_' | '-'])
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
    
    _set_phases(self, junc_phases, overwrite=False)
    
def _set_phases(self, junction_phases: dict, start_phase: int = 0, overwrite: bool = True) -> None:
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

    _update_lights(self, list(junction_phases.keys()))

def _set_m_phases(self, junction_phases: dict, start_phase: int = 0, overwrite: bool = True) -> None:
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
                    desc = "Invalid phase colour(s) in junction '{0}', movement '{1}' (['{2}'] are invalid, must be in ['{3}']).".format(junction_id, m_key, "','".join(invalid_colours), "' | '".join(list(valid_colours)))
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

    _set_phases(self, new_phase_dict, start_phase=start_phase, overwrite=overwrite)

def _get_phase_string(curr_phases, masks):

    m_len = len(list(masks.values())[0])
    phase_arr = ['-'] * m_len
    
    for m_key in masks.keys():
        for idx in range(m_len):
            if masks[m_key][idx] == "1":
                phase_arr[idx] = curr_phases[m_key]

    phase_str = "".join(phase_arr)
    return phase_str

def _set_tl_metering_rate(self,
                            rm_id: str,
                            metering_rate: int | float,
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
    _set_phases(self, {rm_id: phases_dict}, overwrite=False)
    return phases_dict

def _change_phase(self, junction_id: str | int, phase_number: int) -> None:
    """
    Change to a different phase at the specified junction_id.
    
    Args:
        `junction_id` (str, int): Junction ID
        `phase_number` (int): Phase number
    """
    
    if 0 < phase_number < len(self._junc_phases[junction_id]["phases"]):
        self._junc_phases[junction_id]["curr_phase"] = phase_number
        self._junc_phases[junction_id]["curr_time"] = sum(self.junc_phase["times"][:phase_number])

        _update_lights(self, junction_id)

    else:
        desc = "Invalid phase number '{0}' (must be [0-{1}]).".format(phase_number, len(self._junc_phases[junction_id]["phases"]))
        raise_error(ValueError, desc, self.curr_step)

def _update_lights(self, junction_ids: list | str | None = None) -> None:
    """
    Update light settings for given junctions.
    
    Args:
        `junction_ids` (list, str, optional): Junction ID, or list of IDs (defaults to all)
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