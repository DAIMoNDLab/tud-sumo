import json, math, os.path, numpy as np, pickle as pkl
from copy import deepcopy
from random import random, seed, choice
from tqdm import tqdm

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.colors import is_color_like as is_mpl_colour
from mpl_toolkits.axes_grid1 import make_axes_locatable

from .simulation import Simulation
from .demand import DemandProfile
from .utils import *

class _GenericPlotter():

    def __init__(self, units: str="METRIC", time_unit: str="seconds", save_fig_loc: str="", save_fig_dpi: int=600, overwrite_figs: bool=True):

        self._default_labels = {"no_vehicles": "No. of Vehicles", "no_waiting": "No. of Waiting Vehicles", "tts": "Total Time Spent (s)", "twt": "Total Waiting Time (s)", "avg_wt": "Waiting Time (s)",
                                "delay": "Delay (veh s)", "avg_delay": "Delay (s)", "throughput": "Throughput (veh/hr)", "vehicle_counts": "No. of Vehicles", "flows": "Flow (vehicles/hour)",
                                "occupancies": "Occupancy (%)", "densities": "Density unit", "metres": "Distance (m)", "kilometres": "Distance (km)", "yards": "Distance (yd)", "feet": "Distance (ft)",
                                "miles": "Distance (mi)", "m/s": "Speed (m/s)", "kmph": "Speed (kmph)", "mph": "Speed (mph)", "steps": "Time (Simulation Steps)", "seconds": "Time (s)", "minutes": "Time (m)",
                                "hours": "Time (hr)", "to_depart": "No. of Vehicles"}

        self._default_titles = {"no_vehicles": "Number of Vehicles", "no_waiting": "Number of Waiting Vehicles", "tts": "Total Time Spent",
                                "twt": "Total Waiting Time", "avg_wt": "Average Vehicle Waiting Time", "delay": "Vehicle Delay", "avg_delay": "Average Vehicle Delay",
                                "vehicle_counts": "Number of Vehicles", "occupancies": "Vehicle Occupancies", "densities": "Vehicle Density",
                                "speeds": "Average Vehicle Speed", "flows": "Vehicle Flow", "limits": "Speed Limit", "throughput": "Throughput", "to_depart": "Vehicles to Depart"}

        # TU Delft colours as defined here: https://www.tudelft.nl/huisstijl/bouwstenen/kleur
        self._tud_colours = {"cyaan": "#00A6D6", "donkerblauw": "#0C2340", "turkoois": "#00B8C8", "blauw": "#0076C2", "paars": "#6F1D77", "roze": "#EF60A3",
                             "framboos": "#A50034", "rood": "#E03C31", "oranje": "#EC6842", "geel": "#FFB81C", "lichtgroen": "#6CC24A", "donkergroen": "#009B77"}

        units = units.upper()
        error, desc = test_valid_string(units, ["METRIC", "IMPERIAL", "UK"], "simulation units")
        if error != None: raise_error(error, desc)

        self.units, avg_speed, speed, limit = units, "Avg. Speed ", "Vehicle Speed ", "Speed Limit "
        if units in ["IMPERIAL", "UK"]:
            avg_speed += "(mph)"
            speed += "(mph)"
            limit += "(mph)"
        elif units in ["METRIC"]:
            avg_speed += "(km/h)"
            speed += "(km/h)"
            limit += "(km/h)"

        error, desc = test_valid_string(time_unit, ["steps", "seconds", "minutes", "hours"], "time_unit")
        if error != None: raise_error(error, desc)
        self.time_unit = time_unit.lower()
        
        self._default_labels["sim_time"] = "Simulation Time ({0})".format(self.time_unit)
        self._default_labels["speeds"] = avg_speed
        self._default_labels["speed"] = speed
        self._default_labels["limits"] = limit

        self.save_fig_loc, self.overwrite_figs = save_fig_loc, overwrite_figs
        if self.save_fig_loc != "":
            if not self.save_fig_loc.endswith('/'): self.save_fig_loc += "/"
            if not os.path.exists(self.save_fig_loc):
                desc = "File path '{0}' does not exist.".format(self.save_fig_loc)
                raise_error(FileNotFoundError, desc)
            
        self.save_fig_dpi = save_fig_dpi

        self.CYAAN = self._tud_colours["cyaan"]
        self.DONKERBLAUW = self._tud_colours["donkerblauw"]
        self.TURKOOIS = self._tud_colours["turkoois"]
        self.BLAUW = self._tud_colours["blauw"]
        self.PAARS = self._tud_colours["paars"]
        self.ROZE = self._tud_colours["roze"]
        self.FRAMBOOS = self._tud_colours["framboos"]
        self.ROOD = self._tud_colours["rood"]
        self.ORANJE = self._tud_colours["oranje"]
        self.GEEL = self._tud_colours["geel"]
        self.LICHTGROEN = self._tud_colours["lichtgroen"]
        self.DONKERGROEN = self._tud_colours["donkergroen"]

        self.line_colours = [self.CYAAN, self.ORANJE, self.LICHTGROEN, self.FRAMBOOS, self.PAARS,
                             self.BLAUW,  self.GEEL, self.DONKERGROEN, self.ROOD, self.ROZE]
        
        self.line_styles = ['solid', 'solid', 'solid', 'solid', 'solid',
                            'dashed', 'dashed', 'dashed', 'dashed', 'dashed']
        
        self._default_colour_idx = 0
        self._default_colour = self.line_colours[self._default_colour_idx]
        self._next_colour_idx = 0

    def _display_figure(self, filename: str|None=None) -> None:
        """
        Display figure, either saving to file or showing on screen.
        
        Args:
            `filename` (str, optional): Save file name, if saving
        """

        if filename is None: plt.show()
        else:
            
            if not filename.endswith(".png") and not filename.endswith('.jpg'):
                filename += ".png"

            fp = self.save_fig_loc + filename
            if os.path.exists(fp) and not self.overwrite_figs:
                desc = "File '{0}' already exists.".format(fp)
                raise_error(FileExistsError, desc)
            
            plt.savefig(fp, dpi=self.save_fig_dpi)

        plt.close()

    def _add_grid(self, ax, zorder=0):
        ax.grid(True, 'both', color='grey', linestyle='dashed', linewidth=0.5, zorder=zorder)
        
    def _get_colour(self, colour: str|int|None=None, reset_wheel: bool=False) -> str:

        if reset_wheel: self._next_colour_idx = 0

        if colour == None:
            colour = self.line_colours[0]
        
        elif isinstance(colour, int):
            if self._next_colour_idx >= 0 and self._next_colour_idx < len(self.line_colours):
                colour = self.line_colours[colour]
            else:
                desc = "Colour wheel index '{0}' out of range.".format(self._next_colour_idx)
                raise_error(IndexError, desc)

        elif isinstance(colour, str):
            if colour in self._tud_colours:
                colour = self._tud_colours[colour]
            elif colour.upper() == "DEFAULT":
                colour = self._tud_colours[self._default_colour]
            elif colour.upper() == "RANDOM":
                colour = choice(self.line_colours)
            elif colour.upper() == "WHEEL":
                if self._next_colour_idx >= 0 and self._next_colour_idx < len(self.line_colours):
                    colour = self.line_colours[self._next_colour_idx]
                    line_style = self.line_styles[self._next_colour_idx]
                    
                    if self._next_colour_idx == len(self.line_colours) - 1: self._next_colour_idx = 0
                    else: self._next_colour_idx += 1
                    
                    return colour, line_style
            
                else:
                    desc = "Colour wheel index '{0}' out of range.".format(self._next_colour_idx)
                    raise_error(IndexError, desc)
            elif not is_mpl_colour(colour):
                desc = "Unrecognised colour '{0}'.".format(colour)
                raise_error(ValueError, desc)

        else:
            desc = "Invalid plt_colour '{0}' (must be 'str', not '{1}').".format(colour, type(colour).__name__)
            raise_error(TypeError, desc)
        
        return colour

    def _plot_event(self, ax, event_ids: str|list|None=None) -> None:
        """
        Plot events from the simulation data on a given axes.
        
        Args:
            `ax` (matplotlib.axes): matplotlib axes
            `event_ids` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
        """

        if event_ids != None:
            if "events" in self.sim_data["data"].keys():
                
                statuses, all_statuses = {}, ["scheduled", "active", "completed"]

                if isinstance(event_ids, str) and event_ids not in all_statuses + ["all"]: event_ids = [event_ids]

                if isinstance(event_ids, str):
                    if event_ids == "all":
                        event_ids = []
                        for status in all_statuses:
                            if status in self.sim_data["data"]["events"]:
                                s_events = list(self.sim_data["data"]["events"][status].keys())
                                event_ids += s_events
                                statuses.update({e_id: status for e_id in s_events})

                    elif event_ids in all_statuses:
                        if event_ids in self.sim_data["data"]["events"]:
                            statuses = {e_id: event_ids for e_id in self.sim_data["data"]["events"][event_ids].keys()}
                            event_ids = list(statuses.keys())
                        else: event_ids = []
                        
                else:
                    if not isinstance(event_ids, (list, tuple)): event_ids = [event_ids]
                    validate_list_types(event_ids, str, param_name="event_ids")
                    for e_id in event_ids:
                        
                        for status in all_statuses:
                            if status in self.sim_data["data"]["events"]:
                                if e_id in self.sim_data["data"]["events"][status]:
                                    statuses[e_id] = status
                        
                        if e_id not in statuses:
                            desc = "Event with ID '{0}' not found.".format(e_id)
                            raise_error(KeyError, desc)

                if len(event_ids) > 0:

                    _, y_lim = ax.get_xlim(), ax.get_ylim()
                    for event_id in event_ids:

                        event = self.sim_data["data"]["events"][statuses[event_id]][event_id]
                        event_start, event_end = convert_units([event["start_time"], event["end_time"]], "steps", self.time_unit, self.sim_data["step_len"])
                        ax.axvspan(event_start, event_end, color="red", zorder=1, alpha=0.2)

                        ax.axvline(event_start, color="red", alpha=0.4, linestyle='--')
                        ax.axvline(event_end, color="red", alpha=0.4, linestyle='--')

                        ax.text(event_start + ((event_end - event_start)/2), y_lim[1] * 0.9, event_id, horizontalalignment='center', color="red", zorder=10)

class Plotter(_GenericPlotter):
    """ Visualisation class that plots TUD-SUMO data for one simulation. """

    def __init__(self, simulation: Simulation|str, sim_label: str|None=None, time_unit: str="seconds", save_fig_loc: str="", save_fig_dpi: int=600, overwrite_figs: bool=True) -> None:
        """
        Args:
            `simulation` (Simulation, str): Either simulation object, sim_data dict or sim_data filepath
            `sim_label` (str, optional): Simulation or scenario label added to the beginning of all plot titles (set to 'scenario' for scenario name)
            `time_unit` (str): Plotting time unit used for all plots (must be ['_steps_'|'_seconds_'|'_minutes_'|'_hours_'])
            `save_fig_loc` (str): Figure filepath when saving (defaults to current file)
            `save_fig_dpi` (int): Figure dpi when saving (defaults to 600dpi)
            `overwrite_figs` (bool): Denotes whether to allow overwriting of saved figures with the same name
        """

        self.simulation = None
        if isinstance(simulation, Simulation):
            self.simulation = simulation
            self.sim_data = simulation.__dict__()
            units = simulation.units.name
            scenario_name = simulation.scenario_name

        elif isinstance(simulation, str):

            if simulation.endswith(".json"): r_class, r_mode = json, "r"
            elif simulation.endswith(".pkl"): r_class, r_mode = pkl, "rb"
            else:
                desc = "Invalid simulation file '{0}' (must be '.json' or '.pkl' file).".format(simulation)
                raise_error(ValueError, desc)

            if os.path.exists(simulation):
                with open(simulation, r_mode) as fp:
                    self.sim_data = r_class.load(fp)
                    units = self.sim_data["units"]
                    scenario_name = self.sim_data["scenario_name"]
            else:
                desc = "Simulation file '{0}' not found.".format(simulation)
                raise_error(FileNotFoundError, desc)

        elif isinstance(simulation, dict): self.sim_data, units, scenario_name = simulation, simulation["units"], simulation["scenario_name"]

        else:
            desc = "Invalid simulation type (must be Simulation|str|dict, not '{0}').".format(type(simulation).__name__)
            raise_error(TypeError, desc)

        if isinstance(sim_label, str) and sim_label.upper() == "SCENARIO": self.sim_label = scenario_name + ": "
        elif sim_label != None: self.sim_label = sim_label + ": "
        else: self.sim_label = ""

        super().__init__(units, time_unit, save_fig_loc, save_fig_dpi, overwrite_figs)

    def __str__(self): return "<{0}>".format(self.__name__)
    def __name__(self): return "Plotter"
        
    def plot_junc_flows(self, junc_id: str, vehicle_types: list|tuple|None=None, plot_all: bool=True, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot junction flow, either as inflow & outflow or number of vehicles at the intersection.
        
        Args:
            `junc_id` (str): Junction ID
            `vehicle_types` (list, tuple, optional): Vehicle type ID or list of IDs
            `plot_all` (bool): If `True`, plot total values as well as vehicle type data
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:

            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

            if junc_id in self.simulation.tracked_junctions.keys(): tl = self.simulation.tracked_junctions[junc_id]
            else:
                desc = "Junction '{0}' not found in tracked junctions.".format(junc_id)
                raise_error(KeyError, desc)

            if tl._track_flow: junc_flows = tl.__dict__()["flows"]
            else:
                desc = "No traffic light at junction '{0}'.".format(junc_id)
                raise_error(ValueError, desc)

            step = self.simulation.step_length
            start = self.simulation.tracked_junctions[junc_id].init_time

        elif "junctions" in self.sim_data["data"].keys() and junc_id in self.sim_data["data"]["junctions"].keys():
            if "flows" in self.sim_data["data"]["junctions"][junc_id].keys():
                junc_flows = self.sim_data["data"]["junctions"][junc_id]["flows"]
                step = self.sim_data["step_len"]
                start = self.sim_data["data"]["junctions"][junc_id]["init_time"]

            else:
                desc = "Junction '{0}' does not track flows (no detectors).".format(junc_id)
                raise_error(ValueError, desc)
        else:
            desc = "Junction '{0}' not found in tracked junctions.".format(junc_id)
            raise_error(KeyError, desc)

        if vehicle_types == None: vehicle_types = list(junc_flows["all_inflows"].keys())
        elif not isinstance(vehicle_types, (list, tuple)): vehicle_types = [vehicle_types]

        if "all" in vehicle_types and not plot_all: vehicle_types.remove("all")

        fig, ax = plt.subplots(1, 1)

        for vehicle_type_idx, vehicle_type in enumerate(vehicle_types):
            inflow_data, outflow_data = junc_flows["all_inflows"][vehicle_type], junc_flows["all_outflows"][vehicle_type]
            cumulative_inflow, cumulative_outflow = get_cumulative_arr(inflow_data), get_cumulative_arr(outflow_data)
            
            time_steps = get_time_steps(cumulative_inflow, self.time_unit, step, start)
            _, cumulative_inflow = limit_vals_by_range(time_steps, cumulative_inflow, time_range)
            time_steps, cumulative_outflow = limit_vals_by_range(time_steps, cumulative_outflow, time_range)

            linewidth = 1.5 if vehicle_type == "all" else 1
            colour, _ = self._get_colour("WHEEL", vehicle_type_idx==0)
            inflow_line = plt.plot(time_steps, cumulative_inflow, color=colour, label=vehicle_type+' in', linewidth=linewidth)
            ax.plot(time_steps, cumulative_outflow, label=vehicle_type + ' out', linestyle='--', linewidth=linewidth, color=inflow_line[-1].get_color())

        fig_title = self.sim_label+"Vehicle Flows at Intersection '{0}'".format(junc_id) if fig_title == None else fig_title
        ax.set_title(fig_title, pad=20)
        ax.set_ylabel(self._default_labels["vehicle_counts"])
        ax.set_xlim([time_steps[0], time_steps[-1] + convert_units(step, "seconds", self.time_unit, step)])
        ax.set_ylim(bottom=0)
        ax.set_xlabel(self._default_labels["sim_time"])
        fig.tight_layout()
        ax.legend(title="Vehicle Types", fontsize="small", shadow=True)
        self._add_grid(ax, None)

        self._plot_event(ax, show_events)
        
        fig.tight_layout()

        self._display_figure(save_fig)
    
    def plot_tl_colours(self, tl_id: str, plt_movements: list|tuple|None=None, plot_percent: bool=False, time_range: list|tuple|None=None, save_fig: str|None=None) -> None:
        """
        Plot traffic light sequence, as colours or green/red/yellow durations as a percent of time.
        
        Args:
            `tl_id` (str): Traffic light ID
            `plt_movements` (list, tuple, optional): List of movements to plot by index (defaults to all)
            `plot_percent` (bool):  Denotes whether to plot colours as percent of time
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        tl_colours = {"G": self.DONKERGROEN, "Y": self.GEEL, "R": self.ROOD}

        if self.simulation != None:

            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

            if tl_id in self.simulation.tracked_junctions.keys(): tl = self.simulation.tracked_junctions[tl_id]
            else:
                desc = "Junction '{0}' not found in tracked junctions.".format(tl_id)
                raise_error(KeyError, desc)

            if tl._has_tl: tl_durs = deepcopy(tl._durations)
            else:
                desc = "No traffic light at junction '{0}'.".format(tl_id)
                raise_error(ValueError, desc)

            m_len = tl.m_len
            init_time = tl.init_time
            end_time = tl.curr_time

        elif "junctions" in self.sim_data["data"].keys() and tl_id in self.sim_data["data"]["junctions"].keys():
            if "tl" in self.sim_data["data"]["junctions"][tl_id].keys():
                tl_durs = deepcopy(self.sim_data["data"]["junctions"][tl_id]["tl"]["m_phases"])
                m_len, init_time = self.sim_data["data"]["junctions"][tl_id]["tl"]["m_len"], self.sim_data["data"]["junctions"][tl_id]["init_time"]
                end_time = self.sim_data["data"]["junctions"][tl_id]["init_time"]

            else:
                desc = "No traffic light at junction '{0}'.".format(tl_id)
                raise_error(ValueError, desc)
        else:
            desc = "Junction '{0}' not found in tracked junctions.".format(tl_id)
            raise_error(KeyError, desc)

        if plt_movements != None:
            m_mask = plt_movements
            m_mask.sort()
            for idx in m_mask:
                if idx >= m_len or idx < 0:
                    desc = "Invalid movement index '{0}' (must be 0 <= idx <= {1})".format(idx, m_len - 1)
                    raise_error(ValueError, desc)
            for i in reversed(range(m_len)):
                if i not in m_mask: tl_durs.pop(i)

            m_len = len(m_mask)

        xlim = convert_units([self.sim_data["start"], self.sim_data["end"]], "steps", self.time_unit, self.sim_data["step_len"])
        if time_range != None and isinstance(time_range, (list, tuple)):
            if len(time_range) != 2:
                desc = "Invalid time range (must have length 2, not {0}).".format(len(time_range))
                raise_error(ValueError, desc)
            elif time_range[0] >= time_range[1]:
                desc = "Invalid time range (start_time ({0}) >= end_time ({1})).".format(time_range[0], time_range[1])
                raise_error(ValueError, desc)
            else:
                clipped_tl_durs = []
                step_range = convert_units(time_range, self.time_unit, "steps", self.sim_data["step_len"])
                start_step, end_step = step_range[0], step_range[1]
                xlim = time_range
                for m in tl_durs:

                    # phase times are in steps
                    new_m, curr_time = [], init_time
                    for (colour, phase_dur) in m:
                        if curr_time >= end_step:
                            break
                        elif curr_time < start_step and curr_time + phase_dur < start_step:
                            continue
                        elif curr_time >= start_step:
                            new_m.append([colour, min(phase_dur, end_step - curr_time)])
                        elif curr_time < start_step and curr_time + phase_dur > start_step:
                            new_m.append([colour, curr_time + phase_dur - start_step])

                        curr_time += phase_dur

                    clipped_tl_durs.append(new_m)
                
                tl_durs = clipped_tl_durs

        fig, ax = plt.subplots(1, 1)

        if plot_percent:
            percent_tl_durs = [[] for _ in range(m_len)]
            for idx, m_durs in enumerate(tl_durs):
                total_len = sum([x[1] for x in m_durs])
                percent_tl_durs[idx].append(['G', sum([x[1] for x in m_durs if x[0] == 'G']) / total_len * 100])
                percent_tl_durs[idx].append(['Y', sum([x[1] for x in m_durs if x[0] == 'Y']) / total_len * 100])
                percent_tl_durs[idx].append(['R', sum([x[1] for x in m_durs if x[0] == 'R']) / total_len * 100])

            tl_durs = percent_tl_durs
            ax.set_xlabel("Movements")
            ax.set_ylabel("Colour Duration (%)")
            ax.set_ylim((0, 100))
        else:
            ax.set_xlabel(self._default_labels["sim_time"])
            ax.set_ylabel("Movement")
            ax.set_xlim(xlim)

        if plt_movements == None: ms = list([str(i) for i in range(1, m_len + 1)])
        else: ms = list([str(i) for i in m_mask])

        curr_colour = 'G'
        all_plotted = False

        if time_range != None: offset_value = time_range[0]
        else: offset_value = convert_units(init_time, "steps", self.time_unit, self.sim_data["step_len"])
        offset = [offset_value for _ in range(m_len)]

        while not all_plotted:

            curr_bar = [0 for _ in range(m_len)]
            for idx, m_durs in enumerate(tl_durs):
                if len(m_durs) == 0: continue
                
                if m_durs[0][0] == curr_colour:
                    curr_bar[idx] = convert_units(m_durs[0][1], "steps", self.time_unit, self.sim_data["step_len"])
                    tl_durs[idx].pop(0)
            
            all_plotted = True
            for m_durs in tl_durs:
                if len(m_durs) != 0: all_plotted = False
            
            if plot_percent: ax.bar(ms, curr_bar, bottom=offset, color=tl_colours[curr_colour])
            else: ax.barh(ms, curr_bar, left=offset, color=tl_colours[curr_colour])

            for m in range(m_len): offset[m] = offset[m] + curr_bar[m]

            if curr_colour == 'G': curr_colour = 'Y'
            elif curr_colour == 'Y': curr_colour = 'R'
            elif curr_colour == 'R': curr_colour = 'G'

        ax.set_title(self.sim_label+"'{0}' Signal Phases".format(tl_id), pad=20)
        fig.tight_layout()
        self._display_figure(save_fig)

    def plot_rm_rate(self, rm_id: str, ax=None, yax_labels: bool=True, xax_labels: bool=True, show_legend: bool=True, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot ramp metering rate.
        
        Args:
            `rm_id` (str): Ramp meter junction ID
            `ax` (matplotlib.axes, optional): Matplotlib axis, used when creating subplots
            `yax_labels` (bool): Bool denoting whether to include y-axis labels (for subplots)
            `xax_labels` (bool): Bool denoting whether to include x-axis labels (for subplots)
            `show_legend` (bool): Bool denoting whether to show figure legend
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:

            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

            if rm_id in self.simulation.tracked_junctions.keys(): tl = self.simulation.tracked_junctions[rm_id]
            else:
                desc = "Junction '{0}' not found in tracked junctions.".format(rm_id)
                raise_error(KeyError, desc)

            if tl._is_meter:
                rates = tl._metering_rates
                times = tl._rate_times
                min_r, max_r = tl.min_rate, tl.max_rate
            else:
                desc = "Junction '{0}' is not tracked as a meter.".format(rm_id)
                raise_error(ValueError, desc)

        elif "junctions" in self.sim_data["data"].keys() and rm_id in self.sim_data["data"]["junctions"].keys():
            if "meter" in self.sim_data["data"]["junctions"][rm_id].keys():
                rates = self.sim_data["data"]["junctions"][rm_id]["meter"]["metering_rates"]
                times = self.sim_data["data"]["junctions"][rm_id]["meter"]["rate_times"]
                min_r, max_r = self.sim_data["data"]["junctions"][rm_id]["meter"]["min_rate"], self.sim_data["data"]["junctions"][rm_id]["meter"]["max_rate"]

            else:
                desc = "Junction '{0}' is not tracked as a meter.".format(rm_id)
                raise_error(ValueError, desc)
        else:
            desc = "Junction '{0}' not found in tracked junctions.".format(rm_id)
            raise_error(KeyError, desc)

        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        start = convert_units(start, "steps", self.time_unit, step)
        end = convert_units(end, "steps", self.time_unit, step)
        times = convert_units(times, "steps", self.time_unit, step)
        if time_range == None: time_range = [-math.inf, math.inf]

        is_subplot = ax != None
        if not is_subplot: fig, ax = plt.subplots(1, 1)
        
        colour = self.CYAAN
        label, width, zorder = "Metering Rate", 1.5, 3
        
        for idx, (time, rate) in enumerate(zip(times, rates)):
            if idx > 0:
                prev_line_start, prev_line_end = times[int(idx - 1)], time

                if prev_line_start >= time_range[1]:
                    break

                elif prev_line_start < time_range[0] and prev_line_end < time_range[0]:
                    prev_rate = rate
                    continue
                
                elif prev_line_start >= time_range[0]:
                    ax.plot([prev_line_start, min(prev_line_end, time_range[1])], [prev_rate, prev_rate], color=colour, label=label, linewidth=width, zorder=zorder)

                elif prev_line_start < time_range[0] and prev_line_end > time_range[0]:
                    ax.plot([time_range[0], prev_line_end], [prev_rate, prev_rate], color=colour, label=label, linewidth=width, zorder=zorder)

                label = None

                if prev_line_end < time_range[1]:
                    ax.plot([prev_line_end, prev_line_end], [prev_rate, rate], color=colour, linewidth=width, zorder=zorder)

            prev_rate = rate

        if prev_line_end < time_range[1]:
            ax.plot([prev_line_end, min(end, time_range[1])], [prev_rate, prev_rate], color=colour, label=label, linewidth=width, zorder=zorder)

        xlim = [start, end]
        if time_range != None:
            xlim[0], xlim[1] = max(xlim[0], time_range[0]), min(xlim[1], time_range[1])
        ax.set_xlim(xlim)
        ax.set_ylim([0, get_axis_lim(max_r)])
        ax.axhline(max_r, label="Min/Max Rate", color=self.ROOD, linestyle="--", zorder=1)
        ax.axhline(min_r, color=self.ROOD, linestyle="--", zorder=2)
        self._add_grid(ax, None)
        if yax_labels: ax.set_ylabel("Metering Rate (veh/hr)")
        if xax_labels: ax.set_xlabel(self._default_labels["sim_time"])
        fig_title = "{0}'{1}' Metering Rate".format(self.sim_label, rm_id) if not isinstance(fig_title, str) else fig_title
        if fig_title != "": ax.set_title(fig_title, pad=20)

        if show_legend:
            box = ax.get_position()
            if not is_subplot:
                ax.set_position([box.x0, box.y0 + box.height * 0.08,
                                box.width, box.height * 0.92])
                ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13),
                        fancybox=True, ncol=2, shadow=True)
            else:
                ax.set_position([box.x0, box.y0 + box.height * 0.02,
                                box.width, box.height * 0.80])
                ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                        fancybox=True, ncol=2, shadow=True)
            
        self._plot_event(ax, show_events)

        if not is_subplot:
            self._display_figure(save_fig)

    def plot_rm_rate_detector_data(self, rm_ids: str|list|tuple, all_detector_ids: list|tuple, data_key: str, aggregate_data: int=10, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot ramp metering rate next to detector data.
        
        Args:
            `rm_ids` (str, list, tuple): Ramp meter junction ID or list of IDs
            `all_detector_ids` (list, tuple): List of detector IDs or nested list for multiple meters
            `data_key` (str): Plotting data key, from '_speeds_', '_vehicle_counts_' and '_occupancies_'
            `aggregate_data` (int): Averaging interval in steps (defaults to 10)
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """
        
        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        if isinstance(all_detector_ids, str): all_detector_ids = [all_detector_ids]
        validate_type(data_key, str, "data_key")

        if not isinstance(rm_ids, (list, tuple)): rm_ids = [rm_ids]
        all_detector_ids = [[det_ids] if not isinstance(det_ids, (list, tuple)) else det_ids for det_ids in all_detector_ids]

        if len(rm_ids) != len(all_detector_ids):
            desc = "Number of rm_ids '{0}' and all_detector_ids groups '{1}' do not match.".format(len(rm_ids), len(all_detector_ids))
            raise_error(ValueError, desc)

        fig_dimensions = 4 if len(rm_ids) == 1 else 3
        fig, all_axes = plt.subplots(len(rm_ids), 2, figsize=((2)*fig_dimensions, fig_dimensions*len(rm_ids)))
        
        if len(rm_ids) == 1: all_axes = [all_axes]
        else:
            new_axes = []
            new_row = []
            for col_idx in range(2):
                for rm_idx in range(len(rm_ids)):
                    new_row.append(all_axes[rm_idx][col_idx])
                new_axes.append(new_row)
                new_row = []
            all_axes = new_axes

        for rm_idx, (rm_id, detector_ids, axes) in enumerate(zip(rm_ids, all_detector_ids, all_axes)):
            self.plot_rm_rate(rm_id, axes[0],
                                    yax_labels=rm_idx==0, xax_labels=len(rm_ids)==1,
                                    show_legend=False,
                                    time_range=time_range, show_events=show_events,
                                    fig_title="Metering Rate" if len(rm_ids) == 1 else rm_id)

            for idx, ax in enumerate(axes[1:]):

                all_detector_data = []
                for det_id in detector_ids:
                    if "detectors" in self.sim_data["data"].keys():
                        if det_id in self.sim_data["data"]["detectors"].keys():
                            if data_key in self.sim_data["data"]["detectors"][det_id].keys():
                                det_data = self.sim_data["data"]["detectors"][det_id][data_key]
                                if data_key == "occupancies": all_detector_data.append([val * 100 for val in det_data])
                                else: all_detector_data.append(det_data)
                            elif data_key == "occupancies" and self.sim_data["data"]["detectors"][det_id]["type"] != "inductionloop":
                                desc = "Invalid data_key '{0}' ('{1}' detectors do not collect occupancy data).".format(data_key, self.sim_data["data"]["detectors"][det_id]["type"])
                                raise_error(KeyError, desc)
                            else:
                                desc = "Unrecognised dataset key '{0}'.".format(data_key)
                                raise_error(KeyError, desc)
                        else:
                            desc = "Unrecognised detector ID '{0}'.".format(det_id)
                            raise_error(KeyError, desc)
                    else:
                        desc = "No detector data to plot."
                        raise_error(KeyError, desc)

                if len(set([len(data) for data in all_detector_data])) == 1:
                    n_steps = len(all_detector_data[0])
                else:
                    desc = "Mismatching detector data lengths."
                    raise_error(ValueError, desc)

                avg_data = []
                for det_vals in zip(*all_detector_data):
                    _det_vals = [val for val in det_vals if val != -1]
                    avg_data.append(sum(_det_vals) / len(_det_vals) if len(_det_vals) > 0 else 0)

                time_steps = get_time_steps(avg_data, self.time_unit, step, start)
                time_steps, avg_data = limit_vals_by_range(time_steps, avg_data, time_range)
                if aggregate_data != None: avg_data, time_steps = get_aggregated_data(avg_data, time_steps, aggregate_data)

                colour, line_style = self._get_colour("WHEEL", idx==0)
                ax.plot(time_steps, avg_data, color=colour, linestyle=line_style)

                xlim = [convert_units(start, "steps", self.time_unit, step), convert_units(end, "steps", self.time_unit, step)]
                if time_range != None:
                    xlim[0], xlim[1] = max(xlim[0], time_range[0]), min(xlim[1], time_range[1])

                ax.set_xlim(xlim)
                ax.set_ylim([0, get_axis_lim(avg_data)])
                self._add_grid(ax, None)
                if rm_idx == 0: ax.set_ylabel(self._default_labels[data_key])
                ax.set_xlabel(self._default_labels["sim_time"])
                
                if len(rm_ids) == 1: ax.set_title(self._default_titles[data_key], pad=20)

                self._plot_event(ax, show_events)

        if len(rm_ids) == 1:
            fig_title = "{0}'{1}' Data".format(self.sim_label, rm_id) if not isinstance(fig_title, str) else fig_title
        else: fig_title = "{0}Ramp Metering & Detector Data".format(self.sim_label, rm_id) if not isinstance(fig_title, str) else fig_title
        if fig_title != "": fig.suptitle(fig_title, fontweight='bold')
        fig.tight_layout()
        self._display_figure(save_fig)

    def plot_rm_queuing(self, rm_id: str, ax=None, yax_labels: bool|list|tuple=True, xax_labels: bool=True, pct_capacity: bool=False, plot_delay: bool=False, cumulative_delay: bool=False, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot ramp metering rate.
        
        Args:
            `rm_id` (str): Ramp meter junction ID
            `ax` (matplotlib.axes, optional): Matplotlib axis, used when creating subplots
            `yax_labels` (bool, list, tuple): Bool denoting whether to include y-axis labels (for subplots). Either single bool for both y-axis labels or list of two bools to set both y-axes (when plotting delay).
            `xax_labels` (bool): Bool denoting whether to include x-axis labels (for subplots)
            `pct_capacity` (bool): Bool denoting whether to plot queue length as a percentage of capacity (if max_queue known)
            `plot_delay` (bool): Bool denoting whether to plot queue delay. This will be done on the same plot with a separate y-axis.
            `cumulative_delay` (bool): Bool denoting whether to plot cumulative delay
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:

            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

            if rm_id in self.simulation.tracked_junctions.keys(): tl = self.simulation.tracked_junctions[rm_id]
            else:
                desc = "Junction '{0}' not found in tracked junctions.".format(rm_id)
                raise_error(KeyError, desc)

            if tl._is_meter: 
                if tl._measure_queues:
                    queue_lengths = tl._queue_lengths
                    queue_delays = tl._queue_delays
                    max_queue = tl.max_queue
                else:
                    desc = "Meter '{0}' does not track queue lengths (no queue detector).".format(rm_id)
                    raise_error(ValueError, desc)
            else:
                desc = "Junction '{0}' is not tracked as a meter.".format(rm_id)
                raise_error(ValueError, desc)

        elif "junctions" in self.sim_data["data"].keys() and rm_id in self.sim_data["data"]["junctions"].keys():
            if "meter" in self.sim_data["data"]["junctions"][rm_id].keys():
                if "queue_lengths" in self.sim_data["data"]["junctions"][rm_id]["meter"].keys():
                    queue_lengths = self.sim_data["data"]["junctions"][rm_id]["meter"]["queue_lengths"]
                    if "queue_delays" in self.sim_data["data"]["junctions"][rm_id]["meter"]:
                        queue_delays = self.sim_data["data"]["junctions"][rm_id]["meter"]["queue_delays"]
                    else: queue_delays = None
                    if "max_queue" in self.sim_data["data"]["junctions"][rm_id]["meter"]:
                        max_queue = self.sim_data["data"]["junctions"][rm_id]["meter"]["max_queue"]
                    else: max_queue = None
                else:
                    desc = "Meter '{0}' has not tracked queue lengths (no queue detector).".format(rm_id)
                    raise_error(ValueError, desc)
            else:
                desc = "Junction '{0}' has not been tracked as a meter.".format(rm_id)
                raise_error(ValueError, desc)
        else:
            desc = "Junction '{0}' not found in tracked junctions.".format(rm_id)
            raise_error(KeyError, desc)

        if plot_delay and queue_delays == None:
            desc = f"Meter '{rm_id} has not tracked queue delay (no ramp edges)."
            raise_error(KeyError, desc)

        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        is_subplot = ax != None
        if not is_subplot: fig, ax1 = plt.subplots(1, 1)
        else: ax1 = ax

        colour = self.CYAAN
        all_data_time_vals = convert_units([x for x in range(start, end)], "steps", self.time_unit, step)
        data_time_vals, queue_lengths = limit_vals_by_range(all_data_time_vals, queue_lengths, time_range)

        if max_queue != None:
            
            if pct_capacity:
                queue_lengths = [val / max_queue * 100 for val in queue_lengths]
                ax1.set_ylim([0, 105])
                def_y_label = "Queue Length (% of Capacity)"
            else: 
                ax1.axhline(max_queue, label="Max Length", color=self.DONKERGROEN, linestyle="--", zorder=10)
                ax1.set_ylim([0, get_axis_lim(max_queue)])
                ax1.legend(shadow=True)
                def_y_label = "Queue Length (No. of Vehicles)"
        
        else:
            ax1.set_ylim([0, get_axis_lim(queue_lengths)])
            def_y_label = "Queue Length (No. of Vehicles)"

        ax1.plot(data_time_vals, queue_lengths, linewidth=1, zorder=3, color=colour)
        if xax_labels: ax1.set_xlabel(self._default_labels["sim_time"])
        if (isinstance(yax_labels, bool) and yax_labels) or (isinstance(yax_labels, (list, tuple)) and len(yax_labels) == 2 and yax_labels[0]):
            ax1.set_ylabel(def_y_label)
        else:
            desc = "Invalid yax_label, must be bool or list of 2 bools denoting each axis."
            raise_error(TypeError, desc)

        if time_range == None: time_range = [-math.inf, math.inf]
        ax1.set_xlim([max(time_range[0], data_time_vals[0]), min(time_range[1], data_time_vals[-1])])

        ax1.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)
        if not is_subplot or fig_title != None:
            if not isinstance(fig_title, str):
                default_title = "{0}'{1}' Queue Lengths".format(self.sim_label, rm_id)
                if plot_delay and cumulative_delay: default_title += " & Cumulative Delay"
                elif plot_delay: default_title += " & Delay"
                fig_title = default_title
            ax1.set_title(fig_title, pad=20)

        self._plot_event(ax1, show_events)

        if plot_delay:
            ax1.tick_params(axis='y', labelcolor=colour)
            if (isinstance(yax_labels, bool) and yax_labels) or (isinstance(yax_labels, (list, tuple)) and len(yax_labels) == 2 and yax_labels[0]):
                ax1.set_ylabel(def_y_label, color=colour)
        
            colour = self.ROOD
            data_time_vals, queue_delays = limit_vals_by_range(all_data_time_vals, queue_delays, time_range)
            if cumulative_delay: queue_delays = get_cumulative_arr(queue_delays)
            ax2 = ax1.twinx()

            ax2.plot(data_time_vals, queue_delays, linewidth=1, zorder=3, color=colour)
            ax2.tick_params(axis='y', labelcolor=colour)
            if (isinstance(yax_labels, bool) and yax_labels) or (isinstance(yax_labels, (list, tuple)) and len(yax_labels) == 2 and yax_labels[1]):
                if cumulative_delay: ax2.set_ylabel("Cumulative Delay (s)", color=colour)
                else: ax2.set_ylabel("Delay (s)", color=colour)
            else: TypeError("Plotter.plot_rm_queuing(): Invalid yax_label, must be bool or list of 2 bools denoting each axis.")
            ax2.set_ylim([0, get_axis_lim(queue_delays)])

        if not is_subplot:
            fig.tight_layout()
            self._display_figure(save_fig)

    def plot_rm_rate_queuing(self, rm_ids: str|list|tuple, plot_queuing: bool=True, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot meter queue length and delay.
        
        Args:
            `rm_ids` (str, list, tuple): Ramp meter junction ID or list of IDs
            `plot_queuing` (bool): Bool denoting whether to plot queue lengths and delay (set False to only plot metering rate)
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        if not isinstance(rm_ids, (list, tuple)): rm_ids = [rm_ids]
        
        fig_dimensions = 4
        if len(rm_ids) == 1:
            if plot_queuing:
                fig, (ax, ax2) = plt.subplots(1, 2, figsize=(fig_dimensions*2, fig_dimensions))
                self.plot_rm_queuing(rm_ids[0], ax2, True, True, True, False, time_range, show_events, fig_title="Queue Lengths & Delays")
            else: fig, ax = plt.subplots(1, 1)
            self.plot_rm_rate(rm_ids[0], ax,
                                    yax_labels=True, xax_labels=True,
                                    time_range=time_range,
                                    show_legend=False,
                                    show_events=show_events,
                                    fig_title="Metering Rate")
        
        else:
            nrows, ncols = 2 if plot_queuing else 1, len(rm_ids)
            fig, axes = plt.subplots(nrows, ncols, figsize=(ncols*fig_dimensions*1.2, nrows*fig_dimensions))

            for idx, rm_id in enumerate(rm_ids):
                ax = axes[0][idx] if plot_queuing else axes[idx]
                self.plot_rm_rate(rm_id, ax,
                                        yax_labels=idx==0,
                                        xax_labels=not plot_queuing,
                                        time_range=time_range,
                                        show_legend=False,
                                        show_events=show_events,
                                        fig_title=rm_id)
                
                if plot_queuing:
                    self.plot_rm_queuing(rm_id, axes[1][idx], (idx==0, idx==len(rm_ids)-1), True, True, False, time_range, show_events, "")

        if len(rm_ids) > 1:
            def_title = "Ramp Metering Rates"
            if plot_queuing: def_title += " & Queuing Data"
            fig_title = self.sim_label+def_title if not isinstance(fig_title, str) else fig_title
            if fig_title != "": fig.suptitle(fig_title, fontweight='bold')

        fig.tight_layout()
        self._display_figure(save_fig)

    def plot_vehicle_data(self, data_key: str, plot_cumulative: bool=False, aggregation_steps: int|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, plt_colour: str|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot network-wide vehicle data.
        
        Args:
            `data_key` (str): Data key to plot, either '_no_vehicles_', '_no_waiting_', '_tts_', '_twt_', '_avg_wt_', '_delay_', '_avg_delay_' or '_to_depart_'
            `plot_cumulative` (bool): Bool denoting whether to plot cumulative values
            `aggregation_steps` (int, optional): If given, values are aggregated using this interval
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `plt_colour` (str, optional): Line colour for plot (defaults to TUD 'cyaan')
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        if data_key not in ["no_vehicles", "no_waiting", "tts", "twt", "avg_wt", "delay", "avg_delay", "to_depart"]:
            desc = "Unrecognised data key '{0}' (must be ['no_vehicles'|'no_waiting'|'tts'|'twt'|'avg_wt'|'delay'|'avg_delay'|'to_depart']).".format(data_key)
            raise_error(KeyError, desc)

        fig, ax = plt.subplots(1, 1)
        start, step = self.sim_data["start"], self.sim_data["step_len"]

        if data_key == "avg_wt": key, avg = "no_waiting", True
        elif data_key == "avg_delay": key, avg = "delay", True
        else: key, avg = data_key, False

        y_vals = self.sim_data["data"]["vehicles"][key]
        if avg: y_vals = [y_val / n_vehicles for y_val, n_vehicles in zip(y_vals, self.sim_data["data"]["vehicles"]["no_vehicles"])]

        if plot_cumulative: y_vals = get_cumulative_arr(y_vals)
        x_vals = get_time_steps(y_vals, self.time_unit, step, start)
        x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)

        if aggregation_steps != None:
            y_vals, x_vals = get_aggregated_data(y_vals, x_vals, aggregation_steps)

        ax.plot(x_vals, y_vals, color=self._get_colour(plt_colour))

        if fig_title == None:
            fig_title = "Network-wide "+self._default_titles[data_key]
            if plot_cumulative: fig_title = "Cumulative "+fig_title
            fig_title = self.sim_label + fig_title
        ax.set_title(fig_title, pad=20)

        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel(self._default_labels[data_key])
        ax.set_xlim([x_vals[0], x_vals[-1]])
        ax.set_ylim([0, get_axis_lim(y_vals)])
        self._add_grid(ax, None)

        self._plot_event(ax, show_events)
        
        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_detector_data(self, detector_id: str, data_key: str, plot_cumulative: bool=False, aggregation_steps: int|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, plt_colour: str|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot detector data.
        
        Args:
            `detector_id` (str): Detector ID
            `data_key` (str): Data key to plot, either '_speeds_', '_vehicle_counts_' or '_occupancies_'
            `plot_cumulative` (bool): Bool denoting whether to plot cumulative values
            `aggregation_steps` (int, optional): If given, values are aggregated using this interval
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `plt_colour` (str, optional): Line colour for plot (defaults to TUD 'cyaan')
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """
        
        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        fig, ax = plt.subplots(1, 1)
        start, step = self.sim_data["start"], self.sim_data["step_len"]

        if data_key not in ["speeds", "vehicle_counts", "occupancies"]:
            desc = "Unrecognised data key '{0}' (must be [speeds|vehicle_counts|occupancies]).".format(data_key)
            raise_error(KeyError, desc)
        elif detector_id not in self.sim_data["data"]["detectors"].keys():
            desc = "Detector ID '{0}' not found.".format(detector_id)
            raise_error(KeyError, desc)
        elif data_key == "occupancies" and self.sim_data["data"]["detectors"][detector_id]["type"] == "multientryexit":
            desc = "Multi-Entry-Exit Detectors ('{0}') do not measure '{1}'.".format(detector_id, data_key)
            raise_error(ValueError, desc)
        
        y_vals = self.sim_data["data"]["detectors"][detector_id][data_key]
        if data_key == "occupancies": y_vals = [val * 100 for val in y_vals]
        if plot_cumulative: y_vals = get_cumulative_arr(y_vals)
        x_vals = get_time_steps(y_vals, self.time_unit, step, start)
        x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)

        if aggregation_steps != None:
            y_vals, x_vals = get_aggregated_data(y_vals, x_vals, aggregation_steps)

        ax.plot(x_vals, y_vals, color=self._get_colour(plt_colour))

        if fig_title == None:
            fig_title = "{0} (Detector '{1}')".format(self._default_titles[data_key], detector_id)
            if plot_cumulative: fig_title = "Cumulative "+fig_title
            fig_title = self.sim_label + fig_title
        ax.set_title(fig_title, pad=20)

        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel(self._default_labels[data_key])
        ax.set_xlim([x_vals[0], x_vals[-1]])
        if data_key == "occupancies": ax.set_ylim([0, 100])
        else: ax.set_ylim([0, get_axis_lim(y_vals)])
        self._add_grid(ax, None)

        self._plot_event(ax, show_events)
        
        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_edge_data(self, edge_id: str, data_key: str, plot_cumulative: bool=False, aggregation_steps: int|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, plt_colour: str|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot tracked egde data.
        
        Args:
            `edge_id` (str): Tracked edge ID
            `data_key` (str): Data key to plot, either '_flows_', '_speeds_', '_densities_', '_occupancies_', '_vehicle_counts_'
            `plot_cumulative` (bool): Bool denoting whether to plot cumulative values
            `aggregation_steps` (int, optional): If given, values are aggregated using this interval
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `plt_colour` (str, optional): Line colour for plot (defaults to TUD 'cyaan')
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """
        
        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        fig, ax = plt.subplots(1, 1)
        start, step = self.sim_data["start"], self.sim_data["step_len"]

        if "edges" not in self.sim_data["data"].keys():
            desc = "No TrackedEdge data found."
            raise_error(KeyError, desc)
        elif data_key not in ["flows", "speeds", "densities", "occupancies", "vehicle_counts"]:
            desc = "Unrecognised data key '{0}' (must be [flows|speeds|densities|occupancies|vehicle_counts]).".format(data_key)
            raise_error(KeyError, desc)
        elif edge_id not in self.sim_data["data"]["edges"].keys():
            desc = "Edge ID '{0}' not found.".format(edge_id)
            raise_error(KeyError, desc)
        
        y_vals = self.sim_data["data"]["edges"][edge_id][data_key if data_key != "vehicle_counts" else "step_vehicles"]
        if data_key == "occupancies": y_vals = [val * 100 for val in y_vals]
        elif data_key == "vehicle_counts": y_vals = [len(step_data) for step_data in y_vals]
        if plot_cumulative: y_vals = get_cumulative_arr(y_vals)
        x_vals = get_time_steps(y_vals, self.time_unit, step, start)
        x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)

        if aggregation_steps != None:
            y_vals, x_vals = get_aggregated_data(y_vals, x_vals, aggregation_steps)

        ax.plot(x_vals, y_vals, color=self._get_colour(plt_colour))

        if fig_title == None:
            fig_title = "'{0}' {1}{2}".format(edge_id, "Cumulative " if plot_cumulative else "", self._default_titles[data_key])
            fig_title = self.sim_label + fig_title
        ax.set_title(fig_title, pad=20)

        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel(self._default_labels[data_key])
        ax.set_xlim([x_vals[0], x_vals[-1]])
        if data_key == "occupancies": ax.set_ylim([0, 100])
        else: ax.set_ylim([0, get_axis_lim(y_vals)])
        self._add_grid(ax, None)

        self._plot_event(ax, show_events)
        
        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_demand(self, routing: str|list|tuple|None=None, demand_profiles: DemandProfile|list|tuple|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, plt_colour: str|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plots demand from TUD-SUMO DemandProfile(s). By default, all demand profiles are plotted.
        Demand defined within '_.rou.xml_' files is not plotted.

        Args:
            `routing` (str, list, tuple, optional): Either string (route ID or 'all'), OD pair eg. ('A', 'B') or None (defaulting to all)
            `demand_profiles` (DemandProfile, list, tuple, optional): Either DemandProfile object or list of DemandProfiles (defaults to all added to simulation)
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `plt_colour` (str, optional): Line colour for plot (defaults to TUD 'cyaan')
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name
            step_len = self.simulation.step_length

        dps = []
        if isinstance(demand_profiles, DemandProfile): dps.append(demand_profiles)
        elif isinstance(demand_profiles, (list, tuple)):
            validate_list_types(demand_profiles, DemandProfile, param_name='demand_profiles')
            dps = list(demand_profiles)
        elif demand_profiles == None:
            
            if self.simulation != None and len(self.simulation._demand_profiles) > 0:
                    dps = list(self.simulation._demand_profiles.values())

            elif "demand" in self.sim_data["data"]:
                dps = self.sim_data["data"]["demand"]["profiles"]
                step_len = self.sim_data["step_len"]

            if len(dps) == 0:
                desc = "Cannot plot demand (no demand profiles found)."
                raise_error(KeyError, desc)

        else:
            desc = f"Invalid demand_profiles (must be [DemandProfile|list|tuple|None], not '{type(demand_profiles)}')"
            raise_error(TypeError, desc)

        dps = [dp._demand_arrs if isinstance(dp, DemandProfile) else dp for dp in dps]

        start_times, end_times = [], []
        for dp in dps:
            for arr in dp:
                start_times.append(arr[1][0])
                end_times.append(arr[1][1])

        if time_range == None:
            time_range = [min(start_times), max(end_times)]
        else:
            validate_list_types(time_range, (int, float), param_name='time_range')
            if time_range[1] <= min(start_times) or time_range[0] >= max(end_times) or time_range[0] >= time_range[1]:
                desc = f"Invalid time_range (no demand within range '{time_range[0]} - {time_range[1]}')."
                raise_error(ValueError, desc)

        demand_vals = [0] * int((time_range[1] - time_range[0]) / step_len + 1)
        time_vals = [idx * step_len + time_range[0] for idx, _ in enumerate(demand_vals)]

        added_demand = False
        for demand_profile in dps:
            for demand_arr in demand_profile:

                arr_routing, demand_val = demand_arr[0], demand_arr[2]
                arr_start, arr_end = int(demand_arr[1][0]), int(demand_arr[1][1])

                if routing != None:
                    if isinstance(routing, str) and not isinstance(arr_routing, str):
                        if routing != "all": continue

                    elif isinstance(routing, str) and isinstance(arr_routing, str):
                        if routing != arr_routing and routing != "all": continue
                        
                    elif isinstance(routing, (list, tuple)) and isinstance(arr_routing, (list, tuple)):
                        if routing[0] != arr_routing[0] or routing[1] != arr_routing[1]: continue

                if arr_start > time_range[1] or arr_end < time_range[0]: continue

                arr_start = max(int((arr_start - time_range[0]) / step_len), 0)
                arr_end = min(int((arr_end - time_range[0]) / step_len), len(demand_vals) - 1)
                
                for idx in range(arr_start, arr_end + 1): demand_vals[idx] += demand_val
                added_demand = True

        if not added_demand:
            desc = "Cannot plot demand (empty demand profiles)."
            raise_error(KeyError, desc)

        time_vals = convert_units(time_vals, "seconds", self.time_unit, step_len)
        fig, ax = plt.subplots(1, 1)

        ax.plot(time_vals, demand_vals, color=self._get_colour(plt_colour))
        ax.set_ylabel("Demand (vehicles/hour)")
        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_xlim([time_vals[0], time_vals[-1]])
        ax.set_ylim([0, get_axis_lim(demand_vals)])
        self._add_grid(ax, None)
        if fig_title == None:
            if routing == "all" or routing == None: fig_title = "Network-wide Demand"
            else:
                if isinstance(routing, str): fig_title = "Route '{0}' Demand".format(routing)
                else: fig_title = "OD Demand ('{0}')".format(' → '.join(routing))
        fig_title = self.sim_label + fig_title
        ax.set_title(fig_title, pad=20)

        if self.sim_data != None and "data" in self.sim_data:
            self._plot_event(ax, show_events)
        
        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_od_trip_times(self, od_pairs: list|tuple|None=None, vehicle_types: list|tuple|None=None, ascending_vals: bool=True, trip_time_unit: str="minutes", time_range: list|tuple|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plots average trip times for Origin-Destination pairs.
        
        Args:
            `od_pairs` (list, tuple, optional): (n x 2) list containing OD pairs. If not given, all OD pairs are plotted
            `vehicle_types` (list, tuple, optional): List of vehicle types for included trips (defaults to all)
            `ascending_vals` (bool): If `True`, the largest values are plotted in the bottom-right, if `False`, top-left
            `trip_time_unit` (str): Time unit for displaying values, must be ['_seconds_'|'_minutes_'|'_hours_'], defaults to '_minutes_'
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """
        
        if trip_time_unit not in ["seconds", "minutes", "hours"]:
            desc = "Invalid time unit '{0}' (must be ['seconds'|'minutes'|'hours']).".format(trip_time_unit)
            raise_error(ValueError, desc)
        
        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        step = self.sim_data["step_len"]

        od_trip_times, add_new = {}, od_pairs == None
        all_origins, all_destinations = set([]), set([])
        if od_pairs != None:
            for pair in od_pairs:
                od_trip_times[pair[0]] = {pair[1]: []}
                all_origins.add(pair[0])
                all_destinations.add(pair[1])

        n_trips = 0
        com_trip_data = self.sim_data["data"]["trips"]["completed"]
        for trip in com_trip_data.values():
            origin, destination = trip["origin"], trip["destination"]
            veh_type = trip["vehicle_type"]

            if vehicle_types != None and veh_type not in vehicle_types: continue
            
            if origin not in od_trip_times.keys():
                if add_new:
                    od_trip_times[origin] = {}
                else: continue
            
            if destination not in od_trip_times[origin].keys():
                if add_new: od_trip_times[origin][destination] = []
                else: continue

            trip_time = convert_units(trip["arrival"] - trip["departure"], "steps", trip_time_unit, step)
            trip_departure, trip_arrival = convert_units([trip["departure"], trip["arrival"]], "steps", self.time_unit, step)

            if time_range != None and trip_departure < time_range[0] and trip_arrival > time_range[1]:
                continue

            od_trip_times[origin][destination].append(trip_time)
            all_origins.add(origin)
            all_destinations.add(destination)
            n_trips += 1
        
        if n_trips == 0:
            desc = "No trips found."
            raise_error(ValueError, desc)

        all_origins = list(all_origins)
        all_destinations = list(all_destinations)

        if add_new:
            avg_o_tts = []
            for o in all_origins:
                o_tts = [sum(d)/len(d) for d in od_trip_times[o].values()]
                avg_o_tts.append(sum(o_tts)/len(o_tts))
            
            all_origins = [x for _, x in sorted(zip(avg_o_tts, all_origins))]
            
            avg_d_tts = []
            for d in all_destinations:
                d_tts = []
                for o_data in od_trip_times.values():
                    if d in o_data.keys():
                        d_tts.append(sum(o_data[d])/len(o_data[d]))
                avg_d_tts.append(sum(d_tts)/len(d_tts))

            all_destinations = [x for _, x in sorted(zip(avg_d_tts, all_destinations))]

        att_matrix = np.empty((len(all_origins), len(all_destinations)))
        att_matrix[:] = np.nan
        
        if add_new and not ascending_vals:
            all_origins.reverse()
            all_destinations.reverse()

        for i, origin in enumerate(all_origins):
            for j, destination in enumerate(all_destinations):
                if destination in od_trip_times[origin].keys():
                    trip_times = od_trip_times[origin][destination]
                    att_matrix[i][j] = sum(trip_times) / len(trip_times)

        fig, ax = plt.subplots(1, 1, figsize=(6, 6))
        masked_array = np.ma.array(att_matrix, mask=np.isnan(att_matrix))
        cmap = matplotlib.cm.Reds
        cmap.set_bad('#f7f7f7')
        ax.matshow(masked_array, interpolation='nearest', cmap=cmap)

        ax.set_xticks(np.arange(len(all_destinations)), labels=all_destinations)
        ax.set_yticks(np.arange(len(all_origins)), labels=all_origins)
        ax.xaxis.set_ticks_position("bottom")
        ax.xaxis.set_label_position("top")
        ax.yaxis.set_label_position("right")

        for row in range(att_matrix.shape[0]):
            for col in range(att_matrix.shape[1]):
                if not np.isnan(att_matrix[row, col]):
                    ax.text(x=col, y=row, s=round(att_matrix[row, col], 2) if trip_time_unit != "seconds" else int(att_matrix[row, col]),
                            va='center', ha='center', color='white', path_effects=[pe.withStroke(linewidth=2, foreground="black")]) 

        plt.setp(ax.get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")

        ax.set_xlabel("Destination ID")
        ax.set_ylabel("Origin ID")
        
        if fig_title == None:
            fig_title = self.sim_label + "Average Trip Times ({0})".format(trip_time_unit)
        ax.set_title(fig_title, pad=30, fontweight='bold')

        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_cumulative_curve(self, inflow_detectors: list|tuple|None=None, outflow_detectors: list|tuple|None=None, outflow_offset: int|float=0, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot inflow and outflow cumulative curves, either system-wide or using inflow/outflow detectors (if given).
        
        Args:
            `inflow_detectors` (list, tuple, optional): List of inflow detectors
            `outflow_detectors` (list, tuple, optional): List of outflow detectors
            `outflow_offset` (int, float): Offset for outflow values if not starting at t=0
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        inflows, outflows = [], []
        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        if time_range == None: time_range = [-math.inf, math.inf]

        if inflow_detectors == None and outflow_detectors == None:

            inflows, outflows = [0] * (end - start+1), [0] * (end - start+1)

            trips = self.sim_data["data"]["trips"]
            for inc_trip in trips["incomplete"].values():
                inflows[inc_trip["departure"] - start] += 1
            
            for com_trip in trips["completed"].values():
                inflows[com_trip["departure"] - start] += 1
                outflows[com_trip["arrival"] - start] += 1

            x_vals = get_time_steps(inflows, self.time_unit, step, start)
            _, inflows = limit_vals_by_range(x_vals, inflows, time_range)
            x_vals, outflows = limit_vals_by_range(x_vals, outflows, time_range)

        else:
            if inflow_detectors == None or outflow_detectors == None:
                desc = "When using detectors, both inflow and outflow detectors are required."
                raise_error(TypeError, desc)
            
            if "detectors" not in self.sim_data["data"].keys():
                desc = "No detector data to plot."
                raise_error(KeyError, desc)
            
            detector_data = self.sim_data["data"]["detectors"]
            if not isinstance(inflow_detectors, (list, tuple)): inflow_detectors = [inflow_detectors]
            if not isinstance(outflow_detectors, (list, tuple)): outflow_detectors = [outflow_detectors]

            if len(set(inflow_detectors + outflow_detectors) - set(detector_data.keys())) != 0:
                desc = "Detectors ['{0}'] could not be found.".format("', '".join(list(set(inflow_detectors + outflow_detectors) - set(detector_data.keys()))))
                raise_error(KeyError, desc)

            prev_in_vehicles, prev_out_vehicles = set([]), set([])
            for step_no in range(self.sim_data["end"] - self.sim_data["start"]):

                curr_time = convert_units(step_no, "steps", self.time_unit, step)
                
                if curr_time >= time_range[0] and curr_time <= time_range[1]:

                    vehs_in, vehs_out = set([]), set([])

                    for detector_id in inflow_detectors: vehs_in = vehs_in | set(detector_data[detector_id]["vehicle_ids"][step_no])
                    for detector_id in outflow_detectors: vehs_out = vehs_out | set(detector_data[detector_id]["vehicle_ids"][step_no])

                    inflows.append(len(vehs_in - prev_in_vehicles))
                    outflows.append(len(vehs_out - prev_out_vehicles))

                    prev_in_vehicles = prev_in_vehicles | vehs_in
                    prev_out_vehicles = prev_out_vehicles | vehs_out

                elif curr_time > time_range[1]:
                    break

            start = max(time_range[0], start)
            x_vals = get_time_steps(inflows, self.time_unit, step, start)

        inflows = get_cumulative_arr(inflows)
        outflows = get_cumulative_arr(outflows)
        outflows = [val - outflow_offset for val in outflows]

        fig, ax = plt.subplots(1, 1)
        fig_title = "{0}Cumulative Arrival-Departure Curve".format(self.sim_label) if not isinstance(fig_title, str) else fig_title
        ax.set_title(fig_title, pad=20)
        
        ax.plot(x_vals, inflows, color=self.DONKERGROEN, label="Inflow", zorder=3)
        ax.plot(x_vals, outflows, color=self.ROOD, label="Outflow", zorder=4)
        ax.set_xlim([x_vals[0], x_vals[-1]])
        ax.set_ylim([0, get_axis_lim(inflows)])
        self._add_grid(ax, None)
        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel("Cumulative No. of Vehicles")
        ax.legend(loc='lower right', shadow=True)

        self._plot_event(ax, show_events)

        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_vsl_data(self, vsl_id: str, avg_geomtry_speeds: bool=False, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot VSL settings and average vehicle speeds on affected edges.
        
        Args:
            `vsl_id` (str): VSL controller ID
            `avg_geometry_speeds` (bool): Bool denoting whether to plot average edge speed, or individual edge data
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        if "controllers" not in self.sim_data["data"].keys():
            desc = "No controllers used during simulation."
            raise_error(KeyError, desc)
        elif vsl_id not in self.sim_data["data"]["controllers"].keys():
            desc = "Controller ID '{0}' not found.".format(vsl_id)
            raise_error(KeyError, desc)
        
        vsl_data = self.sim_data["data"]["controllers"][vsl_id]
        if vsl_data["type"] != "VSL":
            desc = "Controller '{0}' is not a VSL controller.".format(vsl_id)
            raise_error(KeyError, desc)
        
        start, end, step = vsl_data["init_time"], vsl_data["curr_time"], self.sim_data["step_len"]
        
        colour = self.DONKERGROEN
        activation_times = vsl_data["activation_times"]

        if len(activation_times) == 0:
            desc = "VSL controller '{0}' has no data, likely was not activated.".format(vsl_id)
            raise_error(ValueError, desc)

        fig, ax = plt.subplots(1, 1)

        prev = None
        activated = False
        active_times, activated_time = [], None
        label = "Speed Limit"
        linewidth = 1.5 if avg_geomtry_speeds else 2
        for idx, (val, time) in enumerate(activation_times):
            if prev == None: prev = val
            else:
                if prev != -1 and val != -1:
                    ax.plot(convert_units([time, time], "steps", self.time_unit, step), [prev, val], color=colour, alpha=0.8, linewidth=linewidth, label=label, zorder=3)
                    label = None
                
            if val != -1:
                if not activated:
                    activated = True
                    if activated_time == None: activated_time = time

                if idx == len(activation_times) - 1: line_x_lim = end
                else: line_x_lim = activation_times[idx+1][1]
                ax.plot(convert_units([time, line_x_lim], "steps", self.time_unit, step), [val, val], color=colour, alpha=0.8, linewidth=linewidth, zorder=3)
            else:
                active_times.append(convert_units([activated_time, time], "steps", self.time_unit, step))
                activated, activated_time = False, None
            
            prev = val

        label = "VSL Activated"
        for ranges in active_times:
            for time in ranges:
                ax.axvline(time, color="grey", alpha=0.2, linestyle='--')
            ax.axvspan(ranges[0], ranges[1], color="grey", alpha=0.1, label=label)
            label = None
        
        edge_ids = list(vsl_data["geometry_data"].keys())
        edge_speeds = [vsl_data["geometry_data"][e_id]["avg_speeds"] for e_id in edge_ids]
        n_edges = len(edge_speeds)
        if avg_geomtry_speeds:
            avg_speeds = []
            for time_idx in range(len(edge_speeds[0])):
                all_pos_vals = [edge_speeds[edge_idx][time_idx] for edge_idx in range(n_edges) if edge_speeds[edge_idx][time_idx] != -1]
                if len(all_pos_vals) == 0: avg_speeds.append(-1)
                else: avg_speeds.append(sum(all_pos_vals) / len(all_pos_vals))

            edge_speeds = [avg_speeds]

        max_speed = max([arr[0] for arr in activation_times])
        for edge_idx, edge in enumerate(edge_speeds):
            if avg_geomtry_speeds: label = "Avg. Edge Speed"
            else: label = "'{0}' Speed".format(edge_ids[edge_idx])
            prev_line = None
            x_vals, y_vals = [], []
            curr_time = start
            for speed_val in edge:
                max_speed = max(max_speed, speed_val)
                if speed_val == -1:
                    if len(x_vals) != 0:
                        if prev_line == None:
                            colour, line_style = self._get_colour("WHEEL", edge_idx==0)
                            prev_line, label = ax.plot(convert_units(x_vals, "steps", self.time_unit, step), y_vals, color=colour, linestyle=line_style, label=label, linewidth=1), None
                        else: prev_line = ax.plot(convert_units(x_vals, "steps", self.time_unit, step), y_vals, color=prev_line[0].get_color(), linestyle=prev_line[0].get_linestyle(), label=label, linewidth=1)
                        x_vals, y_vals = [], []
                else:
                    x_vals.append(curr_time)
                    y_vals.append(speed_val)

                curr_time += 1

            if len(x_vals) != 0 and len(y_vals) != 0:
                colour, line_style = self._get_colour("WHEEL", edge_idx==0)
                if prev_line == None: prev_line = ax.plot(convert_units(x_vals, "steps", self.time_unit, step), y_vals, color=colour, linestyle=line_style, label=label, linewidth=1)
                else: prev_line = ax.plot(convert_units(x_vals, "steps", self.time_unit, step), y_vals, color=prev_line[0].get_color(), linestyle=prev_line[0].get_linestyle(), label=label, linewidth=1)

        self._plot_event(ax, show_events)

        y_lim = get_axis_lim(max_speed)
        ax.set_ylim(0, y_lim)
        xlim = convert_units([start, end], "steps", self.time_unit, step)
        if time_range != None: xlim = [max(xlim[0], time_range[0]), min(xlim[1], time_range[1])]
        ax.set_xlim(xlim)

        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel(self._default_labels["limits"])

        fig_title = "{0}'{1}' Speed Limit and Average Vehicle Speed".format(self.sim_label, vsl_id) if not isinstance(fig_title, str) else fig_title
        ax.set_title(fig_title, pad=20)
        
        box = ax.get_position()
        ax.set_position([box.x0, box.y0 + box.height * 0.02,
                        box.width, box.height * 0.98])

        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.14),
          fancybox=True, ncol=3, shadow=True)

        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_rg_data(self, rg_id: str, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot how many vehicles are diverted by RG controller.
        
        Args:
            `rg_id` (str): RG controller ID
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        if "controllers" not in self.sim_data["data"].keys():
            desc = "No controllers used during simulation."
            raise_error(KeyError, desc)
        elif rg_id not in self.sim_data["data"]["controllers"].keys():
            desc = "Controller ID '{0}' not found.".format(rg_id)
            raise_error(KeyError, desc)
        
        rg_data = self.sim_data["data"]["controllers"][rg_id]
        if rg_data["type"] != "RG":
            desc = "Controller '{0}' is not a RG controller.".format(rg_id)
            raise_error(ValueError, desc)
        
        start, end, step = rg_data["init_time"], rg_data["curr_time"], self.sim_data["step_len"]
        y_vals = get_cumulative_arr(rg_data["n_diverted"])

        if len(y_vals) == 0:
            desc = "RG controller '{0}' has no data, likely was not activated.".format(rg_id)
            raise_error(ValueError, desc)
        
        fig, ax = plt.subplots(1, 1)

        x_vals = get_time_steps(y_vals, self.time_unit, step, start)
        x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)
        ax.plot(x_vals, y_vals, color=self.BLAUW, zorder=8, label="Diverted Vehicles")
        ax.set_xlim([x_vals[0], x_vals[-1]])
        y_lim = get_axis_lim(y_vals)
        ax.set_ylim([0, y_lim])
        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel("No. of Diverted Vehicles")
        self._add_grid(ax, None)

        active_times = rg_data["activation_times"]
        label = "RG Activated"
        active_ranges, active = [], False
        for arrs in active_times:
            if not active:
                if arrs[0] != -1:
                    active = True
                    active_ranges.append(arrs[2])
            else:
                if arrs[0] == -1:
                    active = False
                    start_val = active_ranges[-1]
                    active_ranges[-1] = [start_val, arrs[2]]
        
        if isinstance(active_ranges[-1], (int, float)):
            start_val = active_ranges[-1]
            active_ranges[-1] = [start_val, end]

        label = "RG Active"
        for ranges in active_ranges:
            ranges = convert_units(ranges, "steps", self.time_unit, step, start)
            for time in ranges:
                ax.axvline(time, color=self.CYAAN, alpha=0.3, linestyle='--')
            ax.axvspan(ranges[0], ranges[1], zorder=1, color=self.CYAAN, alpha=0.2, label=label)
            label=None

        fig_title = "{0}'{1}' Number of Diverted Vehicles".format(self.sim_label, rg_id) if not isinstance(fig_title, str) else fig_title
        ax.set_title(fig_title, pad=20)
        ax.legend(shadow=True)

        self._plot_event(ax, show_events)

        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_space_time_diagram(self, edge_ids: list|tuple, upstream_at_top: bool=True, dist_labels: list|tuple|None=None, time_range: list|tuple|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot space time data from tracked edge data.
        
        Args:
            `edge_ids` (list, tuple): Single tracked egde ID or list of IDs
            `upstream_at_top` (bool): If `True`, upstream values are displayed at the top of the diagram
            `dist_labels` (list, tuple, optional): A list of labels and distances (km/mi) to be plotted on the graph (as a list of (str, [int|float]) pairs)
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        if "edges" not in self.sim_data["data"].keys():
            desc = "No edges tracked during the simulation."
            raise_error(KeyError, desc)
        
        if not isinstance(edge_ids, (list, tuple)): edge_ids = [edge_ids]

        fig, ax = plt.subplots(1, 1)
        edge_offset = 0

        total_len = sum([self.sim_data["data"]["edges"][e_id]["length"] for e_id in edge_ids])

        if self.units in ["IMPERIAL"]:
            orig_units, new_units = "miles", "miles" if total_len > 1 else "feet"
        elif self.units in ["METRIC", "UK"]:
            orig_units, new_units = "kilometres", "kilometres" if total_len > 1 else "metres"

        x_label, y_label = self._default_labels["sim_time"], self._default_labels[new_units]

        if time_range == None: time_range = [-math.inf, math.inf]

        ordered_points = {}
        for e_id in edge_ids:
            if e_id not in self.sim_data["data"]["edges"].keys():
                desc = "Edge '{0}' not found in tracked edges.".format(e_id)
                raise_error(KeyError, desc)
            else: e_data = self.sim_data["data"]["edges"][e_id]

            step_vehicles, edge_length = e_data["step_vehicles"], e_data["length"]
            start, step = e_data["init_time"], self.sim_data["step_len"]

            curr_step = start

            for step_data in step_vehicles:
                curr_time = convert_units(curr_step, "steps", self.time_unit, step)
                if curr_time <= time_range[1] and curr_time >= time_range[0]:
                    for veh_data in step_data:

                        y_val = (veh_data[1] * edge_length) + edge_offset
                        if not upstream_at_top: y_val = total_len - y_val
                        y_val = convert_units(y_val, orig_units, new_units)

                        if curr_step not in ordered_points.keys():
                            ordered_points[curr_step] = [(y_val, veh_data[2])]
                        else: ordered_points[curr_step].append((y_val, veh_data[2]))
                        
                elif curr_time > time_range[1]:
                    break

                curr_step += 1

            edge_offset += edge_length

        idxs = ordered_points.keys()
        x_vals, y_vals, speed_vals = [], [], []
        for idx in idxs:
            x_vals += [convert_units(idx, "steps", self.time_unit, step)] * len(ordered_points[idx])
            dist_speed = ordered_points[idx]
            y_vals += [val[0] for val in dist_speed]
            speed_vals += [val[1] for val in dist_speed]

        if len(x_vals) == 0 or len(y_vals) == 0:
            if time_range == None:
                desc = "No data to plot (no vehicles recorded on edges)."
                raise_error(ValueError, desc)
            else:
                desc = "No data to plot (no vehicles recorded during time frame '{0}-{1}{2}').".format(time_range[0], time_range[1], self.time_unit)
                raise_error(ValueError, desc)
        
        points = ax.scatter(x_vals, y_vals, c=speed_vals, s=0.5, cmap='hot', zorder=1)

        if dist_labels != None:
            validate_list_types(dist_labels, (list, tuple), param_name='dist_labels')
            for dist_label in dist_labels:
                if not isinstance(dist_label, (list, tuple)) or len(dist_label) != 2:
                    desc = f"Invalid dist_label '{dist_label}' (must be length 2: [str, (int, float)])."
                    raise_error(TypeError, desc)
                elif not isinstance(dist_label[0], (str, type(None))) or not isinstance(dist_label[1], (int, float)):
                    desc = f"Invalid dist_label '{dist_label}' (must be type [str, (int, float)])."
                    raise_error(TypeError, desc)

                label, dist = dist_label
                dist = convert_units(dist, orig_units, new_units)
                
                ax.plot([min(x_vals), max(x_vals)], [dist, dist], linestyle='--', color='black', linewidth=1.5, zorder=10)
                padding = (max(x_vals) - min(x_vals)) * 0.02
                if label != "" and label != None:
                    ax.text(padding + min(x_vals), dist, label, fontsize=6,
                            zorder=20, horizontalalignment='left', verticalalignment='center', color='red',
                            bbox=dict(facecolor='white', alpha=0.9, linewidth=0))

        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="5%", pad=0.1)
        plt.colorbar(points, cax=cax, label=self._default_labels["speed"])

        ax.set_xlim(min(x_vals), max(x_vals))
        ax.set_ylim(0, max(y_vals))
        ax.set_ylabel(y_label)
        ax.set_xlabel(x_label)

        if not isinstance(fig_title, str):
            if len(edge_ids) == 0: e_label = "Edge '{0}'".format(edge_ids[0])
            elif upstream_at_top: e_label = "Edges '{0}' - '{1}'".format(edge_ids[-1], edge_ids[0])
            else: e_label = "Edges '{0}' - '{1}'".format(edge_ids[0], edge_ids[-1])
            fig_title = "{0}{1}".format(self.sim_label, e_label)

        ax.set_title(fig_title, pad=20)

        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_trajectories(self, edge_ids: str|list|tuple, lane_idx: int|None=None, vehicle_pct: float=1, rnd_seed: int|None=None, dist_labels: list|tuple|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, plt_colour: str|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot vehicle trajectory data from tracked edge data.
        
        Args:
            `edge_ids` (str, list, tuple): Edge ID or list of IDs
            `lane_idx` (int, optional): Lane index for vehicles on all edges
            `vehicle_pct` (float): Percent of vehicles plotted (defaults to all)
            `rnd_seed` (int, optional): When `vehicle_pct < 1`, vehicles are selected randomly with `rnd_seed`
            `dist_labels` (list, tuple, optional): A list of labels and distances (km/mi) to be plotted on the graph (as a list of (str, [int|float]) pairs)
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `plt_colour` (str, optional): Line colour for plot (defaults to TUD 'cyaan')
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        if not isinstance(edge_ids, (list, tuple)): edge_ids = [edge_ids]

        total_len = 0
        if "edges" not in self.sim_data["data"].keys():
            desc = "No edges tracked during the simulation."
            raise_error(KeyError, desc)
        else:
            for edge_id in edge_ids:
                if edge_id in self.sim_data["data"]["edges"].keys():
                    total_len += self.sim_data["data"]["edges"][edge_id]["length"]
                else:
                    desc = "Edge ID '{0}' not found.".format(edge_id)
                    raise_error(KeyError, desc)

        if time_range == None: time_range = [-math.inf, math.inf]
        x_lim, y_lim = [math.inf, -math.inf], [math.inf, -math.inf]

        if self.units in ["IMPERIAL"]:
            orig_units, new_units = "miles", "miles" if total_len > 1 else "feet"
        elif self.units in ["METRIC", "UK"]:
            orig_units, new_units = "kilometres", "kilometres" if total_len > 1 else "metres"

        seed(rnd_seed)

        included_vehs, skipped_vehs = set([]), set([])
        step_length = self.sim_data["step_len"]
        all_step_vehicle_data, edge_offset = [], 0

        for edge_idx, edge_id in enumerate(edge_ids):
            edge_data = self.sim_data["data"]["edges"][edge_id]

            step_vehicle_data, edge_length = edge_data["step_vehicles"], edge_data["length"]
            start = edge_data["init_time"]

            curr_step, first_step = start, True

            for step_data in step_vehicle_data:

                curr_time = convert_units(curr_step, "steps", self.time_unit, step_length)
                
                if curr_time < time_range[0] or curr_time > time_range[1]:
                    curr_step += 1
                    continue

                for vehicle_data in step_data:
                    vehicle_id, vehicle_pos, vehicle_lane = vehicle_data[0], vehicle_data[1], vehicle_data[3]
                    
                    if edge_idx == 0 or first_step:
                        if vehicle_id not in included_vehs and vehicle_id not in skipped_vehs:
                            if random() <= vehicle_pct: included_vehs.add(vehicle_id)
                            else: skipped_vehs.add(vehicle_id)
                    
                    if vehicle_id in included_vehs:
                        dist_val = (vehicle_pos * edge_length) + edge_offset
                        all_step_vehicle_data.append([vehicle_id, curr_time, dist_val, vehicle_lane])

                first_step = False
                curr_step += 1
            
            edge_offset += edge_length
        
        all_step_vehicle_data = sorted(all_step_vehicle_data, key=lambda x: x[1])

        fig, ax = plt.subplots(1, 1)
        plotted = False

        if len(all_step_vehicle_data) == 0:
            desc = "No vehicles found within range '[{0}]'. ".format(", ".join([str(val) for val in time_range]))
            raise_error(ValueError, desc)

        curr_time = all_step_vehicle_data[0][1]
        line_x_vals, line_y_vals = {}, {}
        entry_x, entry_y, exit_x, exit_y = [], [], [], []
        for vehicle_data in all_step_vehicle_data:
            vehicle_id, vehicle_time, vehicle_pos, vehicle_lane = vehicle_data
            vehicle_pos = convert_units(vehicle_pos, orig_units, new_units)

            if lane_idx == None or vehicle_lane == lane_idx:
                if vehicle_id not in line_x_vals and vehicle_id not in line_y_vals:
                    line_x_vals[vehicle_id] = []
                    line_y_vals[vehicle_id] = []

                    if lane_idx != None and vehicle_time > x_lim[0]:
                        entry_x.append(vehicle_time)
                        entry_y.append(vehicle_pos)
                
                line_x_vals[vehicle_id].append(vehicle_time)
                line_y_vals[vehicle_id].append(vehicle_pos)

                x_lim = [min(x_lim[0], vehicle_time), max(x_lim[1], vehicle_time)]
                y_lim = [min(y_lim[0], vehicle_pos), max(y_lim[1], vehicle_pos)]
            
            elif vehicle_id in line_x_vals and vehicle_id in line_y_vals:
                if len(line_x_vals[vehicle_id]) > 1:
                    ax.plot(line_x_vals[vehicle_id], line_y_vals[vehicle_id], color=self._get_colour(plt_colour), linewidth=0.5, zorder=1)
                    
                    if lane_idx != None and line_x_vals[vehicle_id][-1] < x_lim[1]:
                        exit_x.append(line_x_vals[vehicle_id][-1])
                        exit_y.append(line_y_vals[vehicle_id][-1])

                del line_x_vals[vehicle_id]
                del line_y_vals[vehicle_id]

                plotted = True

        for vehicle_id in line_x_vals.keys():
            if len(line_x_vals[vehicle_id]) > 1:
                ax.plot(line_x_vals[vehicle_id], line_y_vals[vehicle_id], color=self._get_colour(plt_colour), linewidth=0.5, zorder=1)
                if lane_idx != None and line_x_vals[vehicle_id][-1] < x_lim[1]:
                    exit_x.append(line_x_vals[vehicle_id][-1])
                    exit_y.append(line_y_vals[vehicle_id][-1])
            plotted = True

        if not plotted:
            desc = "No vehicles found on lane '{0}'. ".format(lane_idx)
            raise_error(ValueError, desc)
        
        if lane_idx != None:
            ax.scatter(entry_x, entry_y, color='lightgrey', marker='+', s=30, zorder=2)
            ax.scatter(exit_x, exit_y, color='lightgrey', marker='x', s=20, zorder=2)

        x_label, y_label = self._default_labels["sim_time"], self._default_labels[new_units]

        if dist_labels != None:
            validate_list_types(dist_labels, (list, tuple), param_name='dist_labels')
            for dist_label in dist_labels:
                if not isinstance(dist_label, (list, tuple)) or len(dist_label) != 2:
                    desc = f"Invalid dist_label '{dist_label}' (must be length 2: [str, (int, float)])."
                    raise_error(TypeError, desc)
                elif not isinstance(dist_label[0], str) or not isinstance(dist_label[1], (int, float)):
                    desc = f"Invalid dist_label '{dist_label}' (must be type [str, (int, float)])."
                    raise_error(TypeError, desc)

                label, dist = dist_label
                dist = convert_units(dist, orig_units, new_units)
                
                ax.plot(x_lim, [dist, dist], linestyle='--', color='black', linewidth=1.5, zorder=10)
                padding = (x_lim[1] - x_lim[0]) * 0.02
                ax.text(padding + x_lim[0], dist, label, fontsize=6,
                        zorder=20, horizontalalignment='left', verticalalignment='center', color='red',
                        bbox=dict(facecolor='white', alpha=0.9, linewidth=0))
                
        ax.set_xlim(x_lim)
        ax.set_ylim(y_lim)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        if fig_title == None:
            fig_title = "Vehicle Trajectories"
            if lane_idx != None: fig_title += " (Lane {0})".format(lane_idx+1)
        fig_title = self.sim_label + fig_title
        ax.set_title(fig_title, pad=20)

        self._plot_event(ax, show_events)

        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_fundamental_diagram(self, edge_ids: list|tuple|str|None=None, x_axis: str="density", y_axis: str="flow", x_percentile: int=100, y_percentile: int=100, lr_degree: int|None=None, aggregation_steps: int=0, separate_edges: bool=False, point_size: int=3, time_range: list|tuple|None=None, plt_colour: str|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot a fundamental diagram from tracked egde data.
        
        Args:
            `edge_ids` (list, tuple, str, optional): Single tracked edge ID or list of IDs
            `x_axis` (str): x-axis variable ('_s_'|'_f_'|'_d_'|'_speed_'|'_flow_'|'_density_')
            `y_axis` (str): y-axis variable ('_s_'|'_f_'|'_d_'|'_speed_'|'_flow_'|'_density_')
            `x_percentile` (int): x-axis value plotting percentile [1-100]
            `y_percentile` (int): y-axis value plotting percentile [1-100]
            `lr_degree` (int, optional): Degree of linear regression (LR) model (if given, regression line and R^2 value plotted)
            `aggregation_steps` (int): If given, values are aggregated using this interval
            `separate_edges` (bool): If True, individual edges are plotted with separate colours
            `point_size` (int): Scatter graph point size
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `plt_colour` (str, optional): Line colour for plot (defaults to TUD 'cyaan')
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        if "edges" not in self.sim_data["data"] or ("edges" in self.sim_data["data"] and len(self.sim_data["data"]) == 0):
            desc = "No tracked edges found."
            raise_error(KeyError, desc)

        if edge_ids != None:
            if isinstance(edge_ids, str): edge_ids = [edge_ids]
            elif not isinstance(edge_ids, (list, tuple)):
                desc = "Invalid edge_ids '{0}' type (must be '[str|list|tuple]' not '{1}')".format(edge_ids, type(edge_ids).__name__)
                raise_error(TypeError, desc)
        else:
            edge_ids = list(self.sim_data["data"]["edges"].keys())

        axes = {"S": "speed", "F": "flow", "D": "density"}
        if not isinstance(x_axis, str) or x_axis.upper() not in ["S", "F", "D", "SPEED", "FLOW", "DENSITY"]:
            desc = "Invalid x_axis '{0}' (must be ['s'|'f'|'d'|'speed'|'flow'|'density']).".format(x_axis)
            error = TypeError if not isinstance(x_axis, str) else ValueError
            raise_error(error, desc)
        elif x_axis in ['s', 'f', 'd']: x_axis = axes[x_axis.upper()]

        if not isinstance(y_axis, str) or y_axis.upper() not in ["S", "F", "D", "SPEED", "FLOW", "DENSITY"]:
            desc = "Invalid y_axis '{0}' (must be ['s'|'f'|'d'|'speed'|'flow'|'density']).".format(y_axis)
            error = TypeError if not isinstance(y_axis, str) else ValueError
            raise_error(error, desc)
        elif y_axis in ['s', 'f', 'd']: y_axis = axes[y_axis.upper()]

        fig, ax = plt.subplots(1, 1)
        start, step_len = self.sim_data["start"], self.sim_data["step_len"]
        all_edge_data = self.sim_data["data"]["edges"]

        data_labels = {"speed": "speeds", "flow": "flows", "density": "densities"}

        # Calculate limits for plotting percentiles (across data for all edges)
        if x_percentile < 100 or y_percentile < 100:
            all_x_vals, all_y_vals = [], []
            for edge_id in edge_ids:
                all_x_vals += all_edge_data[edge_id][data_labels[x_axis]]
                all_y_vals += all_edge_data[edge_id][data_labels[y_axis]]

            x_percentile, y_percentile = max(min(x_percentile, 100), 1), max(min(y_percentile, 100), 1)
            outlier_x_lim = np.percentile(all_x_vals, x_percentile)
            outlier_y_lim = np.percentile(all_y_vals, y_percentile)

        plotted = False
        max_x, max_y = -math.inf, -math.inf
        x_points, y_points = [], []
        all_x, all_y = [], []
        for idx, edge_id in enumerate(edge_ids):

            x_vals = all_edge_data[edge_id][data_labels[x_axis]]
            y_vals = all_edge_data[edge_id][data_labels[y_axis]]
            
            # Get time_steps if plotting within time range
            if time_range != None:
                time_steps = get_time_steps(x_vals, self.time_unit, step_len, start)
                _, x_vals = limit_vals_by_range(time_steps, x_vals, time_range)
                _, y_vals = limit_vals_by_range(time_steps, y_vals, time_range)

            # Change -1 vals (no vehicles) to 0 for calculating averages
            x_vals = [max(val, 0) for val in x_vals]
            y_vals = [max(val, 0) for val in y_vals]

            if len(x_vals) == 0 or len(y_vals) == 0: continue

            # Calculate moving average, using aggregation_steps as the average
            if aggregation_steps > 0:
                avg_x_vals, avg_y_vals = [], []
                for idx in range(len(x_vals)):
                    per_x_vals = x_vals[max(idx+1-aggregation_steps, 0):idx+1]
                    per_y_vals = y_vals[max(idx+1-aggregation_steps, 0):idx+1]

                    avg_x_vals.append(sum(per_x_vals) / len(per_x_vals))
                    avg_y_vals.append(sum(per_y_vals) / len(per_y_vals))

                x_vals, y_vals = avg_x_vals, avg_y_vals

            # Filter for outlying points if plotting percentiles
            if x_percentile < 100 or y_percentile < 100:

                lim_x_vals, lim_y_vals = [], []
                for x_val, y_val in zip(x_vals, y_vals):
                    if x_val <= outlier_x_lim and y_val <= outlier_y_lim:
                        lim_x_vals.append(x_val)
                        lim_y_vals.append(y_val)
                x_vals, y_vals = lim_x_vals, lim_y_vals

            if len(x_vals) == 0 or len(y_vals) == 0: continue
            max_x, max_y = max(max(x_vals), max_x), max(max(y_vals), max_y)

            x_points += x_vals
            y_points += y_vals

            if separate_edges:
                colour, _ = self._get_colour("WHEEL", idx==0)
                ax.scatter(x_points, y_points, s=point_size, color=colour, label=edge_id if separate_edges else None, zorder=2)
                all_x += x_points
                all_y += y_points
                x_points, y_points, plotted = [], [], True

        if not separate_edges and len(x_vals) > 0 and len(y_vals) > 0:
            ax.scatter(x_points, y_points, s=point_size, color=self._get_colour(plt_colour), zorder=2)
            all_x, all_y = x_points, y_points

        elif not plotted:
            desc = "No data to plot (no vehicles found on tracked edges during time period)."
            raise_error(KeyError, desc)

        if lr_degree != None:
            model = np.poly1d(np.polyfit(all_x, all_y, lr_degree))
            x_vals = np.linspace(min(all_x), max(all_x), 100)
            y_vals = model(x_vals)
            ax.plot(x_vals, y_vals, color='green')

            r_2 = round(_get_r2(all_x, all_y, lr_degree), 5)
            stats = f"$R^2 = {r_2}$"

            if lr_degree > 1:
                lr_max_y, lr_max_x = max(y_vals), x_vals[list(y_vals).index(max(y_vals))]
                ax.plot([0, lr_max_x], [lr_max_y, lr_max_y], color='darkgrey', linestyle='--', zorder=12)
                ax.plot([lr_max_x, lr_max_x], [0, lr_max_y], color='darkgrey', linestyle='--', zorder=12)
                ax.scatter([lr_max_x], [lr_max_y], marker='o', color='darkgrey', s=point_size*5, zorder=12)

                stats += f"\n$Max: ({round(lr_max_x, 1)}, {round(lr_max_y, 1)})$"

            buffer = 0.02
            ax.text(get_axis_lim(max_x) * (1 - buffer), get_axis_lim(max_y) * (1 - buffer), stats,
                    zorder=20, horizontalalignment='right', verticalalignment='top', color='black',
                    bbox=dict(facecolor='white', alpha=0.6, linewidth=0))

        dist = "mi" if self.units == "IMPERIAL" else "km"
        sp = "mph" if self.units == "IMPERIAL" else "kmph"

        axis_labels = {"DENSITY": "Density (veh/{0})".format(dist),
                        "SPEED": "Average Speed ({0})".format(sp),
                        "FLOW": "Flow (veh/hr)"}
        
        if fig_title == None:
            fig_title = "{0}-{1} Fundamental Diagram".format(x_axis.title(), y_axis.title())
            fig_title = self.sim_label + fig_title
        ax.set_title(fig_title, pad=20)
        ax.set_xlabel(axis_labels[x_axis.upper()])
        ax.set_ylabel(axis_labels[y_axis.upper()])
        ax.set_xlim(0, get_axis_lim(max_x))
        ax.set_ylim(0, get_axis_lim(max_y))
        self._add_grid(ax)
        if separate_edges and len(edge_ids) > 1: ax.legend(shadow=True)

        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_throughput(self, od_pair: list|tuple|None=None, vehicle_types: list|tuple|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, plt_colour: str|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plots vehicle throughput, ie. the rate of completed trips.
        
        Args:
            `od_pair` (list, tuple, optional): (n x 2) list containing OD pairs. If not given, all OD pairs are plotted
            `vehicle_types` (list, tuple, optional): List of vehicle types to include (defaults to all)
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `plt_colour` (str, optional): Line colour for plot (defaults to TUD 'cyaan')
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        fig, ax = plt.subplots(1, 1)
        x_vals, y_vals = _get_throughput_x_y(self.sim_data, self.time_unit, od_pair, vehicle_types, time_range)
        ax.plot(x_vals, y_vals, color=self._get_colour(plt_colour), linewidth=1)
        
        if fig_title == None:
            if od_pair == None: fig_title = "{0}Network Throughput".format(self.sim_label)
            else: fig_title = "{0}'{1}' → '{2}' Trip Throughput".format(self.sim_label, od_pair[0], od_pair[1])

        ax.set_title(fig_title, pad=20)
        ax.set_ylabel("Throughput (veh/hr)")
        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_xlim([x_vals[0], x_vals[-1]])
        ax.set_ylim([0, get_axis_lim(y_vals)])

        fig.tight_layout()

        self._add_grid(ax)
        self._plot_event(ax, show_events)
        self._display_figure(save_fig)

    def plot_trip_time_histogram(self, od_pair: list|tuple|None=None, n_bins: int|None=None, cumulative_hist: bool=False, vehicle_types: list|tuple|None=None, time_range: list|tuple|None=None, plt_colour: str|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plots a histogram for (completed) trip times, either network-wide or for a specific OD pair.
        
        Args:
            `od_pair` (list, tuple, optional): (n x 2) list containing OD pairs. If not given, all OD pairs are plotted
            `n_bins` (int, optional): Number of bins in the histogram, calculated using the Freedman-Diaconis rule if not given
            `cumulative_hist` (bool): Denotes whether to plot histogram values cumulatively
            `vehicle_types` (list, tuple, optional): List of vehicle types to include (defaults to all)
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `plt_colour` (str, optional): Line colour for plot (defaults to TUD 'cyaan')
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """
        
        if self.simulation != None:
            self.sim_data = self.simulation.__dict__()
            self.units = self.simulation.units.name

        if time_range == None: time_range = [-math.inf, math.inf]
        else: time_range = convert_units(time_range, self.time_unit, "steps", self.sim_data["step_len"])

        com_trip_data = self.sim_data["data"]["trips"]["completed"]

        trip_times = []
        for trip in com_trip_data.values():
            origin, destination, veh_type = trip["origin"], trip["destination"], trip["vehicle_type"]
            departure, arrival = trip["departure"], trip["arrival"]

            if vehicle_types != None and veh_type not in vehicle_types: continue
            elif od_pair != None and origin != od_pair[0]: continue
            elif od_pair != None and destination != od_pair[1]: continue
            elif departure <= time_range[0] or arrival >= time_range[1]: continue
            else:
                trip_times.append(convert_units(trip["arrival"] - trip["departure"], "steps", self.time_unit, self.sim_data["step_len"]))

        if len(trip_times) == 0:
            desc = "No trip data to plot."
            raise_error(ValueError, desc)

        fig, ax = plt.subplots(1, 1)

        if n_bins == None:
            q1 = np.quantile(trip_times, 0.25)
            q3 = np.quantile(trip_times, 0.75)
            bin_width = (2 * (q3 - q1)) / (len(trip_times) ** (1 / 3))
            n_bins = math.ceil((max(trip_times) - min(trip_times)) / bin_width)

        ax.hist(trip_times, n_bins, color=self._get_colour(plt_colour), cumulative=cumulative_hist, zorder=2)
        
        if fig_title == None:
            if od_pair == None: fig_title = "{0}Trip Time Distribution".format(self.sim_label)
            else: fig_title = "{0}'{1}' → '{2}' Trip Time Distribution".format(self.sim_label, od_pair[0], od_pair[1])

        ax.set_title(fig_title, pad=20)
        ax.set_ylabel("Frequency")
        ax.set_xlabel(self._default_labels[self.time_unit])
        ax.set_ylim(0, get_axis_lim(ax.get_ylim()[1]))

        fig.tight_layout()

        self._add_grid(ax)
        self._display_figure(save_fig)

class MultiPlotter(_GenericPlotter):
    """ Visualisation class that plots TUD-SUMO data for multiple simulations. """

    def __init__(self, groups_title: str|None=None, scenario_label: str|None=None, units: str="metric", time_unit: str="seconds", sim_data_loc: str="", save_fig_loc: str="", save_fig_dpi: int=600, overwrite_figs: bool=True) -> None:
        """
        Args:
            `groups_title` (str, optional): Groups title (ie. 'Algorithm' label when comparing results of different algorithms)
            `scenario_label` (str, optional): Scenario label added to the beginning of all plot titles
            `units` (str): Simulation data units, must match all added simulations (must be ['_metric_'|'_imperial_'|'_uk_'])
            `time_unit` (str): Plotting time unit used for all plots (must be ['_steps_'|'_seconds_'|'_minutes_'|'_hours_'])
            `sim_data_loc` (str): Location of simulation data files (for all simulations)
            `save_fig_loc` (str): Figure filepath when saving (defaults to current file)
            `save_fig_dpi` (int): Figure dpi when saving (defaults to 600dpi)
            `overwrite_figs` (bool): Denotes whether to allow overwriting of saved figures with the same name
        """

        self.sim_datasets = {}
        self.sim_dataset_ids = []
        self.sim_groups = {}
        self.sim_group_ids = []
        self.sim_labels = {}
                
        if scenario_label != None: self.scenario_label = scenario_label + ": "
        else: self.scenario_label = ""

        self.sim_data_loc = sim_data_loc
        self.groups_title = groups_title
        
        super().__init__(units, time_unit, save_fig_loc, save_fig_dpi, overwrite_figs)

    def __str__(self): return "<{0}>".format(self.__name__)
    def __name__(self): return "MultiPlotter"

    def add_simulations(self, simulations: list|tuple, labels: list|tuple|None=None, groups: str|list|tuple|None=None, pbar: bool=True) -> None:
        """
        Add simulation dataset(s) to the plotter. `simulations` and `labels` must have the same length, with each
        label corresponding to a simulation. By default, all simulations will use their scenario name as a labels,
        or set a label in the list to `None` to use the scenario name.
        
        `Groups` can either be a single group ID, meaning all simulations belong to the same group, or a list of
        group IDs corresponding to each simulation. By default, all simulations are not assigned a group, or set
        a group ID in the list to `None` to only assign specific simulations a group.

        Note that large simulation data files may take some time to load. Errors may also occur when adding very
        large numbers of simulation data files.

        Args:
            `simulations` (list, tuple): List of sim_data filepaths
            `labels` (list, tuple, optional): List of simulation dataset labels
            `groups` (str, list, tuple, optional): List of group IDs or single ID
            `pbar` (bool): Denotes whether to print a progress bar when loading multiple files
        """

        start_step, end_step, step_length = None, None, None

        if not isinstance(simulations, (list, tuple)): simulations = [simulations]
        validate_list_types(simulations, str, param_name="sim_data filenames")

        if labels == None: labels = [None]*len(simulations)
        elif not isinstance(labels, (list, tuple)): labels = [labels]

        if len(labels) != len(simulations):
            desc = "Invalid sim_data labels (length '{0}' must match number of simulations '{1}').".format(len(labels), len(simulations))
            raise_error(ValueError, desc)
        validate_list_types(labels, (str, type(None)), param_name="sim_data labels")
        
        if groups == None: groups = [None]*len(simulations)
        elif isinstance(groups, str): groups = [groups]*len(simulations)
        elif not isinstance(groups, (list, tuple)): groups = [groups]

        if len(groups) != len(simulations):
            desc = "Invalid sim_data groups (length '{0}' must match number of simulations '{1}').".format(len(groups), len(simulations))
            raise_error(ValueError, desc)
        validate_list_types(groups, (str, type(None)), param_name="sim_data groups")

        itr = zip(simulations, labels, groups)
        if pbar and len(simulations) > 1:
            desc = "sim_data files" if not isinstance(groups, str) else f"'{groups}' datasets"
            itr = tqdm(itr, f"Loading {desc}", len(simulations), unit="file(s)", colour='CYAN')

        for simulation, sim_label, sim_group in itr:
            
            simulation = self.sim_data_loc + simulation
            sim_id = len(self.sim_datasets) + 1

            if simulation.endswith(".json"): r_class, r_mode = json, "r"
            elif simulation.endswith(".pkl"): r_class, r_mode = pkl, "rb"
            else:
                desc = "Invalid simulation file '{0}' (must be '.json' or '.pkl' file).".format(simulation)
                raise_error(ValueError, desc)

            if os.path.exists(simulation):
                with open(simulation, r_mode) as fp:
                    sim_data = r_class.load(fp)
                    sim_units = sim_data["units"]

                    if sim_units != self.units:
                        desc = "Invalid Simulation '{0}', units mismatch (must be MultiPlotter unit '{1}', not '{2}').".format(sim_label, sim_units, self.units)
                        raise_error(ValueError, desc)

                    if start_step == None:
                        start_step, end_step, step_length = sim_data["start"], sim_data["end"], sim_data["step_len"]
                        
                    else:
                        if start_step != sim_data["start"]:
                            desc = "Invalid Simulation '{0}' (start time '{1}' does not match previous '{2}').".format(sim_label, sim_data["start"], start_step)
                            raise_error(ValueError, desc)
                        if end_step != sim_data["end"]:
                            desc = "Invalid Simulation '{0}' (end time '{1}' does not match previous '{2}').".format(sim_label, sim_data["end"], end_step)
                            raise_error(ValueError, desc)
                        if step_length != sim_data["step_len"]:
                            desc = "Invalid Simulation '{0}' (step length '{1}' does not match previous '{2}').".format(sim_label, sim_data["step_len"], step_length)
                            raise_error(ValueError, desc)

                    if "all_vehicles" in sim_data["data"]: del sim_data["data"]["all_vehicles"]

                    self.sim_datasets[sim_id] = sim_data
                    self.sim_dataset_ids.append(sim_id)

                    if sim_group != None:
                        self.sim_groups[sim_id] = sim_group
                        if sim_group not in self.sim_group_ids: self.sim_group_ids.append(sim_group)

                    if sim_label != None: self.sim_labels[sim_id] = sim_label
                    else: self.sim_labels[sim_id] = sim_data["scenario_name"]

            else:
                desc = "Simulation file '{0}' not found.".format(simulation)
                raise_error(FileNotFoundError, desc)

    def plot_vehicle_data(self, data_key: str, plot_cumulative: bool=False, plot_groups: list|tuple|None=None, plot_range: bool=True, aggregation_steps: int|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot network-wide vehicle data for each simulation.
        
        Args:
            `data_key` (str): Data key to plot, either '_no_vehicles_', '_no_waiting_', '_tts_', '_twt_', '_avg_wt_', '_delay_', '_avg_delay_' or '_to_depart_'
            `plot_cumulative` (bool): Bool denoting whether to plot cumulative values
            `plot_groups` (list, tuple, optional): List of dataset groups to plot (defaults to all)
            `plot_range` (bool): Denotes whether to plot minimum-maximum value range for groups as a shaded region
            `aggregation_steps` (int, optional): If given, values are aggregated using this interval
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if data_key not in ["no_vehicles", "no_waiting", "tts", "twt", "avg_wt", "delay", "avg_delay", "to_depart"]:
            desc = "Unrecognised data key '{0}' (must be ['no_vehicles'|'no_waiting'|'tts'|'twt'|'avg_wt'|'delay'|'avg_delay'|'to_depart']).".format(data_key)
            raise_error(KeyError, desc)

        fig, ax = plt.subplots(1, 1)

        all_group_data, plotted = {group_id: [] for group_id in self.sim_group_ids}, 0
        x_lim, max_y_val = [math.inf, -math.inf], -math.inf
        for sim_id in self.sim_dataset_ids:

            if isinstance(plot_groups, (list, tuple)) and self.sim_groups[sim_id] not in plot_groups: continue
            sim_data = self.sim_datasets[sim_id]

            start, step = sim_data["start"], sim_data["step_len"]
            if data_key == "avg_wt": key, avg = "no_waiting", True
            elif data_key == "avg_delay": key, avg = "delay", True
            else: key, avg = data_key, False

            y_vals = sim_data["data"]["vehicles"][key]
            if avg: y_vals = [y_val / n_vehicles for y_val, n_vehicles in zip(y_vals, sim_data["data"]["vehicles"]["no_vehicles"])]

            if plot_cumulative: y_vals = get_cumulative_arr(y_vals)
            x_vals = get_time_steps(y_vals, self.time_unit, step, start)
            x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)

            x_lim = [min(min(x_vals), x_lim[0]), max(max(x_vals), x_lim[1])]

            if aggregation_steps != None:
                y_vals, x_vals = get_aggregated_data(y_vals, x_vals, aggregation_steps)

            if sim_id not in self.sim_groups:
                colour, line_style = self._get_colour("WHEEL", plotted==0)
                ax.plot(x_vals, y_vals, label=self.sim_labels[sim_id], color=colour, linestyle=line_style, linewidth=1)
                plotted += 1
            else:
                all_group_data[self.sim_groups[sim_id]].append((x_vals, y_vals))

        plotted, max_y_val = self._plot_group_data(ax, all_group_data, plotted, max_y_val, plot_range)

        if fig_title == None:
            fig_title = "Network-wide "+self._default_titles[data_key]
            if plot_cumulative: fig_title = "Cumulative "+fig_title
            fig_title = self.scenario_label + fig_title
        ax.set_title(fig_title, pad=20)

        if plotted > 1: ax.legend(title=self.groups_title, shadow=True)
        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel(self._default_labels[data_key])
        ax.set_xlim(x_lim)
        ax.set_ylim([0, get_axis_lim(max_y_val)])
        self._add_grid(ax, None)

        self._plot_event(ax, show_events)
        
        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_detector_data(self, detector_id: str, data_key: str, plot_cumulative: bool=False, plot_groups: list|tuple|None=None, plot_range: bool=True, aggregation_steps: int|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot detector data from each simulation.
        
        Args:
            `detector_id` (str): Detector ID
            `data_key` (str): Data key to plot, either '_speeds_', '_vehicle_counts_' or '_occupancies_'
            `plot_cumulative` (bool): Bool denoting whether to plot cumulative values
            `plot_groups` (list, tuple, optional): List of dataset groups to plot (defaults to all)
            `plot_range` (bool): Denotes whether to plot minimum-maximum value range for groups as a shaded region
            `aggregation_steps` (int, optional): If given, values are aggregated using this interval
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """
  
        if data_key not in ["speeds", "vehicle_counts", "occupancies"]:
            desc = "Unrecognised data key '{0}' (must be [speeds|vehicle_counts|occupancies]).".format(data_key)
            raise_error(KeyError, desc)
        
        fig, ax = plt.subplots(1, 1)

        all_group_data, plotted = {group_id: [] for group_id in self.sim_group_ids}, 0
        x_lim, max_y_val = [math.inf, -math.inf], -math.inf
        for sim_id in self.sim_dataset_ids:

            if isinstance(plot_groups, (list, tuple)) and self.sim_groups[sim_id] not in plot_groups: continue
            sim_data = self.sim_datasets[sim_id]

            if detector_id not in sim_data["data"]["detectors"].keys():
                desc = f"(Simulation '{self.sim_labels[sim_id]}') Detector ID '{detector_id}' not found."
                raise_error(KeyError, desc)
            elif data_key == "occupancies" and sim_data["data"]["detectors"][detector_id]["type"] == "multientryexit":
                desc = f"(Simulation '{self.sim_labels[sim_id]}') Multi-Entry-Exit Detectors ('{detector_id}') do not measure '{data_key}'."
                raise_error(ValueError, desc)

            start, step = sim_data["start"], sim_data["step_len"]
            y_vals = sim_data["data"]["detectors"][detector_id][data_key]
            if data_key == "occupancies": y_vals = [val * 100 for val in y_vals]
            if plot_cumulative: y_vals = get_cumulative_arr(y_vals)
            x_vals = get_time_steps(y_vals, self.time_unit, step, start)
            x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)

            x_lim = [min(min(x_vals), x_lim[0]), max(max(x_vals), x_lim[1])]
            
            if aggregation_steps != None:
                y_vals, x_vals = get_aggregated_data(y_vals, x_vals, aggregation_steps)

            if sim_id not in self.sim_groups:
                colour, line_style = self._get_colour("WHEEL", plotted==0)
                ax.plot(x_vals, y_vals, label=self.sim_labels[sim_id], color=colour, linestyle=line_style, linewidth=1)
                plotted += 1
            else:
                all_group_data[self.sim_groups[sim_id]].append((x_vals, y_vals))

        plotted, max_y_val = self._plot_group_data(ax, all_group_data, plotted, max_y_val, plot_range)

        if fig_title == None:
            fig_title = "{0} (Detector '{1}')".format(self._default_titles[data_key], detector_id)
            if plot_cumulative: fig_title = "Cumulative "+fig_title
            fig_title = self.scenario_label + fig_title
        ax.set_title(fig_title, pad=20)

        if plotted > 1: ax.legend(title=self.groups_title, shadow=True)
        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel(self._default_labels[data_key])
        ax.set_xlim(x_lim)
        if data_key == "occupancies": ax.set_ylim([0, 100])
        else: ax.set_ylim([0, get_axis_lim(max_y_val)])
        self._add_grid(ax, None)

        self._plot_event(ax, show_events)
        
        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_edge_data(self, edge_id: str, data_key: str, plot_cumulative: bool=False, plot_groups: list|tuple|None=None, plot_range: bool=True, aggregation_steps: int|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot tracked edge data from each simulation.
        
        Args:
            `edge_id` (str): Tracked edge ID
            `data_key` (str): Data key to plot, either '_flows_', '_speeds_', '_densities_', '_occupancies_', '_vehicle_counts_'
            `plot_cumulative` (bool): Bool denoting whether to plot cumulative values
            `plot_groups` (list, tuple, optional): List of dataset groups to plot (defaults to all)
            `plot_range` (bool): Denotes whether to plot minimum-maximum value range for groups as a shaded region
            `aggregation_steps` (int, optional): If given, values are aggregated using this interval
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """
  
        if data_key not in ["flows", "speeds", "densities", "occupancies", "vehicle_counts"]:
            desc = "Unrecognised data key '{0}' (must be [flows|speeds|densities|occupancies|vehicle_counts]).".format(data_key)
            raise_error(KeyError, desc)
        
        fig, ax = plt.subplots(1, 1)

        all_group_data, plotted = {group_id: [] for group_id in self.sim_group_ids}, 0
        x_lim, max_y_val = [math.inf, -math.inf], -math.inf
        for sim_id in self.sim_dataset_ids:

            if isinstance(plot_groups, (list, tuple)) and self.sim_groups[sim_id] not in plot_groups: continue
            sim_data = self.sim_datasets[sim_id]

            if "edges" not in sim_data["data"].keys():
                desc = f"(Simulation '{self.sim_labels[sim_id]}') No TrackedEdge data found."
                raise_error(KeyError, desc)
            elif edge_id not in sim_data["data"]["edges"].keys():
                desc = f"(Simulation '{self.sim_labels[sim_id]}') Edge ID '{edge_id}' not found."
                raise_error(KeyError, desc)

            start, step = sim_data["start"], sim_data["step_len"]
            y_vals = sim_data["data"]["edges"][edge_id][data_key if data_key != "vehicle_counts" else "step_vehicles"]
            if data_key == "occupancies": y_vals = [val * 100 for val in y_vals]
            elif data_key == "vehicle_counts": y_vals = [len(step_data) for step_data in y_vals]
            if plot_cumulative: y_vals = get_cumulative_arr(y_vals)
            x_vals = get_time_steps(y_vals, self.time_unit, step, start)
            x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)

            x_lim = [min(min(x_vals), x_lim[0]), max(max(x_vals), x_lim[1])]
            
            if aggregation_steps != None:
                y_vals, x_vals = get_aggregated_data(y_vals, x_vals, aggregation_steps)

            if sim_id not in self.sim_groups:
                colour, line_style = self._get_colour("WHEEL", plotted==0)
                ax.plot(x_vals, y_vals, label=self.sim_labels[sim_id], color=colour, linestyle=line_style, linewidth=1)
                plotted += 1
            else:
                all_group_data[self.sim_groups[sim_id]].append((x_vals, y_vals))

        plotted, max_y_val = self._plot_group_data(ax, all_group_data, plotted, max_y_val, plot_range)

        if fig_title == None:
            fig_title = "'{0}' {1}{2}".format(edge_id, "Cumulative " if plot_cumulative else "", self._default_titles[data_key])
            fig_title = self.scenario_label + fig_title
        ax.set_title(fig_title, pad=20)

        if plotted > 1: ax.legend(title=self.groups_title, shadow=True)
        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel(self._default_labels[data_key])
        ax.set_xlim(x_lim)
        if data_key == "occupancies": ax.set_ylim([0, 100])
        else: ax.set_ylim([0, get_axis_lim(max_y_val)])
        self._add_grid(ax, None)

        self._plot_event(ax, show_events)
        
        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_throughput(self, od_pair: list|tuple|None=None, vehicle_types: list|tuple|None=None, plot_groups: list|tuple|None=None, plot_range: bool=True, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot vehicle throughput, ie. the rate of completed trips, for each simulation.
        
        Args:
            `od_pair` (list, tuple, optional): (n x 2) list containing OD pairs. If not given, all OD pairs are plotted
            `vehicle_types` (list, tuple, optional): List of vehicle types to include (defaults to all)
            `plot_groups` (list, tuple, optional): List of dataset groups to plot (defaults to all)
            `plot_range` (bool): Denotes whether to plot minimum-maximum value range for groups as a shaded region
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """
        
        fig, ax = plt.subplots(1, 1)

        all_group_data, plotted = {group_id: [] for group_id in self.sim_group_ids}, 0
        x_lim, max_y_val = [math.inf, -math.inf], -math.inf
        for sim_id in self.sim_dataset_ids:

            if isinstance(plot_groups, (list, tuple)) and self.sim_groups[sim_id] not in plot_groups: continue
            sim_data = self.sim_datasets[sim_id]

            x_vals, y_vals = _get_throughput_x_y(sim_data, self.time_unit, od_pair, vehicle_types, time_range)

            x_lim = [min(min(x_vals), x_lim[0]), max(max(x_vals), x_lim[1])]

            if sim_id not in self.sim_groups:
                colour, line_style = self._get_colour("WHEEL", plotted==0)
                ax.plot(x_vals, y_vals, label=self.sim_labels[sim_id], color=colour, linestyle=line_style, linewidth=1)
                plotted += 1
            else:
                all_group_data[self.sim_groups[sim_id]].append((x_vals, y_vals))

        plotted, max_y_val = self._plot_group_data(ax, all_group_data, plotted, max_y_val, plot_range)

        if fig_title == None:
            if od_pair == None: fig_title = "{0}Network Throughput".format(self.scenario_label)
            else: fig_title = "{0}'{1}' → '{2}' Trip Throughput".format(self.scenario_label, od_pair[0], od_pair[1])
        ax.set_title(fig_title, pad=20)

        if plotted > 1: ax.legend(title=self.groups_title, shadow=True)
        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel(self._default_labels["throughput"])
        ax.set_xlim(x_lim)
        ax.set_ylim([0, get_axis_lim(max_y_val)])
        self._add_grid(ax, None)

        self._plot_event(ax, show_events)
        
        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_rm_rate(self, rm_id: str, plot_groups: list|tuple|None=None, plot_range: bool=True, aggregation_steps: int|None=None, time_range: list|tuple|None=None, show_events: str|list|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot the average metering rate for a ramp meter in each simulation.
        
        Args:
            `rm_id` (str): Ramp meter junction ID
            `plot_groups` (list, tuple, optional): List of dataset groups to plot (defaults to all)
            `plot_range` (bool): Denotes whether to plot minimum-maximum value range for groups as a shaded region
            `aggregation_steps` (int, optional): If given, values are aggregated using this interval
            `time_range` (list, tuple, optional): Plotting time range (in plotter class units)
            `show_events` (str, list, optional): Event ID, list of IDs, '_all_', '_scheduled_', '_active_', '_completed_' or `None`
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        fig, ax = plt.subplots(1, 1)

        all_group_data, plotted = {group_id: [] for group_id in self.sim_group_ids}, 0
        x_lim, max_y_val = [math.inf, -math.inf], -math.inf
        min_rate, max_rate = math.inf, -math.inf

        for sim_id in self.sim_dataset_ids:

            if isinstance(plot_groups, (list, tuple)) and self.sim_groups[sim_id] not in plot_groups: continue
            sim_data = self.sim_datasets[sim_id]

            if "junctions" in sim_data["data"] and rm_id in sim_data["data"]["junctions"]:
                if "meter" in sim_data["data"]["junctions"][rm_id].keys():
                    rm_data, step_length = sim_data["data"]["junctions"][rm_id], sim_data["step_len"]
                    init_time, end_time, rm_data = rm_data["init_time"], rm_data["curr_time"], rm_data["meter"]
                    rates, times, min_r, max_r = np.array(rm_data["metering_rates"]), np.array(rm_data["rate_times"]), rm_data["min_rate"], rm_data["max_rate"]
                    
                    min_rate, max_rate = min(min_r, min_rate), max(max_r, max_rate)

                    curr_time, x_vals, y_vals = init_time, [], []
                    while curr_time <= end_time:
                        x_vals.append(curr_time)

                        idxs = np.where(times <= curr_time)[0]
                        if len(idxs) > 0: y_vals.append(rates[idxs[-1]])
                        else: y_vals.append(max_r)
                        
                        curr_time += step_length

                    x_vals = get_time_steps(y_vals, self.time_unit, step_length, init_time)
                    x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)

                    if aggregation_steps != None:
                        y_vals, x_vals = get_aggregated_data(y_vals, x_vals, aggregation_steps)
                    
                    if sim_id not in self.sim_groups:
                        colour, line_style = self._get_colour("WHEEL", plotted==0)
                        ax.plot(x_vals, y_vals, label=self.sim_labels[sim_id], color=colour, linestyle=line_style, linewidth=1)
                        plotted += 1
                    else:
                        all_group_data[self.sim_groups[sim_id]].append((x_vals, y_vals))

                    x_lim = [min(min(x_vals), x_lim[0]), max(max(x_vals), x_lim[1])]

                else:
                    desc = f"(Simulation '{self.sim_labels[sim_id]}') Junction '{rm_id}' is not tracked as a meter."
                    raise_error(ValueError, desc)
            else:
                desc = f"(Simulation '{self.sim_labels[sim_id]}') Junction '{rm_id}' not found in tracked junctions."
                raise_error(KeyError, desc)

        plotted, max_y_val = self._plot_group_data(ax, all_group_data, plotted, max_y_val, plot_range)

        ax.set_ylim([0, get_axis_lim(max_rate)])
        ax.axhline(max_rate, label="Min/Max Rate", color=self.ROOD, linestyle="--", zorder=2)
        ax.axhline(min_rate, color=self.ROOD, linestyle="--", zorder=2)

        fig_title = "{0}'{1}' Metering Rate".format(self.scenario_label, rm_id) if not isinstance(fig_title, str) else fig_title
        if fig_title != "": ax.set_title(fig_title, pad=20)
        ax.legend(shadow=True)

        if plotted > 1: ax.legend(title=self.groups_title, shadow=True)
        ax.set_xlabel(self._default_labels["sim_time"])
        ax.set_ylabel("Metering Rate (veh/hr)")
        ax.set_xlim(x_lim)
        ax.set_ylim([0, get_axis_lim(max_y_val)])
        self._add_grid(ax, None)

        self._plot_event(ax, show_events)

        fig.tight_layout()

        self._display_figure(save_fig)

    def plot_rm_queue_length(self, rm_id: str, plot_distribution: bool=True, plot_groups: list|tuple|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plots a boxplot showing the distribution of average queue lengths for each simulation group.

        Args:
            `rm_id` (str): Ramp meter ID
            `plot_distribution` (bool): Denotes whether to plot data distribution (boxplot) or not (barchart)
            `plot_groups` (list, tuple, optional): List of dataset groups to plot (defaults to all)
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """
                
        if len(self.sim_group_ids) == 0:
            desc = "No data to plot (no groups added)."
            raise_error(KeyError, desc)

        if plot_groups == None: plot_groups = self.sim_group_ids
        elif isinstance(plot_groups, (list, tuple)):
            for group_id in plot_groups:
                if group_id not in self.sim_group_ids:
                    desc = f"Group '{group_id}' not found."
                    raise_error(KeyError, desc)
        else:
            desc = f"Invalid plot_groups (must be 'list', not '{type(plot_groups).__name__}')."
            raise_error(TypeError, desc)

        all_data = {group_id: [] for group_id in plot_groups}
        
        for sim_id, group_id in self.sim_groups.items():
            if group_id not in plot_groups: continue
            sim_data = self.sim_datasets[sim_id]

            if "junctions" not in sim_data["data"] or rm_id not in sim_data["data"]["junctions"]:
                desc = f"(Simulation '{self.sim_labels[sim_id]}') Junction '{rm_id}' not found in tracked junctions."
                raise_error(KeyError, desc)
            elif "meter" not in sim_data["data"]["junctions"][rm_id]:
                desc = f"(Simulation '{self.sim_labels[sim_id]}') Junction '{rm_id}' is not tracked as a meter."
                raise_error(ValueError, desc)
            elif "queue_lengths" not in sim_data["data"]["junctions"][rm_id]["meter"]:
                desc = f"(Simulation '{self.sim_labels[sim_id]}') No queue length data found for ramp meter '{rm_id}'."
                raise_error(KeyError, desc)
        
            sim_val = sum(sim_data["data"]["junctions"][rm_id]["meter"]["queue_lengths"]) / len(sim_data["data"]["junctions"][rm_id]["meter"]["queue_lengths"])
            all_data[group_id].append(sim_val)

        fig, ax = plt.subplots(1, 1)
        plt_data = [all_data[group_id] for group_id in plot_groups]

        if plot_distribution:
            bplot = ax.boxplot(plt_data, patch_artist=True)

            ax.set_xticklabels(plot_groups, rotation=45)

            for idx, (patch, flier) in enumerate(zip(bplot["boxes"], bplot['fliers'])):
                colour, _ = self._get_colour("WHEEL", idx == 0)
                patch.set(linewidth=0, facecolor=colour)
                flier.set_markeredgecolor(colour)

            for median in bplot["medians"]:
                median.set(color='white', linewidth=1)
        
        else:
            plt_data, colours = [sum(vals) / len(vals) for vals in plt_data], []
            for idx in range(len(plt_data)):
                colour, _ = self._get_colour("WHEEL", idx == 0)
                colours.append(colour)
            
            ax.bar(plot_groups, plt_data, color=colours, zorder=3, label=plot_groups)
            ax.tick_params(axis='x', labelrotation=45)

        if self.groups_title != None: ax.set_xlabel(self.groups_title)
        ax.set_ylabel(self._default_labels["vehicle_counts"])
        fig_title = f"{self.scenario_label}'{rm_id}' Average Queue Length" if not isinstance(fig_title, str) else fig_title
        if fig_title != "": ax.set_title(fig_title, pad=20)
        
        self._add_grid(ax, None)

        fig.tight_layout()
        
        self._display_figure(save_fig)

    def plot_statistics(self, data_key: str, plot_distribution: bool=True, plot_groups: list|tuple|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plots a boxplot showing the distribution of network-wide vehicle statistics (TTS/TWT/cumulative delay) for each simulation group.

        Args:
            `data_key` (str): Either '_tts_', '_twt_' or '_delay_'
            `plot_distribution` (bool): Denotes whether to plot data distribution (boxplot) or not (barchart)
            `plot_groups` (list, tuple, optional): List of dataset groups to plot (defaults to all)
            `fig_title` (str, optional): If given, will overwrite default title
            `save_fig` (str, optional): Output image filename, will show image if not given
        """

        if data_key not in ["tts", "twt", "delay"]:
            desc = f"Unrecognised data key '{data_key}' (must be ['tts'|'twt'|'delay'])."
            raise_error(KeyError, desc)
        elif len(self.sim_group_ids) == 0:
            desc = "No data to plot (no groups added)."
            raise_error(KeyError, desc)

        if plot_groups == None: plot_groups = self.sim_group_ids
        elif isinstance(plot_groups, (list, tuple)):
            for group_id in plot_groups:
                if group_id not in self.sim_group_ids:
                    desc = f"Group '{group_id}' not found."
                    raise_error(KeyError, desc)
        else:
            desc = f"Invalid plot_groups (must be 'list', not '{type(plot_groups).__name__}')."
            raise_error(TypeError, desc)

        all_data = {group_id: [] for group_id in plot_groups}
        
        for sim_id, group_id in self.sim_groups.items():
            if group_id not in plot_groups: continue
            sim_data = self.sim_datasets[sim_id]
            sim_val = sum(sim_data["data"]["vehicles"][data_key])
            all_data[group_id].append(sim_val)

        fig, ax = plt.subplots(1, 1)
        plt_data = [all_data[group_id] for group_id in plot_groups]

        if plot_distribution:
            bplot = ax.boxplot(plt_data, patch_artist=True)

            ax.set_xticklabels(plot_groups, rotation=45)

            for idx, (patch, flier) in enumerate(zip(bplot["boxes"], bplot['fliers'])):
                colour, _ = self._get_colour("WHEEL", idx == 0)
                patch.set(linewidth=0, facecolor=colour)
                flier.set_markeredgecolor(colour)

            for median in bplot["medians"]:
                median.set(color='white', linewidth=1)
        
        else:
            plt_data, colours = [sum(vals) / len(vals) for vals in plt_data], []
            for idx in range(len(plt_data)):
                colour, _ = self._get_colour("WHEEL", idx == 0)
                colours.append(colour)
            
            ax.bar(plot_groups, plt_data, color=colours, zorder=3, label=plot_groups)
            ax.tick_params(axis='x', labelrotation=45)

        if self.groups_title != None: ax.set_xlabel(self.groups_title)
        ax.set_ylabel(self._default_labels[data_key])
        fig_title = f"{self.scenario_label}Network-wide {self._default_titles[data_key]}" if not isinstance(fig_title, str) else fig_title
        if fig_title != "": ax.set_title(fig_title, pad=20)
        
        self._add_grid(ax, None)

        fig.tight_layout()
        
        self._display_figure(save_fig)

    def _plot_group_data(self, ax, all_group_data, plotted = 0, max_y_val=math.inf, plot_range=True):

        for group_id, group_data in all_group_data.items():
            if len(group_data) > 0:
                min_y, max_y, avg_y = [], [], []
                x_vals = group_data[0][0]
                
                n_vals = len(x_vals)
                for idx in range(n_vals):
                    y_vals = [all_vals[1][idx] for all_vals in group_data]

                    min_y.append(min(y_vals))
                    max_y.append(max(y_vals))
                    avg_y.append(sum(y_vals) / len(y_vals))

                if plot_range: max_y_val = max(max(max_y), max_y_val)
                else: max_y_val = max(max(avg_y), max_y_val)

                colour, line_style = self._get_colour("WHEEL", plotted==0)
                ax.plot(x_vals, avg_y, label=group_id, color=colour, linestyle=line_style, linewidth=1)
                if plot_range: ax.fill_between(x_vals, min_y, max_y, color=colour, alpha=0.2)
                plotted += 1

        return plotted, max_y_val

def _get_throughput_x_y(sim_data: dict, time_unit: str, od_pair: list|tuple|None=None, vehicle_types: list|tuple|None=None, time_range: list|tuple|None=None):
    if time_range == None: t_range = [-math.inf, math.inf]
    else: t_range = convert_units(time_range, time_unit, "steps", sim_data["step_len"])

    com_trip_data = sim_data["data"]["trips"]["completed"]

    completion_times = []
    for trip in com_trip_data.values():
        
        if "arrival" not in trip: continue
        origin, destination, arrival, veh_type = trip["origin"], trip["destination"], trip["arrival"], trip["vehicle_type"]

        if vehicle_types != None and veh_type not in vehicle_types: continue
        elif od_pair != None and origin != od_pair[0]: continue
        elif od_pair != None and destination != od_pair[1]: continue
        elif arrival <= t_range[0] or arrival >= t_range[1]: continue
        else:
            completion_times.append(trip["arrival"])

    if len(completion_times) == 0:
        desc = "No trip data to plot."
        raise_error(ValueError, desc)

    start = int(max(t_range[0], sim_data["start"]))
    end = int(min(t_range[1], sim_data["end"]))

    x_vals = list(range(start, end + 1))
    y_vals = [0] * (len(x_vals))
    for val in completion_times:
        if val - start < len(y_vals):
            y_vals[val - start] += 1

    q1 = np.quantile(x_vals, 0.25)
    q3 = np.quantile(x_vals, 0.75)
    aggregation_steps = math.ceil((2 * (q3 - q1)) / (len(x_vals) ** (1 / 3)))
    y_vals, x_vals = get_aggregated_data(y_vals, x_vals, aggregation_steps, False)

    x_vals, y_vals = [start]+x_vals, [0]+y_vals
    x_vals = convert_units(x_vals, "steps", time_unit, sim_data["step_len"])
    agg_time = convert_units(aggregation_steps, "steps", "hours", sim_data["step_len"])
    y_vals = [val / agg_time for val in y_vals]

    return x_vals, y_vals

def _get_r2(x_vals, y_vals, degree):
    coeffs = np.polyfit(x_vals, y_vals, degree)
    p = np.poly1d(coeffs)
    y_avg = np.sum(y_vals)/len(y_vals)
    ssreg = np.sum((p(x_vals) - y_avg)**2)
    sstot = np.sum((y_vals - y_avg)**2)
    return 1 - (((1 - (ssreg / sstot)) * (len(y_vals)-1)) / (len(y_vals) - degree - 1))
