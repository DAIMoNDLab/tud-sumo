from random import randint, seed
import sys
from tqdm import tqdm

sys.path.insert(0, '..')
from simulation import *
from plot import Plotter

if __name__ == "__main__":

    # Initialise the simulation object.
    my_sim = Simulation(scenario_name="A20_ITCS", scenario_desc="Example traffic controllers, with 2 ramp meters, 1 VSL controller and 1 route guidance controller.")

    sim_seed = "1" if "-seed" not in sys.argv[:-1] else sys.argv[sys.argv.index("-seed")+1]
    seed(int(sim_seed))
    
    # Start the simulation, defining the sumo config files.
    my_sim.start("example_scenario/a20.sumocfg", get_individual_vehicle_data=False, gui="-gui" in sys.argv,
                 seed=sim_seed, units="metric") # Units can either be metric (km,kmph)/imperial (mi,mph)/UK (km,mph). All data collected is in these units.
    
    # Add demand from a '.csv' file.
    # my_sim.load_demand("example_scenario/demand.csv")

    # Add a tracked junction to the intersection with ID "utsc", which will track signal phases/times.
    my_sim.add_tracked_junctions({"utsc": {"flow_params": {"inflow_detectors": ["utsc_n_in_1", "utsc_n_in_2", "utsc_w_in", "utsc_e_in"],
                                                           "outflow_detectors": ["utsc_w_out", "utsc_e_out"],
                                                           "vehicle_types": ["cars", "lorries", "motorcycles", "vans"]}}})

    # Set traffic signal phases. The junc_phases dict can be used for multiple junctions.
    my_sim.set_phases({"utsc": {"phases": ["GGrr", "yyrr", "rrGG", "rryy"], "times": [27, 3, 17, 3]}})

    # Add a ramp meter to the junction with ID "crooswijk_meter". The junc_params dict can be used to
    # define meter specifc parameters (min/max rate, spillback tracking or queue detectors) and flow specific
    # parameters (inflow/outflow detectors used to calculate in/out flow).
    my_sim.add_tracked_junctions({"crooswijk_meter": {'meter_params': {'min_rate': 200, 'max_rate': 2000, 'queue_detector': "cw_ramp_queue"},
                                                    'flow_params': {'inflow_detectors': ["cw_ramp_inflow", "cw_rm_upstream"], 'outflow_detectors': ["cw_rm_downstream"]}},
                                  "a13_meter": {'meter_params': {'min_rate': 200, 'max_rate': 2000, 'queue_detector': "a13_ramp_queue"},
                                                'flow_params': {'inflow_detectors': ["a13_ramp_inflow", "a13_rm_upstream"], 'outflow_detectors': ["a13_rm_downstream"]}}})
    
    # Add Route Guidance (RG) & Variable Speed Limit (VSL) controllers. RG controllers need a detector or
    # edge that will act as the redirection point and a target edge/route ID to redirect drivers to. It is
    # also possible to define a diversion percent to randomly divert a certain percent of drivers, and a
    # highlight colour for the SUMO gui, which will highlight affected drivers. VSL controllers only need
    # to have a list of lanes/edges where they will operate.
    my_sim.add_controllers({"rerouter": {"type": "RG", "detector_ids": ["rerouter_2"], "new_destination": "urban_out_w", "diversion_pct": 1, "highlight": "00FF00"},
                            "vsl": {"type": "VSL", "geometry_ids": ["126729982", "126730069", "126730059"]}})

    # Add tracked edges. This will track some basic information, such as average speed etc, but can also
    # be used to create space-time diagrams as individual vehicle speeds and positions are tracked.
    my_sim.add_tracked_edges(["126730026", "1191885773", "1191885771", "126730171", "1191885772", "948542172", "70944365", "308977078", "1192621075"])

    # Add scheduled events from a JSON file (can be dictionary). Use the format as in example_incident.json
    my_sim.add_events("example_scenario/example_incident.json")

    # These individual functions above can be replaced as below, where the 'parameters.json' file contains
    # a dictionary of all necessary parameters (under 'edges', 'junctions', 'phases', 'controllers' and 'events')
    # my_sim.load_objects("parameters.json")
    
    n, sim_dur, new_veh_idx = 1 / my_sim.step_length, 500 / my_sim.step_length, 0
    pbar = tqdm(desc="Running sim (step 0, 0 vehs)", total=sim_dur)
    while my_sim.curr_step < sim_dur:

        # Set ramp metering rate.
        if my_sim.curr_step % 50 / my_sim.step_length == 0:
            my_sim.set_tl_metering_rate(rm_id="crooswijk_meter", metering_rate=randint(1200, 2000))
            my_sim.set_tl_metering_rate(rm_id="a13_meter", metering_rate=randint(1200, 2000))
        
        # Step through n steps.
        my_sim.step_through(n_steps=n, pbar=pbar)

        # Add new vehicles going from "urban_in_e" to "urban_out_w"
        if my_sim.curr_step % 50 / my_sim.step_length == 0:
            od_pair = ("urban_in_e", "urban_out_w")
            my_sim.add_vehicle(vehicle_id="lorry_"+str(new_veh_idx), vehicle_type="lorries", routing=od_pair, origin_lane="first")
            my_sim.add_vehicle(vehicle_id="car_"+str(new_veh_idx), vehicle_type="cars", routing=od_pair)
            new_veh_idx += 1

        if my_sim.curr_step == 100 / my_sim.step_length:
            my_sim.cause_incident(100, n_vehicles=2, edge_speed=5)

        if my_sim.curr_step == 250 / my_sim.step_length:
            # Activate controllers & update UTSC phases.
            my_sim.controllers["rerouter"].activate()
            my_sim.controllers["vsl"].set_speed_limit(60)

            my_sim.set_phases({"utsc": {"phases": ["GGrr", "yyrr", "rrGG", "rryy"], "times": [37, 3, 7, 3]}}, overwrite=False)

        if my_sim.curr_step == 400 / my_sim.step_length:
            my_sim.controllers["vsl"].set_speed_limit(40)

        # Deactivate controllers.
        if my_sim.curr_step == 450 / my_sim.step_length:
            my_sim.controllers["rerouter"].deactivate()
            my_sim.controllers["vsl"].deactivate()

    # End the simulation.
    my_sim.end()

    # Save the simulation data & print a summary, which is also saved.
    my_sim.save_data("example_data.json")
    my_sim.print_summary(save_file="example_summary.txt")
    