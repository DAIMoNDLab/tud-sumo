import sys, io
from .utils import *

# --- SIM DATA STRUCT ---

def print_sim_data_struct(sim_data) -> None:
    """
    Prints the structure of a sim_data dictionary, from a Simulation
    object, dict or filepath, with keys and data types for values. Lists/tuples
    are displayed at max 2D. '*' represents the maximum dimension value if
    the dimension size is inconsistent, and '+' denotes the array dimension is
    greater than 2.

    Args:
        `sim_data` (Simulation, dict, str): Either Simulation object, dictionary or sim_data filepath
    """
    
    #sim_data = validate_type(sim_data, (str, dict, Simulation), "sim_data")
    if isinstance(sim_data, dict):
        dictionary = sim_data
    elif isinstance(sim_data, str):
        if sim_data.endswith(".json"): r_class, r_mode = json, "r"
        elif sim_data.endswith(".pkl"): r_class, r_mode = pkl, "rb"
        else:
            caller = "{0}()".format(inspect.stack()[0][3])
            desc = "{0}: sim_data file '{1}' is invalid (must be '.json' or '.pkl').".format(caller, sim_data)
            raise ValueError(desc)

        if os.path.exists(sim_data):
            with open(sim_data, r_mode) as fp:
                dictionary = r_class.load(fp)
    else: dictionary = sim_data.__dict__()

    _print_dict({dictionary["scenario_name"]: dictionary})

def _print_dict(dictionary, indent=0, prev_indent="", prev_key=None):
    dict_keys = list(dictionary.keys())
    for key in dict_keys:
        val, is_last = dictionary[key], key == dict_keys[-1]
        curr_indent = _get_indent(indent, is_last, prev_indent)
        if isinstance(val, dict) and prev_key != "trips":
            print(curr_indent+key+":")
            _print_dict(val, indent+1, curr_indent, key)
        else:
            if isinstance(val, (list, tuple, dict)):
                type_str = type(val).__name__ + " " + _get_2d_shape(val)
            else:
                type_str = type(val).__name__
                
            print("{0}{1}: {2}".format(curr_indent, key, type_str))
        prev_indent = curr_indent
    
def _get_indent(indent_no, last, prev_indent, col_width=6):
    if indent_no == 0: return ""
    else:
        v_connector = "|{0}".format(" "*(col_width-1))
        end = "─- "

        connector = "└" if last else "├"
        indent_str = "  "+v_connector*(indent_no-1) + connector + end
        indent_arr, prev_indent = [*indent_str], [*prev_indent]

        for idx, _ in enumerate(indent_arr):
            if idx < len(prev_indent):
                prev_char = prev_indent[idx]
                if prev_char in [" ", "└"]: indent_arr[idx] = " "
                elif prev_char in ["|"]: indent_arr[idx] = "|"
                elif prev_char == "├" and indent_arr[idx] not in ["├", "└"]: indent_arr[idx] = "|"

        indent_str = "".join(indent_arr)
        return indent_str
    
def _get_2d_shape(array):

    x = len(array)
    arrs = []
    deeper = False
    for elem in array:
        if isinstance(elem, (list, tuple)):
            arrs.append(len(elem))
            deeper = deeper or True in [isinstance(elem2, (list, tuple)) for elem2 in elem]

    if len(arrs) > 0:
        if len(set(arrs)) == 1:
            return "({0}x{1})".format(x, arrs[0])
        else:
            return_str = "({0}x{1}*)".format(x, max(arrs))
            if deeper: return_str += "+"
            return return_str
    else: return "(1x{0})".format(x)


# --- SIMULATION SUMMARY ---

def print_summary(sim_data: dict | str, save_file: str | None=None, tab_width: int=58):
    """
    Prints a summary of a sim_data file or dictionary, listing
    simulation details, vehicle statistics, detectors, controllers,
    tracked edges/junctions and events.
    
    Args:
        `sim_data` (dict, str):  Simulation data dictionary or filepath
        `save_file` (str, optional): '_.txt_' filename, if given will be used to save summary
        `tab_width` (int): Table width
    """
    caller = "{0}()".format(inspect.stack()[0][3])
    if save_file != None:
        save_file = validate_type(save_file, str, "save_file")
        if not save_file.endswith(".txt"): save_file += ".txt"
    old_stdout = sys.stdout
    sys.stdout = buffer = io.StringIO()
    
    sim_data = validate_type(sim_data, (str, dict), "sim_data")
    if isinstance(sim_data, str):
        if sim_data.endswith(".json"): r_class, r_mode = json, "r"
        elif sim_data.endswith(".pkl"): r_class, r_mode = pkl, "rb"
        else:
            desc = f"{caller}: sim_data file '{sim_data}' is invalid (must be '.json' or '.pkl')."
            raise ValueError(desc)

        if os.path.exists(sim_data):
            with open(sim_data, r_mode) as fp:
                sim_data = r_class.load(fp)
        else:
            desc = "{0}: sim_data file '{1}' not found.".format(caller, sim_data)
            raise FileNotFoundError(desc)
    elif len(sim_data.keys()) == 0 or "data" not in sim_data.keys():
        desc = "{0}: Invalid sim_data (no data found)."
        raise ValueError(desc)
    
    name = sim_data["scenario_name"]
    if math.floor((tab_width-len(name))/2) != math.ceil((tab_width-len(name))/2):
        tab_width += 1
    
    primary_delineator = " *"+"="*(tab_width+2)+"*"
    secondary_delineator = " *"+"-"*(tab_width+2)+"*"
    tertiary_delineator = " * "+"-"*tab_width+" *"
    
    print(primary_delineator)
    _table_print("TUD-SUMO v{0}".format(sim_data["tuds_version"]), tab_width)

    print(primary_delineator)
    _table_print(sim_data["scenario_name"], tab_width)
    if "scenario_desc" in sim_data.keys():
        desc = sim_data["scenario_desc"]
        print(primary_delineator)
        if tab_width - len("Description: "+desc) > 0:
            _table_print("Description: "+desc, tab_width)
        else:
            _table_print("Description:", tab_width)
            desc_lines = _add_linebreaks(desc, tab_width)
            for line in desc_lines: _table_print(line, tab_width)
    print(primary_delineator)
    
    start_step, end_step = sim_data["start"], sim_data["end"]
    start_time, end_time = datetime.strptime(sim_data["sim_start"], datetime_format), datetime.strptime(sim_data["sim_end"], datetime_format)
    sim_duration, sim_duration_steps = end_time - start_time, end_step - start_step
    if start_time.date() == end_time.date():
        _table_print(f"Simulation Run: {start_time.strftime(date_format)}", tab_width)
        _table_print(f"{start_time.strftime(time_format)} - {end_time.strftime(time_format)} ({sim_duration})", tab_width)
    else:
        _table_print(f"Simulation Run: ({sim_duration})", tab_width)
        _table_print([start_time.strftime(date_format), end_time.strftime(date_format)], tab_width, centre_cols=True)
        _table_print([start_time.strftime(time_format), end_time.strftime(time_format)], tab_width, centre_cols=True)
    
    print(secondary_delineator)
    _table_print(["Number of Steps:", f"{sim_duration_steps} ({start_step}-{end_step})"], tab_width)
    _table_print(["Step Length:", f"{sim_data['step_len']}s"], tab_width)
    _table_print(["Avg. Step Duration:", f"{sim_duration.total_seconds() / sim_duration_steps}s"], tab_width)
    _table_print(["Units Type:", unit_desc[sim_data["units"]]], tab_width)
    _table_print(["Seed:", sim_data["seed"]], tab_width)
    
    print(primary_delineator)
    _table_print("Data", tab_width)
    print(primary_delineator)
    _table_print("Vehicle Data", tab_width)
    print(secondary_delineator)

    labels = {"no_vehicles": ["No. Vehicles", "Overall TTS"],
              "no_waiting": ["No. Waiting Vehicles", "Overall TWT"],
              "delay": ["Vehicle Delay", "Cumulative Delay"],
              "to_depart": ["Vehicles to Depart", None]}

    for key, label in labels.items():
        data = sim_data["data"]["vehicles"][key]
        unit = "s" if key == "delay" else ""
        _table_print(f"{label[0]}:", tab_width)
        _table_print(["Average:", f"{round(sum(data)/len(data), 2)}{unit}"], tab_width)
        _table_print(["Peak:", f"{round(max(data), 2)}{unit}"], tab_width)
        _table_print(["Final:", f"{round(data[-1], 2)}{unit}"], tab_width)
        if label[1] != None: _table_print([f"{label[1]}:", f"{round(sum(data), 2)}s"], tab_width)
        print(tertiary_delineator)

    _table_print(["Floating Car Data:", "Yes" if "fc_data" in sim_data["data"].keys() else "No"], tab_width)

    print(secondary_delineator)
    _table_print("Trip Data", tab_width)
    print(secondary_delineator)
    n_inc, n_com = len(sim_data["data"]["trips"]["incomplete"]), len(sim_data["data"]["trips"]["completed"])
    _table_print(["Incomplete Trips:", f"{n_inc} ({round(100 * n_inc / (n_inc + n_com), 2)}%)"], tab_width)
    _table_print(["Completed Trips:", f"{n_com} ({round(100 * n_com / (n_inc + n_com), 1)}%)"], tab_width)

    print(secondary_delineator)
    _table_print("Detectors", tab_width)
    print(secondary_delineator)
    if "detectors" not in sim_data["data"].keys() or len(sim_data["data"]["detectors"]) == 0:
        _table_print("No detectors found.")
    else:
        ils, mees, unknown, labels = [], [], [], ["Induction Loop Detectors", "Multi-Entry-Exit Detectors", "Unknown Type"]
        for det_id, det_info in sim_data["data"]["detectors"].items():
            if det_info["type"] == "inductionloop": ils.append(det_id)
            elif det_info["type"] == "multientryexit": mees.append(det_id)
            else: unknown.append(det_id)
        
        add_spacing = False
        for ids, label in zip([ils, mees, unknown], labels):
            if len(ids) > 0:
                if add_spacing: _table_print(tab_width=tab_width)
                _table_print(label+": ({0})".format(len(ids)), tab_width)
                id_lines = _add_linebreaks(", ".join(ids), tab_width)
                for line in id_lines: _table_print(line, tab_width)
                add_spacing = True
 
    print(secondary_delineator)
    _table_print("Tracked Edges", tab_width)
    print(secondary_delineator)
    if "edges" not in sim_data["data"].keys() or len(sim_data["data"]["edges"]) == 0:
        _table_print("No tracked edges found.")
    else:
        id_lines = _add_linebreaks(", ".join(sim_data["data"]["edges"].keys()), tab_width)
        for line in id_lines: _table_print(line, tab_width)
        
    print(secondary_delineator)
    _table_print("Tracked Junctions", tab_width)
    print(secondary_delineator)
    if "junctions" not in sim_data["data"].keys() or len(sim_data["data"]["junctions"]) == 0:
        _table_print("No tracked junctions found.")
    else:
        for junc_id, junc_info in sim_data["data"]["junctions"].items():
            j_arr = []
            if "tl" in junc_info.keys(): j_arr.append("Signalised")
            if "meter" in junc_info.keys(): j_arr.append("Metered")
            if len(j_arr) == 0: j_arr.append("Uncontrolled")
            _table_print("{0} ({1})".format(junc_id, ", ".join(j_arr)), tab_width)

    if "controllers" in sim_data["data"].keys():
        print(secondary_delineator)
        _table_print("Controllers", tab_width)
        print(secondary_delineator)

        rgs, vsls, unknown, labels = [], [], [], ["Route Guidance", "Variable Speed Limits", "Unknown Type"]
        for cont_id, cont_info in sim_data["data"]["controllers"].items():
            if cont_info["type"] == "RG": rgs.append(cont_id)
            elif cont_info["type"] == "VSL": vsls.append(cont_id)
            else: unknown.append(cont_id)
        
        add_spacing = False
        for ids, label in zip([rgs, vsls, unknown], labels):
            if len(ids) > 0:
                if add_spacing: _table_print(tab_width=tab_width)
                _table_print(label+": ({0})".format(len(ids)), tab_width)
                id_lines = _add_linebreaks(", ".join(ids), tab_width)
                for line in id_lines: _table_print(line, tab_width)
                add_spacing = True
        

    if "events" in sim_data["data"].keys():
        event_statuses = []
        for s in ["scheduled", "active", "completed"]:
            if s in sim_data["data"]["events"].keys() and len(sim_data["data"]["events"][s]) > 0:
                event_statuses.append(s)

        if len(event_statuses) >= 0:
            print(secondary_delineator)
            _table_print("Event IDs & Statuses", tab_width)
            print(secondary_delineator)

            for event_status in event_statuses:
                event_ids = list(sim_data["data"]["events"][event_status].keys())
                event_str = "{0}: {1}".format(event_status.title(), ", ".join(event_ids))
                event_lines = _add_linebreaks(event_str, tab_width)
                for line in event_lines: _table_print(line, tab_width)

    print(secondary_delineator)
    
    sys.stdout = old_stdout
    summary = buffer.getvalue()

    if save_file != None:
        with open(save_file, "w") as fp:
            fp.write(summary)
    else:
        print(summary)

def _table_print(strings=None, tab_width=58, side=" | ", padding=" ", centre_cols=False):
    
    if strings == None: strings = ""
    print_str = None
    if isinstance(strings, str):
        print_str = side+_centre_str(strings, tab_width)+side
    elif isinstance(strings, (list, tuple)):
        if centre_cols:
            col_spacing = padding*math.floor((tab_width - sum([len(str(string)) for string in strings])) / (len(strings) + 1))
            print_str = col_spacing+col_spacing.join([str(string) for string in strings])+col_spacing
            print_str = side+_centre_str(print_str, tab_width)+side
        else:
            col_spacing = padding*math.floor((tab_width - sum([len(str(string)) for string in strings])) / (len(strings) - 1))
            print_str = side+col_spacing.join([str(string) for string in strings])+side
    else:
        desc = "_table_print(): Invalid type (must be [str | list | tuple], not '{0}').".format(type(strings).__name__)
        raise TypeError(desc)
    
    if print_str != None:
        print(print_str)
    else: exit()

def _centre_str(string, width, padding=" "):
    if len(string) > width: return string
    else: return padding*math.floor((width-len(string))/2)+string+padding*math.ceil((width-len(string))/2)

def _add_linebreaks(string, width):
    lines = []
    while string != "":
        if " " not in string or len(string) < width:
            lines.append(string)
            break
        elif " " in string and len(string) > width:
            max_len_str = string[:width]
            line_break_index = max_len_str.rfind(" ")
            lines.append(max_len_str[:line_break_index])
            string = string[line_break_index+1:]
    return lines