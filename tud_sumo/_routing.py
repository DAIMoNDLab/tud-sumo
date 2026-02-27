import traci
from .utils import *

def _get_path_edges(self, origin: str, destination: str, curr_optimal: bool = True) -> list | None:
    """
    Find an optimal route between 2 edges (using the A* algorithm).
    
    Args:
        `origin` (str): Origin edge ID
        `destination` (str): Destination edge ID
        `curr_optimal` (str): Denotes whether to find current optimal route (ie. whether to consider current conditions)
    
    Returns:
        (None, (list, float)): List of edge IDs & travel time (s) (based on curr_optimal), or None if no route exists
    """
    origin = validate_type(origin, str, "origin", self.curr_step)
    destination = validate_type(destination, str, "destination", self.curr_step)

    if origin == destination:
        desc = "Invalid origin-destination pair ({0}, {1}).".format(origin, destination)
        raise_error(ValueError, desc, self.curr_step)
    if self.geometry_exists(origin) != "edge":
        desc = "Origin edge with ID '{0}' not found.".format(origin)
        raise_error(KeyError, desc, self.curr_step)
    if self.geometry_exists(destination) != "edge":
        desc = "Destination edge with ID '{0}' not found.".format(origin)
        raise_error(KeyError, desc, self.curr_step)

    tt_key = "curr_travel_time" if curr_optimal else "ff_travel_time"

    h = {node: 1 for node in self._all_edges}
    open_list = set([origin])
    closed_list = set([])
    distances = {origin: 0}
    adjacencies = {origin: origin}

    # A* algorithm implemented
    while len(open_list) > 0:
        curr_edge = None

        for node in open_list:
            if curr_edge == None or distances[node] + h[node] < distances[curr_edge] + h[curr_edge]: curr_edge = node

        if curr_edge == None: return None

        if curr_edge == destination:
            optimal_path = []

            while adjacencies[curr_edge] != curr_edge:
                optimal_path.append(curr_edge)
                curr_edge = adjacencies[curr_edge]

            optimal_path.append(origin)
            optimal_path.reverse()

            return optimal_path, self.get_path_travel_time(optimal_path, curr_tt=curr_optimal)

        outgoing_edges = self.get_geometry_vals(curr_edge, "outgoing_edges")
        for connected_edge in outgoing_edges:

            if connected_edge not in open_list and connected_edge not in closed_list:
                open_list.add(connected_edge)
                adjacencies[connected_edge] = curr_edge
                distances[connected_edge] = distances[curr_edge] + self.get_geometry_vals(connected_edge, tt_key)
                
            else:
                if distances[connected_edge] > distances[curr_edge] + self.get_geometry_vals(connected_edge, tt_key):
                    distances[connected_edge] = distances[curr_edge] + self.get_geometry_vals(connected_edge, tt_key)
                    adjacencies[connected_edge] = curr_edge

                    if connected_edge in closed_list:
                        closed_list.remove(connected_edge)
                        open_list.add(connected_edge)

        open_list.remove(curr_edge)
        closed_list.add(curr_edge)

    return None

def _is_valid_path(self, edge_ids: list | tuple) -> bool:
    """
    Checks whether a list of edges is a valid connected path. If two disconnected
    edges are given, it returns whether there is a path between them.
    
    Args:
        `edge_ids` (list, tuple): List of edge IDs
    
    Returns:
        bool: Denotes whether it is a valid path
    """

    edge_ids = validate_list_types(edge_ids, str, param_name="edge_ids", curr_sim_step=self.curr_step)
    if isinstance(edge_ids, (list, tuple)):
        if len(edge_ids) == 0:
            desc = "Empty edge ID list."
            raise_error(ValueError, desc, self.curr_step)
            
        for edge_id in edge_ids:
            if self.geometry_exists(edge_id) != 'edge':
                desc = "Edge with ID '{0}' not found.".format(edge_id)
                raise_error(KeyError, desc, self.curr_step)

        if len(edge_ids) == 1:
            return True
        
        elif len(edge_ids) == 2:
            if edge_ids[-1] not in self.get_geometry_vals(edge_ids[0], "outgoing_edges"):
                return self.get_path_edges(edge_ids[0], edge_ids[1]) != None
            else: return True
        
        else:
            for idx in range(len(edge_ids) - 1):
                if edge_ids[idx + 1] not in self.get_geometry_vals(edge_ids[idx], "outgoing_edges"): return False

    return True

def _add_route(self, routing: list | tuple, route_id: str | None = None, assert_new_id: bool = True) -> None:
    """
    Add a new route. If only 2 edge IDs are given, vehicles calculate
    optimal route at insertion, otherwise vehicles take specific edges.
    
    Args:
        `routing` (list, tuple): List of edge IDs
        `route_id` (str, optional): Route ID, if not given, generated from origin-destination
        `assert_new_id` (bool): If True, an error is thrown for duplicate route IDs
    """
    
    routing = validate_list_types(routing, str, param_name="routing", curr_sim_step=self.curr_step)
    if isinstance(routing, (list, tuple)):
        if len(routing) > 1 and _is_valid_path(self, routing):
            if route_id == None:
                route_id = "{0}_{1}".format(routing[0], routing[-1])

            if self.route_exists(route_id) == None:
                traci.route.add(route_id, routing)
                self._all_routes[route_id] = tuple(routing)
                self._new_routes[route_id] = tuple(routing)

            elif assert_new_id:
                desc = "Route or route with ID '{0}' already exists.".format(route_id)
                raise_error(ValueError, desc, self.curr_step)

            elif not self._suppress_warnings:
                raise_warning("Route or route with ID '{0}' already exists.".format(route_id), self.curr_step)

        else:
            desc = "No valid path between edges '{0}' and '{1}.".format(routing[0], routing[-1])
            raise_error(ValueError, desc, self.curr_step)