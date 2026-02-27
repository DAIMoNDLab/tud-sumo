import traci
from .utils import *

def _track_vehicle(self, vehicle_id: str, view_id: str | None = None, highlight: bool = True) -> None:
    """
    Sets GUI view to track a vehicle by ID.

    Args:
        `vehicle_id` (str): Vehicle ID
        `view_id` (str, optional): View ID, if `None` uses default
    """
    
    if not self._gui:
        desc = f"Cannot take screenshot (GUI is not active)."
        raise_error(SimulationError, desc, self.curr_step)

    if not self.vehicle_exists(vehicle_id):
        desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
        raise_error(KeyError, desc, self.curr_step)

    if view_id == None: view_id = self._default_view
    if highlight: self.set_vehicle_vals(vehicle_id, highlight=True)

    traci.gui.trackVehicle(view_id, vehicle_id)

    if not isinstance(self._gui_veh_tracking, dict): self._gui_veh_tracking = {}
    self._gui_veh_tracking[view_id] = vehicle_id

def _stop_tracking(self, view_id: str | None) -> None:
    """
    Stops GUI view from tracking vehicle.

    Args:
        `view_id` (str, optional): View ID (defaults to default view)
    """

    if view_id == None: view_id = self._default_view

    if not self._gui:
        desc = f"Cannot stop vehicle tracking (GUI is not active)."
        raise_error(SimulationError, desc, self.curr_step)
    elif not isinstance(self._gui_veh_tracking, dict) or view_id not in self._gui_veh_tracking:
        desc = f"View ID '{view_id}' is not tracking a vehicle."
        raise_error(KeyError, desc, self.curr_step)

    if view_id not in self.get_gui_views():
        desc = f"View ID '{view_id}' not found."
        raise_error(KeyError, desc, self.curr_step)
    
    traci.gui.trackVehicle(view_id, "")

    del self._gui_veh_tracking[view_id]

def _add_view(self, view_id: str, bounds: list | tuple | None = None, zoom: int | float | None = None) -> None:
    """
    Adds a new GUI view.
    
    Args:
        `view_id` (str): View ID
        `bounds` (list, tuple, optional): View bounds coordinates (lower-left, upper-right)
        `zoom` (int, float, optional): Zoom level
    """

    if view_id in self.get_gui_views():
        desc = f"View ID '{view_id}' already exists."
        raise_error(KeyError, desc, self.curr_step)
    
    traci.gui.addView(view_id)
    
    if bounds != None or zoom != None: _set_view(self, view_id, bounds, zoom)
    self._gui_views.append(view_id)

def _set_view(self, view_id: str | None = None, bounds: list | tuple | None = None, zoom: int | float | None = None) -> None:
    """
    Sets the bounds and/or zoom level of a GUI view.

    Args:
        `view_id` (str, optional): View ID (defaults to default view)
        `bounds` (list, tuple, optional): View bounds coordinates (lower-left, upper-right)
        `zoom` (int, float, optional): Zoom level
    """

    if not self._gui:
        desc = f"Cannot set view (GUI is not active)."
        raise_error(SimulationError, desc, self.curr_step)

    if view_id == None: view_id = self._default_view

    if bounds == None and zoom == None:
        desc = "Invalid bounds and zoom (both cannot be 'None')."
        raise_error(ValueError, desc, self.curr_step)
    
    if bounds != None:
        _ = validate_list_types(bounds, (tuple, tuple), True, "bounds", self.curr_step)
        traci.gui.setBoundary(view_id, bounds[0][0], bounds[0][1], bounds[1][0], bounds[1][1])
    if zoom != None: traci.gui.setZoom(view_id, zoom)

def _remove_view(self, view_id: str) -> None:
    """
    Removes a GUI view.
    
    Args:
        `view_id` (str): View ID
    """

    if view_id not in self.get_gui_views():
        desc = f"View ID '{view_id}' not found."
        raise_error(KeyError, desc, self.curr_step)

    elif view_id == self._default_view:
        desc = f"Cannot remove default view '{view_id}'."
        raise_error(ValueError, desc, self.curr_step)

    traci.gui.removeView(view_id)
    self._gui_views.remove(view_id)

def _take_screenshot(self, filename: str, view_id: str | None = None, bounds: list | tuple | None = None, zoom: int | float | None = None) -> None:
    """
    Takes a screenshot of a GUI view and saves result to a file.

    Args:
        `filename` (str): Screenshot filename
        `view_id` (str, optional): View ID (defaults to default view)
        `bounds` (list, tuple, optional): View bounds coordinates (lower-left, upper-right) (defaults to current bounds)
        `zoom` (int, float, optional): Zoom level (defaults to current zoom)
    """

    if not self._gui:
        desc = f"Cannot take screenshot (GUI is not active)."
        raise_error(SimulationError, desc, self.curr_step)

    if view_id == None: view_id = self._default_view

    if bounds != None or zoom != None: self.set_view(view_id, bounds, zoom)

    if not filename.endswith(".png"): filename += ".png"

    traci.gui.screenshot(view_id, filename)