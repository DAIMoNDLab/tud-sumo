import traci
import moviepy.video.io.ImageSequenceClip as isc
from shutil import rmtree
from .utils import *

class Recorder():

    def __init__(self, simulation):
        from .simulation import Simulation
        self.sim = simulation
        self.sim._recorder = self

        self._recordings = {}

    def __name__(self): return "Recorder"

    def get_recordings(self) -> list:
        """
        Returns a list of all current recording names.

        Returns:
            list: List of recording names
        """

        recordings = list(self._recordings.keys())
        recordings.sort()

        return recordings
    
    def get_recording_data(self, recording_name) -> dict:
        """
        Returns the data for a specific recording.

        Args:
            `recording_name` (str): Name of the recording

        Returns:
            dict: Recording data
        """

        if recording_name not in self._recordings:
            desc = f"Recording '{recording_name}' not found."
            raise_error(KeyError, desc, self.sim.curr_step)
        else: return self._recordings[recording_name]
    
    def save_recording(self, recording_names: str|list|tuple, video_filename: str|None = None, speed: int|float|None = None, delete_frames: bool = True, delete_view: bool = True, overwrite: bool = True) -> None:
        """
        Saves a recording as a video file.

        Args:
            `recording_names` (str, list, tuple): Name of the recording to save.
            `video_filename` (str, optional): Name of the video file (defaults to recording name + mp4)
            `speed` (int, float, optional): Video speed, where fps = speed / step_length (defaults to 1)
            `delete_frames` (bool): Denotes whether to delete video frames once done
            `delete_view` (bool): Denotes whether to delete the view once done
            `overwrite` (bool): Denotes whether to allow overwriting of an existing video file
        """

        if not isinstance(recording_names, (list, tuple)): recording_names = [recording_names]
        for recording_name in recording_names:
            if recording_name not in self._recordings:
                desc = f"Recording '{recording_name}' not found."
                raise_error(KeyError, desc, self.sim.curr_step)
            else: recording_data = self._recordings[recording_name]

            if len(recording_data["frame_files"]) < 3:
                desc = f"Cannot create video '{recording_name}' (insufficient frames)."
                raise_error(ValueError, desc, self.sim.curr_step)

            if video_filename == None: video_filename = recording_name + ".mp4"
            if os.path.exists(video_filename) and overwrite:
                if not self.sim._suppress_warnings: raise_warning("Video file '{0}' already exists and will be overwritten.".format(video_filename), self.sim.curr_step)
            elif os.path.exists(video_filename) and not overwrite:
                desc = "Video file '{0}' already exists and cannot be overwritten.".format(video_filename)
                raise_error(FileExistsError, desc, self.sim.curr_step)

            if speed == None: speed = recording_data["speed"]

            if delete_view and recording_data["view_id"] != self.sim._default_view:
                traci.gui.removeView(recording_data["view_id"])
            else:
                self.sim.set_view(recording_data["view_id"], recording_data["default_bounds"], recording_data["default_zoom"])

            video = isc.ImageSequenceClip(recording_data["frame_files"][1:], fps=int(speed / self.sim.step_length))
            video.write_videofile(video_filename, logger=None)

            if delete_frames: rmtree(recording_data['frames_loc'])
            del self._recordings[recording_name]

    def _setup_recording(self, recording_name: str, frames_loc: str, empty_frames_loc: bool, view_id: str):
        """
        Standard setup for recordings (creating views/directories).

        Args:
            `recording_name` (str): Name of the recording
            `frames_loc` (str): Video frames directory (defaults to 'recording_name'_frames/)
            `empty_frames_loc` (bool): Denotes whether to delete contents of 'frames_loc' if it already exists
            `view_id` (str): View ID for recording
        """

        if recording_name in self._recordings:
            desc = f"Invalid recording_name '{recording_name}' (already in use)."
            raise_error(ValueError, desc, self.sim.curr_step)

        if view_id in [recording["view_id"] for recording in self._recordings.values()]:
            desc = f"Invalid view ID '{view_id}' (already in use)."
            raise_error(ValueError, desc, self.sim.curr_step)
        elif not traci.gui.hasView(view_id):
            traci.gui.addView(view_id)

        if frames_loc in [recording["frames_loc"] for recording in self._recordings.values()]:
            desc = f"Invalid frames_loc '{frames_loc}' (already in use)."
            raise_error(ValueError, desc, self.sim.curr_step)
        elif os.path.exists(frames_loc):
            if empty_frames_loc:
                rmtree(frames_loc)
            else:
                print(os.listdir(frames_loc))
                if len([file for file in os.listdir(frames_loc) if not file.startswith('.')]) > 0:
                    desc = f"Invalid frames_loc '{frames_loc}' (already exists)."
                    raise_error(ValueError, desc, self.sim.curr_step)
        
        if not os.path.exists(frames_loc): os.makedirs(frames_loc)

    def record_network(self, bounds: list|tuple, recording_name: str, zoom: int|float|None = None, speed: int|float|None = 1, frames_loc: str|None = None, empty_frames_loc: bool = True, view_id: str|None = None) -> None:
        """
        Records a location on the network.

        Args:
            `bounds` (list, tuple): Video view bounds coordinates (lower-left, upper-right) (defaults to current bounds)
            `recording_name` (str): Recording name
            `zoom` (int, optional): Recording zoom level
            `speed` (int, float, optional): Video speed, where fps = speed / step_length (defaults to 1)
            `frames_loc` (str, optional): Video frames directory (defaults to 'recording_name'_frames/)
            `empty_frames_loc` (bool): Denotes whether to delete contents of 'frames_loc' if it already exists
            `view_id` (str, optional): Recording view ID (defaults to main view)
        """
        
        if not self.sim._gui:
            desc = f"Cannot record video (GUI is not active)."
            raise_error(ValueError, desc, self.sim.curr_step)
        
        if view_id == None: view_id = self.sim._default_view
        if frames_loc == None: frames_loc = recording_name+"_frames"
        self._setup_recording(recording_name, frames_loc, empty_frames_loc, view_id)

        default_bounds = traci.gui.getBoundary(view_id)
        default_zoom = traci.gui.getZoom(view_id)
        if zoom == None: zoom = default_zoom

        self._recordings[recording_name] = {"start_step": self.sim.curr_step,
                                            "bounds": bounds,
                                            "zoom": zoom,
                                            "default_bounds": default_bounds,
                                            "default_zoom": default_zoom,
                                            "frames_loc": frames_loc,
                                            "frame_files": [],
                                            "view_id": view_id,
                                            "speed": speed}

        self.sim.set_view(view_id, bounds, zoom)

    def record_vehicle(self, vehicle_id: str, recording_name: str, zoom: int|float|None = None, speed: int|float|None = 1, frames_loc: str|None = None, empty_frames_loc: bool = True, view_id: str|None = None, highlight: bool = True) -> None:
        """
        Tracks and records a vehicle until it has left the network (or is saved earlier).

        Args:
            `vehicle_id` (tuple): Vehicle ID
            `recording_name` (str): Recording name
            `zoom` (int, optional): Recording zoom level
            `speed` (int, float, optional): Video speed, where fps = speed / step_length (defaults to 1)
            `frames_loc` (str, optional): Video frames directory (defaults to 'recording_name'_frames/)
            `empty_frames_loc` (bool): Denotes whether to delete contents of 'frames_loc' if it already exists
            `view_id` (str, optional): Recording view ID (defaults to main view)
            `highlight` (bool): Denotes whether to highlight the tracked vehicle
        """
        
        if not self.sim._gui:
            desc = f"Cannot record vehicle '{vehicle_id}' (GUI is not active)."
            raise_error(ValueError, desc, self.sim.curr_step)

        if not self.sim.vehicle_exists(vehicle_id):
            desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.sim.curr_step)

        if view_id == None: view_id = self.sim._default_view
        if frames_loc == None: frames_loc = recording_name+"_frames"
        self._setup_recording(recording_name, frames_loc, empty_frames_loc, view_id)

        default_bounds = traci.gui.getBoundary(view_id)
        default_zoom = traci.gui.getZoom(view_id)
        if zoom == None: zoom = default_zoom

        self._recordings[recording_name] = {"vehicle_id": vehicle_id,
                                            "start_step": self.sim.curr_step,
                                            "bounds": None,
                                            "zoom": zoom,
                                            "default_bounds": default_bounds,
                                            "default_zoom": default_zoom,
                                            "frames_loc": frames_loc,
                                            "frame_files": [],
                                            "view_id": view_id,
                                            "speed": speed}

        self.sim.gui_track_vehicle(vehicle_id, view_id, highlight)
        self.sim.set_view(view_id, None, zoom)