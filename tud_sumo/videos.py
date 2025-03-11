import moviepy.video.io.ImageSequenceClip as isc
from shutil import rmtree
from .utils import *

class Recorder():

    def __init__(self, simulation):
        from .simulation import Simulation
        self.sim = simulation
        self.sim._recorder = self

        self._recordings = {}
        self.valid_extensions = [".mp4", ".avi", ".gif"]

    def __name__(self): return "Recorder"

    def get_recordings(self) -> list:
        """
        Returns a list of the IDs of all current recordings.

        Returns:
            list: List of recording IDs
        """

        recordings = list(self._recordings.keys())
        recordings.sort()

        return recordings
    
    def get_recording_data(self, recording_id) -> dict:
        """
        Returns the data for a specific recording.

        Args:
            `recording_id` (str): Recording ID

        Returns:
            dict: Recording data
        """

        if recording_id not in self._recordings:
            desc = f"Recording '{recording_id}' not found."
            raise_error(KeyError, desc, self.sim.curr_step)
        else: return self._recordings[recording_id]
    
    def save_recording(self, recording_id: str, video_filename: str|None = None, speed: int|float|None = None, delete_frames: bool = True, delete_view: bool = True, overwrite: bool = True) -> None:
        """
        Saves a recording as a video file.

        Args:
            `recording_ids` (str): Recording ID
            `video_filename` (str, optional): Name of the video file (defaults to recording name + 'mp4')
            `speed` (int, float, optional): Video speed, where fps = speed / step_length (defaults to 1)
            `delete_frames` (bool): Denotes whether to delete video frames once done
            `delete_view` (bool): Denotes whether to delete the view once done
            `overwrite` (bool): Denotes whether to allow overwriting of an existing video file
        """

        if recording_id not in self._recordings:
            desc = f"Recording '{recording_id}' not found."
            raise_error(KeyError, desc, self.sim.curr_step)
        else: recording_data = self._recordings[recording_id]

        if len(recording_data["frame_files"]) < 3:
            desc = f"Cannot create video '{recording_id}' (insufficient frames)."
            raise_error(ValueError, desc, self.sim.curr_step)

        if video_filename == None:
            if "video_filename" in recording_data and recording_data["video_filename"] != None:
                video_filename = recording_data["video_filename"]
            else: video_filename = recording_id + ".mp4"

        if video_filename != None:
            valid = False
            for extension in self.valid_extensions:
                if video_filename.endswith(extension): valid = True
            if not valid:
                desc = f"Invalid video filename '{video_filename}' (must end with '.mp4', '.avi' or '.gif')."
                raise_error(ValueError, desc, self.sim.curr_step)

        if os.path.exists(video_filename) and overwrite:
            if not self.sim._suppress_warnings: raise_warning("Video file '{0}' already exists and will be overwritten.".format(video_filename), self.sim.curr_step)
        elif os.path.exists(video_filename) and not overwrite:
            desc = "Video file '{0}' already exists and cannot be overwritten.".format(video_filename)
            raise_error(FileExistsError, desc, self.sim.curr_step)

        if speed == None: speed = recording_data["speed"]

        if delete_view and recording_data["view_id"] != self.sim._default_view:
            self.sim.remove_gui_view(recording_data["view_id"])
        else: self.sim.set_view(recording_data["view_id"], recording_data["default_bounds"], recording_data["default_zoom"])

        video = isc.ImageSequenceClip(recording_data["frame_files"][1:], fps=int(speed / self.sim.step_length))

        if video_filename.endswith(".gif"): video.write_gif(video_filename, logger=None)
        else:
            codec = "mpeg4" if video_filename.endswith(".mp4") else "rawvideo"
            video.write_videofile(video_filename, codec=codec, logger=None, audio=False)

        if delete_frames: rmtree(recording_data['frames_loc'])
        del self._recordings[recording_id]

    def _setup_recording(self, recording_id: str, frames_loc: str, empty_frames_loc: bool, view_id: str):
        """
        Standard setup for recordings (creating views/directories).

        Args:
            `recording_id` (str): Recording ID
            `frames_loc` (str): Video frames directory (defaults to 'recording_id'_frames/)
            `empty_frames_loc` (bool): Denotes whether to delete contents of 'frames_loc' if it already exists
            `view_id` (str): View ID for recording
        """

        if recording_id in self._recordings:
            desc = f"Invalid recording_id '{recording_id}' (already in use)."
            raise_error(ValueError, desc, self.sim.curr_step)

        if view_id in [recording["view_id"] for recording in self._recordings.values()]:
            desc = f"Invalid view ID '{view_id}' (already in use)."
            raise_error(ValueError, desc, self.sim.curr_step)
        elif view_id not in self.sim.get_gui_views():
            self.sim.add_gui_view(view_id)

        if frames_loc in [recording["frames_loc"] for recording in self._recordings.values()]:
            desc = f"Invalid frames_loc '{frames_loc}' (already in use)."
            raise_error(ValueError, desc, self.sim.curr_step)
        elif os.path.exists(frames_loc):
            if empty_frames_loc:
                rmtree(frames_loc)
            else:
                if len([file for file in os.listdir(frames_loc) if not file.startswith('.')]) > 0:
                    desc = f"Invalid frames_loc '{frames_loc}' (already exists)."
                    raise_error(ValueError, desc, self.sim.curr_step)
        
        if not os.path.exists(frames_loc): os.makedirs(frames_loc)

    def record_network(self, bounds: list|tuple, recording_id: str, video_filename: str|None = None, zoom: int|float|None = None, speed: int|float|None = 1, frames_loc: str|None = None, empty_frames_loc: bool = True, view_id: str|None = None) -> None:
        """
        Records a location on the network.

        Args:
            `bounds` (list, tuple): Video view bounds coordinates (lower-left, upper-right) (defaults to current bounds)
            `recording_id` (str): Recording ID
            `video_filename` (str, optional): Name of the video file (defaults to recording name + 'mp4')
            `zoom` (int, optional): Recording zoom level
            `speed` (int, float, optional): Video speed, where fps = speed / step_length (defaults to 1)
            `frames_loc` (str, optional): Video frames directory (defaults to 'recording_id'_frames/)
            `empty_frames_loc` (bool): Denotes whether to delete contents of 'frames_loc' if it already exists
            `view_id` (str, optional): Recording view ID (defaults to main view)
        """
        
        if not self.sim._gui:
            desc = f"Cannot record video (GUI is not active)."
            raise_error(ValueError, desc, self.sim.curr_step)
        
        if view_id == None: view_id = self.sim._default_view
        if frames_loc == None: frames_loc = recording_id+"_frames"
        self._setup_recording(recording_id, frames_loc, empty_frames_loc, view_id)

        default_bounds = self.sim.get_view_boundaries(view_id)
        default_zoom = self.sim.get_view_zoom(view_id)
        if zoom == None: zoom = default_zoom

        if video_filename == None: video_filename = recording_id + ".mp4"

        self._recordings[recording_id] = {"video_filename": video_filename,
                                          "start_step": self.sim.curr_step,
                                          "bounds": bounds,
                                          "zoom": zoom,
                                          "default_bounds": default_bounds,
                                          "default_zoom": default_zoom,
                                          "frames_loc": frames_loc,
                                          "frame_files": [],
                                          "view_id": view_id,
                                          "speed": speed}

        self.sim.set_view(view_id, bounds, zoom)

    def record_vehicle(self, vehicle_id: str, recording_id: str, video_filename: str|None = None, zoom: int|float|None = None, speed: int|float|None = 1, frames_loc: str|None = None, empty_frames_loc: bool = True, view_id: str|None = None, highlight: bool = True) -> None:
        """
        Tracks and records a vehicle until it has left the network (or is saved earlier).

        Args:
            `vehicle_id` (tuple): Vehicle ID
            `recording_id` (str): Recording ID
            `video_filename` (str, optional): Name of the video file (defaults to recording name + '.mp4')
            `zoom` (int, optional): Recording zoom level
            `speed` (int, float, optional): Video speed, where fps = speed / step_length (defaults to 1)
            `frames_loc` (str, optional): Video frames directory (defaults to 'recording_id'_frames/)
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
        if frames_loc == None: frames_loc = recording_id+"_frames"
        self._setup_recording(recording_id, frames_loc, empty_frames_loc, view_id)

        default_bounds = self.sim.get_view_boundaries(view_id)
        default_zoom = self.sim.get_view_zoom(view_id)
        if zoom == None: zoom = default_zoom

        if video_filename == None: video_filename = recording_id + ".mp4"

        self._recordings[recording_id] = {"video_filename": video_filename,
                                          "vehicle_id": vehicle_id,
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