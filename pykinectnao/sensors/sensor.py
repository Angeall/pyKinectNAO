__author__ = 'Angeall'
import importlib
import pykinectnao.avatars.joints as joints
avatar_package = "pykinectnao.avatars.implemented"

# In order to use the predefined functions you need to
# define the joints supported by your sensor in arguments of init
joint_map = {}

NO_DATA = -1


class Sensor(object):
    def __init__(self, avatar_module_name, _map=joint_map):
        # The avatar_module_name is used to import the motors and the positions needed by the avatar
        assert type(avatar_module_name) == str
        self.avatar_module = importlib.import_module(avatar_package+".%s" % avatar_module_name)
        # These lines need your avatar module to have their dicts well defined
        self.avatar_motors_needed = self.avatar_module.motors_needed
        self.avatar_positions_needed = self.avatar_module.positions_needed

        # Don't forget to link your map with the object, so the methods can reach it
        self.joint_map = _map

        # Lists prepared to gain performance
        self.orientation_pattern_list = []
        self.position_pattern_list =[]
        self.build_patterns()
        # Conversion patterns are build
        self.orientations_conversion_map = {}
        self.positions_conversion_map = {}
        self.build_conversion_maps()
        # Device should be the connection to your sensor
        self.device = None
        # Orientations should contain the joints orientation (list of [Yaw, Pitch, Roll] in degrees)
        self.orientations = None
        # Positions should contain the joints orientation (list of [X, Y, Z])
        self.positions = None

    def build_patterns(self):
        max_index = -1
        # Get the max index for the future list
        for value in self.avatar_motors_needed.values():
            if value[0] > max_index:
                max_index = value[0]
        for i in range(max_index+1):
            # Prepare the list for [Yaw, Pitch, Roll]
            self.orientation_pattern_list.append([None, None, None])

        # If at least one position is needed
        if len(self.avatar_positions_needed) > 0:
            for i in range(max(self.avatar_positions_needed.values())+1):
                self.position_pattern_list.append(None)

    # The conversion maps are build in order to have {..., sensor_joint_index : avatar_joint_index, ...}
    def build_conversion_maps(self):
        # If at least one position is needed
        if len(self.avatar_positions_needed) > 0:
            for key in self.avatar_positions_needed.keys():
                if key in self.joint_map.keys():
                    self.positions_conversion_map[self.joint_map[key]] = self.avatar_positions_needed[key]
        for key in self.avatar_motors_needed.keys():
            if key in self.joint_map.keys():
                self.orientations_conversion_map[self.joint_map[key]] = self.avatar_motors_needed[key]

    # Any descendant should have this method implemented.
    #   It should return a tuple formatted like this : ([motors indication], [positions indication])
    #   and motors indication should be [..., [Yaw, Pitch, Roll], ...]
    #   and positions indication should be [..., [X, Y, Z], ...].
    #   The two lists should be converted, following the two dicts (motors, positions) of the targeted avatar.&
    def get_movement(self, nb_of_body=1):
        # Any descendant could wait for data here
        return self.convert_orientation(), self.convert_positions()

    # Be sure to have the proper joint_map linked with self.joint_map
    # Be sure to have a non-empty list of positions
    # This method match the sensor joint_map with the avatar position_needed map
    # Override if needed
    def convert_positions(self):
        if self.positions is None:
            return []
        positions = self.position_pattern_list[:]
        for index in self.positions_conversion_map.keys():
            positions[self.positions_conversion_map[index]] = self.positions[index]
        return positions

    # Be sure to have the proper joint_map linked with self.joint_map
    # Be sure to have a non-empty list of orientations
    # This method match the sensor joint_map with the avatar motors_needed map
    # Override if needed
    def convert_orientation(self):
        if self.orientations is None:
            return []
        orientations = self.orientation_pattern_list[:]
        for index in self.orientations_conversion_map.keys():
            orientations[self.orientations_conversion_map[index][0]] = self.orientations[index]
        return orientations

    # Any descendant should override this method with the right method
    #   e.g. the Kinect 2 waits for a body to show up and wait again if it loses a body
    def wait_for_data(self, wait_message="Waiting for the sensor to get data..."):
        print wait_message
        pass
