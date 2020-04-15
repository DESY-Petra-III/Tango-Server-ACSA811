__author__ = "Konstantin Glazyrin"

import os
import configparser

STORAGE = None

# global variables
class Storage(object):

    DEFAULT_CONFIG_FILE = 'app.ini'
    DEFAULT_SECTION = 'APP_STORAGE'

    KEY_POSITION_OFFSET = "PositionOffset"

    def __init__(self):
        self.ctrl = None
        object.__init__(self)

        self.basedir = None
        self.fnconfig = None
        self.config = None

        self.get_instance()

    def set_ctrl(self, value):
        self.ctrl = value

    def get_instance(self=None):
        global STORAGE
        if STORAGE is None:
            if self is None:
                STORAGE = Storage()
            else:
                STORAGE = self
        return STORAGE

    def set_base_dir(self, value):
        """
        Sets the base directory for the storage
        Initiates configuration file if needed and performs a reading test
        Returns:

        """
        if os.path.isdir(value):
            self.basedir = value
            self.fnconfig = os.path.join(self.basedir, self.DEFAULT_CONFIG_FILE)

            self.config = configparser.ConfigParser()

            # create the file if it does not exits
            # check if the section and the default entry exist
            if not os.path.exists(self.fnconfig):

                self.config.add_section(self.DEFAULT_SECTION)
                self.config.set(self.DEFAULT_SECTION, self.KEY_POSITION_OFFSET, "0.")

                self.update_config()
            else:
                self.config.read(self.fnconfig)

                # test for an existing value
                try:
                    self.config.get(self.DEFAULT_SECTION, self.KEY_POSITION_OFFSET)
                except configparser.NoOptionError:
                    self.config[self.DEFAULT_SECTION][self.KEY_POSITION_OFFSET] = "0."
                    self.update_config()

    def update_config(self):
        """
        Updates the file with changes
        Returns:

        """
        with open(self.fnconfig, "w") as fh:
            self.config.write(fh)

    def get_position_offset(self):
        """
        Returns a key for position offset
        Returns:

        """
        res = self.get_config_value(self.DEFAULT_SECTION, self.KEY_POSITION_OFFSET, type=float, default=0.)

        return res

    def set_position_offset(self, value):
        """
        Sets a new position offset and writes it down
        Returns:

        """
        if value is not None:
            self.set_config_value(self.DEFAULT_SECTION, self.KEY_POSITION_OFFSET, value, frmt="{:6.5f}")

    def _prep_value(self, value, type=float, default=0.):
        """
        Prepares a value to a certain type
        Args:
            value:
            type:

        Returns:

        """
        res = default
        try:
            res = type(value)
        except ValueError:
            pass

        return res

    def set_config_value(self, section, field, value, frmt="{}"):
        """
        Returns a value prepared according to certain format
        Args:
            value:
            frmt:

        Returns:

        """
        res = frmt.format(value)

        self.config.set(section, field, res)
        self.update_config()

        return res

    def get_config_value(self, section, field, type=float, default=0.):
        """
        Returns a field within a specific section
        Args:
            section:
            field:

        Returns:

        """
        res = default
        try:
            res = self.config.get(section, field)
            res = self._prep_value(res, type=type, default=default)
        except configparser.NoOptionError:
            pass

        return res