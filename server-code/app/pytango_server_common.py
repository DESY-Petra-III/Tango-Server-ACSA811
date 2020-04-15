__author__="Konstantin Glazyrin"

import PyTango

try:
    from PyTango.server import Device, DeviceMeta, attribute, command, server_run, device_property
except (ModuleNotFoundError, ImportError) as e:
    from tango.server import Device, DeviceMeta, attribute, command, server_run, device_property

class CommonDevice(Device):

    PRINT_ENABLE = False

    # internal variables updated by the class
    STATUS = ""

    def init_device(self, *args, **kwargs):
        Device.init_device(self, *args, **kwargs)

    def __init__(self, *args, **kwargs):
        Device.__init__(self, *args, **kwargs)

    def enable_print(self, value=True):
        self.PRINT_ENABLE = value

    def reprint(self, h="", m=""):
        if self.PRINT_ENABLE:
            msg = "{} : {}".format(h, m)
            print(msg)

    def info(self, msg):
        self.info_stream(msg)
        self.reprint("INFO", msg)

    def error(self, msg):
        self.error_stream(msg)
        self.reprint("ERROR", msg)

    def warning(self, msg):
        self.warn_stream(msg)
        self.reprint("WARNING", msg)

    def debug(self, msg):
        self.debug_stream(msg)
        self.reprint("DEBUG", msg)

    def _set_status(self, msg):
        """
        Sets the status for the device
        Args:
            msg: status message

        Returns:
        """
        self.STATUS = "Status: {}\n{}".format(self.get_state(), msg)

    def _append_status(self, msg):
        """
        Sets the status for the device
        Args:
            msg: status message

        Returns:
        """
        self.STATUS += "\n\nStatus: {}\n{}".format(self.get_state(), msg)

    def dev_status(self):
        """
        Returns the status for the device
        Returns:

        """
        return self.STATUS

    def test_value(self, value, type=float):
        """
        Test for a value type
        Args:
            value:
            type:

        Returns:
        """
        return isinstance(value, float)