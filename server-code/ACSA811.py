__author__ = "Konstantin Glazyrin"

"""
License: LGPL v.3 for the code of Tango Server itself, for  licenses of the custom made libraries.
External libraries are covered by the corresponding license agreements. 
"""

import re
import time

from app import *

class A811ctrl(CommonDevice):
    __metaclass__ = DeviceMeta

    # constants
    TCP_PORT = 701
    LINE_TERMINATION = "\r"
    SOCKET_TIMEOUT = 0.5                # in seconds
    BUFFER_SIZE = 256
    THREAD_DELAY = 0.2                  # in seconds - delay for sleeping with cached values
    THREAD_STOP_COMMAND = "Stop"

    # device property - channel number
    IPaddress = device_property(dtype=str, default_value="10.0.0.100", update_db=True)

    #  only values / commands shown in cache (keys) are cached, other values are polled in situ
    CACHE = {}

    def __init__(self, *args, **kwargs):
        """
        Initialization of the device. Creating global controller handle
        :param args:
        :param kwargs:
        """
        CommonDevice.__init__(self, *args, **kwargs)

    def init_device(self, *args, **kwargs):
        """
        Initialization of intrinsic variables
        :param args:
        :param kwargs:
        :return:
        """
        self.storage = Storage.get_instance()
        self.storage.ctrl = self

        self.PRINT_ENABLE = False
        self.get_device_properties()

        self.ip = self.IPaddress
        self.socket = None

        # locks
        self.lock_cache = threading.Lock()
        self.lock_socket = threading.Lock()

        # threading with cache
        self.queue_quit = queue.Queue()
        self.thread_cache = threading.Thread(target=self.worker_cache, args=[self])

        self.threads = [self.thread_cache]

        # default state
        self.set_state(DevState.UNKNOWN)

        if self.TestPing():
            self._set_status("Host {} is online".format(self.ip))

            # set the socket
            self._open_socket()
        else:
            self.set_state(DevState.FAULT)
            self._set_status("Host {} is offline".format(self.ip))

    def worker_cache(self, obj):
        """
        Function
        Returns:

        """
        # for convinience with IDE
        if isinstance(obj, A811ctrl):
            pass

        qstop = obj.queue_quit

        while True:
            # test with a queue
            try:
                test = qstop.get(block=False)
                qstop.task_done()
                break
            except queue.Empty:
                pass

            tstart = time.time()

            keys = obj._get_cache_keys()

            if len(keys) > 0:
                for cmd in keys:
                    tvalue = obj.SocketWriteRead(cmd)
                    obj._set_cache_value(cmd, tvalue)

            # test with a stop queue
            try:
                test = qstop.get(block=False)
                qstop.task_done()
                break
            except queue.Empty:
                pass

            # try to make Hz based operation
            tdelay = obj.THREAD_DELAY - (time.time() - tstart)
            if tdelay > 0:
                time.sleep(tdelay)

    def _open_socket(self):
        """
        Opens the socket
        Returns:
        """
        res = False

        with self.lock_socket:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(0.5)

                self.socket.connect((self.ip, self.TCP_PORT))
                self.set_state(DevState.ON)

                self._append_status("Socket connected")
            except (socket.herror, socket.gaierror, socket.error) as e:
                self.set_state(DevState.FAULT)
                self._append_status("Socket error: {}".format(e))

        return res

    def _close_socket(self):
        """
        Closes the socket
        Returns:
        """
        if self.socket  is not None:
            self.socket.close()

    def test_controller(self):
        res = True
        if self.ctrl is None:
            res = False
        return res

    def _close_threads(self):
        """
        Performs a cleanup of worker threads through a queue
        Returns:

        """
        for th in self.threads:
            if th.is_alive():
                self.queue_quit.put(self.THREAD_STOP_COMMAND)
        self.queue_quit.join()

        self.info("Successfully cleaned the threads")

    def delete_device(self):
        """
        Performes the cleanup for the device
        Returns:
        """
        # cleanup of the threads
        self._close_threads()

        # cleanup of the socket
        if self.socket is not None:
            self._close_socket()
            self.info("Successfully closed the socket")

    # commands
    @command(dtype_out=bool)
    def TestPing(self):
        """
        Tests if the host is reachable by pinging it
        Returns:
        """

        # Option for the number of packets as a function of
        params = []
        pattern = "0 received"
        if platform.system().lower() == 'windows':
            params = ["-n", "1", "-w", "500"]
            pattern = "Received = 0"
        else:
            params = ["-c", "1", "-w", "1"]

        cmd = ["ping"] + params + [self.ip]

        cmplproc = subprocess.run(cmd, capture_output=True)
        cmplproc.stdout = cmplproc.stdout.decode()

        # test for the host to be online
        p = re.compile(pattern, re.IGNORECASE and re.MULTILINE)

        return len(p.findall(cmplproc.stdout)) == 0

    @command(dtype_in=str, dtype_out=str)
    def SocketWriteRead(self, cmd, bread=True):
        """
        Writes and reads from the device. By default the A811 always returns a string.
        Either full or
        Args:
            cmd: string, non terminated

        Returns:
        """
        res = None
        state = self.get_state()

        if self.socket == None and state != DevState.FAULT:
            self._set_status("Socket {} is offline".format(self.ip))
            self.set_state(DevState.FAULT)
            return

        # accept the commands if the device is virtually online
        if state != DevState.FAULT:
            pass

        # use cache
        if cmd in self.CACHE:
            res = self._get_cache_value(cmd)
        else:
            # use socket
            with self.lock_socket:
                try:
                    cmd_out = str(cmd + self.LINE_TERMINATION).encode()

                    # send message
                    self.socket.send(cmd_out)

                    # recover a message if needed
                    if bread:
                        data = b""

                        # fill the data by buffer reading
                        while True:
                            tblock = self.socket.recv(self.BUFFER_SIZE)
                            data += tblock
                            if len(tblock) < self.BUFFER_SIZE:
                                break
                        res = data
                except (socket.herror, socket.gaierror, socket.error) as e:
                    self.set_state(DevState.FAULT)
                    self._append_status("Socket error: {}".format(e))

        # remove the unusual symbols, otherwise decode is confused
        res = res[:res.index(self.LINE_TERMINATION.encode())]
        return str(res.decode())

    @command(dtype_in=str)
    def SocketWrite(self, cmd):
        """
        Writes a socket without a necessity for a response
        Args:
            cmd:

        Returns:
        """
        self.SocketWriteRead(cmd, bread=False)

    def _get_cache_value(self, key):
        """
        Returns cache value with a lock
        Args:
            key:

        Returns:
        """
        res = None
        with self.lock_cache:
            res = self.CACHE[key]
        return res

    def _set_cache_value(self, key, value):
        """
        Sets cache value with a lock
        Args:
            key:
            value:

        Returns:
        """
        with self.lock_cache:
            if not key in self.CACHE:
                self.CACHE.setdefault(key, value)
            else:
                self.CACHE[key] = value

    def _get_cache_keys(self):
        """
        Returns a copy of cached values with a lock
        Returns:
        """
        res = None
        with self.lock_cache:
            res = copy.deepcopy(self.CACHE.keys())
        return res


class A811Channel(CommonDevice):
    __metaclass__ = DeviceMeta

    # DEFAULTS
    DEFAULT_POSITION_OFFSET = 0.
    DEFAULT_CONVERSION = 1.

    # HW COMMANDS, for convinience {} replaced by []
    CMD_MOVING_STATE = "?[%i]MST({}).#MOVE"
    CMD_ENABLED_STATE = "?[%i]MST({}).#ENABLED"         # property is only set by the startup script
    CMD_MOTOR_STAT = "?$${}"

    # HW COMMANDS for attributes reading
    CMD_READ_VELOCITY = "?[%10.10f]VEL({})"
    CMD_READ_ACCELERATION = "?[%10.10f]ACC({})"
    CMD_READ_DECELERATION = "?[%10.10f]DEC({})"
    CMD_READ_KILLDECELERATION = "?[%10.10f]KDEC({})"
    CMD_READ_POSITION = "?[%10.10f]FPOS({})"

    # HW COMMANDS setting attributes
    CMD_WRITE_VELOCITY = "IMM VEL({})={}"
    CMD_WRITE_ACCELERATION = "IMM ACC({})={}"
    CMD_WRITE_DECELERATION = "IMM DEC({})={}"
    CMD_WRITE_KILLDECELERATION = "IMM KDEC({})={}"
    CMD_WRITE_POSITION = "PTP({}),{}"

    CMD_WRITE_STOP = "HALT {}"
    CMD_WRITE_TEST = "?${}"

    CMD_HOME = "START 8,home0"

    # device property - channel number
    AxisID = device_property(dtype=int, default_value=0, update_db=True)
    AxisName = device_property(dtype=str, default_value="a811_0", update_db=True)

    # Attributes
    #Position = attribute(label="Position", dtype=float,
    #                     fget="get_position", fset="set_position",
    #                     doc="Raw position of the motor")
    # Acceleration of the piezo drive
    Enabled = attribute(label="IsEnabled", dtype=bool,
                         fget="get_enabled",
                         doc="Enabled state of the motor")

    Velocity = attribute(label="Velocity", dtype=float, format="6.4f",
                         fget="get_velocity", fset="set_velocity",
                         doc="Velocity value of the motor", unit="units/sec")

    EncoderPosition = attribute(label="EncoderPosition", dtype=float, format="6.4f",
                             fget="get_encposition", fset="set_encposition",
                             doc="Encoded position of the motor (raw)", unit="units")

    Position = attribute(label="Position", dtype=float, format="6.4f",
                                fget="get_position", fset="set_position",
                                doc="Encoded position of the motor (raw)", unit="units")

    Acceleration = attribute(label="Acceleration", dtype=float, format="6.4f",
                         fget="get_acceleration", fset="set_acceleration",
                         doc="Acceleration of the motor", unit="units/sec^2")

    Deceleration = attribute(label="Deceleration", dtype=float, format="6.4f", memorized=False,
                             fget="get_deceleration", fset="set_deceleration",
                             doc="Deceleration of the motor", unit="units/sec^2")

    KillDeceleration = attribute(label="KillDeceleration", dtype=float, format="6.4f",
                             fget="get_killdeceleration", fset="set_killdeceleration",
                             doc="Deceleration on a controlled quick stop", unit="units/sec^2")

    # static, memorized attributes
    PositionOffset = attribute(label="PositionOffset", dtype=float, memorized=True, format="6.4f",
                             fget="get_position_offset", fset="set_position_offset",
                             doc="Offset between the real motor position and the user position. Calibration of encoded motors.",
                             unit="units")

    Conversion = attribute(label="Conversion", dtype=float, memorized=True, format="6.4f",
                               fget="get_position_conversion", fset="set_position_conversion",
                               doc="Conversion parameter for the motor position",
                               unit="units")

    UnitLimitMin = attribute(label="UnitLimitMin", dtype=float, memorized=True, format="6.4f",
                           fget="get_unit_min", fset="set_unit_min",
                           doc="Unit limit for the motor (minimum value)",
                           unit="units")

    UnitLimitMax = attribute(label="UnitLimitMax", dtype=float, memorized=True, format="6.4f",
                             fget="get_unit_max", fset="set_unit_max",
                             doc="Unit limit for the motor (maximum value)",
                             unit="units")

    def __init__(self, *args, **kwargs):
        CommonDevice.__init__(self, *args, **kwargs)

        self.PRINT_ENABLE = False

    def delete_device(self):
        """
        Cleaning up
        Returns:
        """
        self.reprint("At stop")

    def init_device(self, *args, **kwargs):
        """
        Initializes device, sets device channel from device property
        """
        CommonDevice.init_device(self)

        # tests for the directory
        self.storage = Storage.get_instance()
        self.storage.set_base_dir(os.path.dirname(__file__))

        self.ctrl = self.storage.ctrl

        self.reprint("Config. Position Offset", self.storage.get_position_offset() )

        if isinstance(self.ctrl, A811ctrl):
            pass

        self.get_device_properties()

        self.axis_id = self.AxisID
        self.axis_name = self.AxisName

        # HW commands, format update for a channel
        self.cmd_move_state = self.CMD_MOVING_STATE.format(self.axis_id)

        self.cmd_enabled = self.CMD_ENABLED_STATE.format(self.axis_id)
        
        self.cmd_read_velocity = self.CMD_READ_VELOCITY.format(self.axis_id)
        self.cmd_write_velocity = self.CMD_WRITE_VELOCITY.format(self.axis_id, "{}")

        self.cmd_read_acceleration = self.CMD_READ_ACCELERATION.format(self.axis_id)
        self.cmd_write_acceleration = self.CMD_WRITE_ACCELERATION.format(self.axis_id, "{}")

        self.cmd_read_deceleration = self.CMD_READ_DECELERATION.format(self.axis_id)
        self.cmd_write_deceleration = self.CMD_WRITE_DECELERATION.format(self.axis_id, "{}")

        self.cmd_read_killdeceleration = self.CMD_READ_KILLDECELERATION.format(self.axis_id)
        self.cmd_write_killdeceleration = self.CMD_WRITE_KILLDECELERATION.format(self.axis_id, "{}")

        self.cmd_read_position = self.CMD_READ_POSITION.format(self.axis_id)
        self.cmd_write_position = self.CMD_WRITE_POSITION.format(self.axis_id, "{}")

        self.cmd_write_stop = self.CMD_WRITE_STOP.format(self.axis_id)
        self.cmd_write_test = self.CMD_WRITE_TEST.format(self.axis_id)

        self.cmd_write_home = self.CMD_HOME

        # force values
        self.force_position_offset = None

        # home the axis if possible
        if self.ctrl.get_state() != DevState.FAULT:
            #self.MoveHome()
            #time.sleep(1)
            pass

        # sleep for a correct state estimation


        # test of the state
        self.dev_state()
        self._set_status("AxisID: ({}) {}".format(self.axis_id, self.axis_name))

    def read_PositionOffset(self, attr):
        attr.set_value(self.DEFAULT_POSITION_OFFSET)

    def read_Conversion(self, attr):
        attr.set_value(self.DEFAULT_CONVERSION)

    def _check_parent_state(self):
        """
        Tests the parent state before doing anything
        Returns:
        """
        res = True
        if self.ctrl.get_state() == DevState.FAULT:
            self.set_state(DevState.FAULT)
            res = False
        else:
            self.set_state(DevState.ON)
        return res

    def dev_state(self):
        """
        Returns a state with a modification according to the state of the parent
        Returns:
        """
        # check parent
        test =  self._check_parent_state()

        if not test:
            return DevState.FAULT

        test = self.get_enabled()
        state = DevState.FAULT
        if not test:
            self._append_status("Motor is disabled")
            self.set_state(state)
            return state

        cmd = self.cmd_move_state
        test = self._submit_command_ctrl(cmd)
        test = self._prep_value(test, bool)
        if test is None:
            state = DevState.FAULT
            self.set_state(state)
            return state
        elif test:
            state = DevState.MOVING
            self.set_state(state)
            return state
        else:
            state = DevState.ON
            self.set_state(state)
            return state

    def get_velocity(self):
        """
        Returns velocity of the motor
        Returns:
        """
        res = self._submit_command_ctrl(self.cmd_read_velocity)
        res = self._prep_value(res, float)

        return res

    def set_velocity(self, value):
        """
        Sets velocity of the motor
        Args:
            value:

        Returns:
        """
        self._submit_command_ctrl(self.cmd_write_velocity.format(value))

    def get_acceleration(self):
        """
        Returns acceleration of the motor
        Returns:
        """
        res = self._submit_command_ctrl(self.cmd_read_acceleration)
        res = self._prep_value(res, float)

        return res

    def set_acceleration(self, value):
        """
        Sets acceleration of the motor (deceleration is automatically set to the same value as acceleration)
        Args:
            value:

        Returns:
        """
        self._submit_command_ctrl(self.cmd_write_acceleration.format(value))
        self._submit_command_ctrl(self.cmd_write_deceleration.format(value))

    def get_deceleration(self):
        """
        Returns deceleration of the motor
        Returns:
        """
        res = self._submit_command_ctrl(self.cmd_read_deceleration)
        res = self._prep_value(res, float)
        return res

    def set_deceleration(self, value):
        """
        Sets deceleration of the motor (acceleration is not affected)
        Args:
            value:

        Returns:
        """
        self._submit_command_ctrl(self.cmd_write_deceleration.format(value))

    def set_killdeceleration(self, value):
        """
        Sets kill deceleration of the motor (sudden stop)
        Args:
            value:

        Returns:
        """
        self._submit_command_ctrl(self.cmd_write_killdeceleration.format(value))

    def get_killdeceleration(self):
        """
        Returns kill deceleration of the motor (sudden stop)
        Returns:
        """
        res = self._submit_command_ctrl(self.cmd_read_killdeceleration)
        res = self._prep_value(res, float)

        return res

    def set_position_offset(self, value):
        """
        Sets the value for the position offset (MEMORIZED attribute)
        Args:
            value:

        Returns:
        """
        self.PositionOffset.set_write_value(value)
        self.PositionOffset.set_value(value)
        self.storage.set_position_offset(value)

    def get_position_offset(self):
        """
        Returns the memorized position offset
        Returns:
        """

        # only used for rare backups
        res = self.PositionOffset.get_write_value()

        tres = self.storage.get_position_offset()
        if tres != None:
            res = tres

        #self.reprint("Position offset parameters\nMin/Max/Value/Write Value", "{} / {} / {}".format(
        #        self.PositionOffset.get_min_value(),
        #        self.PositionOffset.get_max_value(),
        #        self.PositionOffset.get_write_value()
        #    ))

        if not -180. < res < 180.:
            res = 0.
            self.set_position_offset(res)

        # self.reprint("Current position offset", "{}".format(res))

        return res

    def set_position_conversion(self, value):
        """
        Sets the value for the position offset (MEMORIZED attribute)
        Args:
            value:

        Returns:
        """
        self.Conversion.set_write_value(value)
        self.Conversion.set_value(value)

    def get_position_conversion(self):
        """
        Returns the memorized position offset
        Returns:
        """
        # take care of unreasonable conversion factors
        res = self.Conversion.get_write_value()
        if res == 0.:
            self.Conversion.set_value(self.DEFAULT_CONVERSION)
            res = self.DEFAULT_CONVERSION

        return res

    def set_unit_min(self, value):
        """
        Sets the value for the position UnitLimitMin (MEMORIZED attribute)
        Args:
            value:

        Returns:
        """
        self.UnitLimitMin.set_write_value(value)
        self.UnitLimitMin.set_value(value)

    def get_unit_min(self):
        """
        Returns the memorized position UnitLimitMin
        Returns:
        """
        # take care of unreasonable conversion factors
        res = self.UnitLimitMin.get_write_value()
        return res

    def set_unit_max(self, value):
        """
        Sets the value for the position UnitLimitMax (MEMORIZED attribute)
        Args:
            value:

        Returns:
        """
        self.UnitLimitMax.set_write_value(value)
        self.UnitLimitMax.set_value(value)

    def get_unit_max(self):
        """
        Returns the memorized position UnitLimitMax
        Returns:
        """
        # take care of unreasonable conversion factors
        res = self.UnitLimitMax.get_write_value()
        return res

    def set_encposition(self, value):
        """
        Sets the encoded position (raw, no conversion, no offset)
        Args:
            value:

        Returns:
        """
        if self.get_state() != DevState.FAULT and self.get_enabled():
            self._submit_command_ctrl(self.cmd_write_position.format(value))

    def get_encposition(self):
        """
        Returns the encoded position (raw, no conversion, no offset)
        Returns:
        """
        res = self._submit_command_ctrl(self.cmd_read_position)
        res = self._prep_value(res, float)

        return res

    def set_position(self, value):
        """
        Sets the encoded position (raw, no conversion, no offset)
        Args:
            value:

        Returns:
        """
        # limit the movement within the unit limits
        xi, xa = self.UnitLimitMin.get_write_value(), self.UnitLimitMax.get_write_value()

        if not xi<=value<=xa:
            return

        conv, offset = self.get_position_conversion(), self.get_position_offset()

        # convert the value to enc value
        value = value / conv + offset

        # set the raw / encoder value
        self.set_encposition(value)

    def get_position(self):
        """
        Returns the encoded position (raw, no conversion, no offset)
        Returns:
        """
        res = None

        encvalue = self.get_encposition()

        if self.test_value(encvalue, type=float):
            conv, offset = self.get_position_conversion(), self.get_position_offset()

            # self.reprint("Conversion/Offset", "{} / {}".format(conv, offset))
            res = (encvalue - offset) * conv

        return res

    def _submit_command_ctrl(self, cmd):
        """
        Submits a command to the controller
        Args:
            cmd:

        Returns:
        """
        if len(cmd) == 0:
            return None

        # prepare the command
        cmd = cmd.replace("[", "{").replace("]", "}")

        res = self.ctrl.SocketWriteRead(cmd)
        self.reprint(h=cmd, m=res)

        return res

    def get_enabled(self):
        """
        Performs a test if the axis is enabled
        Returns:
        """
        cmd = self.cmd_enabled

        res = self._submit_command_ctrl(cmd)

        res = self._prep_value(res, bool)

        return res

    def _prep_value(self, value, type):
        """
        Prepares a value according to the type
        Args:
            value:
            type:

        Returns:
        """
        res = None
        try:
            if isinstance(value, str):
                value = value.strip().replace(":", "")
            if type == bool:
                res = int(value)
                if res == 0:
                    res = False
                else:
                    res = True
            else:
                res = type(value)
        except ValueError:
            self.error("Value conversion ({}->{}) failed".format(value, type))

        return res

    @command(dtype_in=float, dtype_out=float)
    def Calibrate(self, value):
        """
        Function calibrating the motor to a certain value by changing the offset
        Args:
            value:

        Returns:
        """
        res = 1

        # get the current encoded value
        encvalue = self.get_encposition()
        conv = self.get_position_conversion()

        # self.reprint("Enc / Value / Conv", "{} / {} / {}".format( encvalue, value, conv))

        # need to calculate offset
        offset = encvalue - value / conv
        # self.reprint("Old offset", "{}".format( self.get_position_offset() ) )
        self.set_position_offset(offset)

        #self.reprint("Changing offset / Enc / Value / Conv", "{} / {} / {} / {}".format(offset, encvalue, value, conv))

        #self.reprint("properties", self.PositionOffset.get_properties().label )

        return offset

    @command(dtype_in=None, dtype_out=None)
    def Stop(self):
        """
        Command stops the motor
        Returns:

        """
        self._submit_command_ctrl(self.cmd_write_stop)

    @command(dtype_in=None, dtype_out=None)
    def  StopMove(self):
        """
        Alias for the command stopping the motor
        Returns:
        """
        self.Stop()

    @command(dtype_in=None, dtype_out=str)
    def Test(self):
        """
        Reports simple information from the system
        Returns:

        """
        res = self._submit_command_ctrl(self.cmd_write_test)
        return res

    @command(dtype_in=None, dtype_out=int)
    def CheckMove(self):
        """
        Returns information on moving
        Returns:

        """
        res = self._submit_command_ctrl(self.cmd_move_state)
        res = self._prep_value(res, int)

        return res

    @command(dtype_in=None, dtype_out=None)
    def MoveHome(self):
        """
        Start moving the motor to the home position
        Returns:

        """
        self._submit_command_ctrl(self.CMD_HOME)

if __name__ == "__main__":
        server_run((A811ctrl, A811Channel))
