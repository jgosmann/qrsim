from __future__ import division
from math import modf
import qrsim.qrs_srv_cli_msg_pb2 as qrsim_proto
import socket


def _map_to_list(target, indexing):
    """Creates a property which is mapped to elements in a list with a getter
    and a setter.

    Example::

        class ListProperties(object):
            def __init__(self):
                self.values = range(0, 4)

            # property mapped to self.values[1]
            second_element = _map_to_list('values', 1)

            # property mapped to self.values[2:4]
            last_two_elements = _map_to_list('values', slice(2, 4))
    """

    def get(self):
        return getattr(self, target)[indexing]

    def set(self, value):
        getattr(self, target)[indexing] = value

    return property(get, set)


class UAVState(object):
    size = 13
    """Size of the state vector."""

    def __init__(self, state=size * [0]):
        """
        :param state: Initial state vector [:attr:`x`, :attr:`y`,
            :attr:`z`, :attr:`phi`, :attr:`theta`, :attr:`psi`, :attr:`u`,
            :attr:`v`, :attr:`w`, :attr:`p`, :attr:`q`, :attr:`r`,
            :attr:`thrust`] to assign.
        :type state: sequence of length :attr:`size`
        """
        assert len(state) == self.size, 'Invalid state vector length.'
        self.__state = state

    position = _map_to_list('__state', slice(0, 3))
    """[:attr:`x`, :attr:`y`, :attr:`z`] position [m] (NED coordinates)"""

    x = _map_to_list('__state', 0)
    """x position [m] (NED coordinate)"""

    y = _map_to_list('__state', 1)
    """y position [m] (NED coordinate)"""

    z = _map_to_list('__state', 2)
    """z position [m] (NED coordinate)"""

    attitude = _map_to_list('__state', slice(3, 6))
    """[:attr:`phi`, :attr:`theta`, :attr:`psi`] attitude in Euler angles
    [rad] right-hand ZYX convention"""

    phi = _map_to_list('__state', 3)
    """phi angle of attitude in Euler angles [rad] right-hand ZYX convention"""

    theta = _map_to_list('__state', 4)
    """theta angle of attitude in Euler angles [rad] right-hand ZYX
    convention"""

    psi = _map_to_list('__state', 5)
    """psi angle of attitude in Euler angles [rad] right-hand ZYX convention"""

    velocity = _map_to_list('__state', slice(6, 9))
    """[:attr:`u`, :attr:`v`, :attr:`w`] velocity [m/s] in body coordinates"""

    u = _map_to_list('__state', 6)
    """u velocity [m/s] in body coordinates"""

    v = _map_to_list('__state', 7)
    """v velocity [m/s] in body coordinates"""

    w = _map_to_list('__state', 8)
    """w velocity [m/s] in body coordinates"""

    rotational_velocity = _map_to_list('__state', slice(10, 12))
    """[:attr:`p`, :attr:`q`, :attr:`r`] rotational velocity [rad/s] in body
    coordinates"""

    p = _map_to_list('__state', 9)
    """p rotational velocity [rad/s] in body coordinates"""

    q = _map_to_list('__state', 10)
    """q rotational velocity [rad/s] in body coordinates"""

    r = _map_to_list('__state', 11)
    """r rotational velocity [rad/s] in body coordinates"""

    thrust = _map_to_list('__state', 12)
    """rotors thrust [N]"""

    state = property(lambda self: self.__state)
    """Current state vector. (read-only)"""

    def __repr__(self):
        return '%s(%s)' % (type(self).__name__, str(self.state))


class NoisyUAVState(object):
    size = 20
    """Size of the state vector."""

    def __init__(self, state=size * [0]):
        """
        :param state: Initial state vector [:attr:`x`, :attr:`y`,
            :attr:`z`, :attr:`phi`, :attr:`theta`, :attr:`psi`, 0, 0, 0,
            :attr:`p`, :attr:`q`, :attr:`r`, 0, :attr:`ax`, :attr:`ay`,
            :attr:`az`, :attr:`h`, :attr:`pxdot`, :attr:`pydot`,
            :attr:`hdot`] to assign.
        :type state: sequence of length :attr:`size`
        """
        assert len(state) == self.size, 'Invalid state vector length.'
        self.__state = state

    position = _map_to_list('__state', slice(0, 3))
    """[:attr:`x`, :attr:`y`, :attr:`z`] position [m] estimated by GPS (NED
    coordinates)"""

    x = _map_to_list('__state', 0)
    """x position [m] estimated by GPS (NED coordinate)"""

    y = _map_to_list('__state', 1)
    """y position [m] estimated by GPS (NED coordinate)"""

    z = _map_to_list('__state', 2)
    """z position [m] estimated by GPS (NED coordinate)"""

    attitude = _map_to_list('__state', slice(3, 6))
    """Estimated [:attr:`phi`, :attr:`theta`, :attr:`psi`] attitude in Euler
    angles [rad] right-hand ZYX convention"""

    phi = _map_to_list('__state', 3)
    """Estimated phi angle of attitude in Euler angles [rad] right-hand ZYX
    convention"""

    theta = _map_to_list('__state', 4)
    """Estimated theta angle of attitude in Euler angles [rad] right-hand ZYX
    convention"""

    psi = _map_to_list('__state', 5)
    """Estimated psi angle of attitude in Euler angles [rad] right-hand ZYX
    convention"""

    rotational_velocity = _map_to_list('__state', slice(10, 12))
    """Measured [:attr:`p`, :attr:`q`, :attr:`r`] rotational velocity [rad/s]
    in body coordinates"""

    p = _map_to_list('__state', 9)
    """Measured p rotational velocity [rad/s] in body coordinates"""

    q = _map_to_list('__state', 10)
    """Measured q rotational velocity [rad/s] in body coordinates"""

    r = _map_to_list('__state', 11)
    """Measured r rotational velocity [rad/s] in body coordinates"""

    acceleration = _map_to_list('__state', slice(13, 16))
    """Measured [:attr:`ax`, :attr:`ay`, :attr:`az`] acceleration [m/s^2]
    in body coordinates"""

    ax = _map_to_list('__state', 13)
    """Measured x acceleration [m/s^2] in body coordinates"""

    ay = _map_to_list('__state', 14)
    """Measured y acceleration [m/s^2] in body coordinates"""

    az = _map_to_list('__state', 15)
    """Measured z acceleration [m/s^2] in body coordinates"""

    altitude = _map_to_list('__state', 16)
    """Estimated h altitutde from altimeter NED, **posititve up!**"""

    gps_velocity = _map_to_list('__state', slice(17, 20))
    """[:attr:`pxdot`, :attr:`pxdot`, :attr:`hdot`] velocity [m/s] from GPS
    and altimeter (NED coordinates)"""

    pxdot = _map_to_list('__state', 17)
    """x velocity from GPS [m/s] (NED coordinates)"""

    pydot = _map_to_list('__state', 18)
    """y velocity from GPS [m/s] (NED coordinates)"""

    hdot = _map_to_list('__state', 19)
    """altitude rate [m/s] from altimeter (NED coordinates)"""

    state = property(lambda self: self.__state)
    """Current state vector. (read-only)"""

    def __repr__(self):
        return '%s(%s)' % (type(self).__name__, str(self.state))


class ServerSideError(Exception):
    """Raised when the server reports an error."""
    pass


class ProtocolError(Exception):
    """Raised when the server sends an unexpected message."""

    def __init__(self, msg, expected):
        """
        :param msg: Received message.
        :param expected: Expected message type.
        """
        self.msg = msg
        self.expected = expected

    def __str__(self):
        return 'Expected message of type %i, but got type %i.' % (
            self.msg.type, self.expected)

    def __repr__(self):
        return '%s(<msg of type %i>, %i)' % (
            type(self).__name__, self.msg.type, self.expected)


class TCPClient(object):
    TOL = 1e-6
    """Tolerance used to compare times."""

    def __init__(self):
        self.__initialized = False  # FIXME reset on discon
        self.__socket_open = False  # FIXME reset on discon
        self.__size_msg = qrsim_proto.Size()
        self.__size_msg.value = 1
        self.__size_msg_size = self.__size_msg.ByteSize()
        self._t = 0
        self._timestep = 0
        self._numUAVs = 0
        self._state = tuple()
        self._noisy_state = tuple()

    timestep = property(lambda self: self._timestep)
    """Simulation timestep [s]"""

    numUAVs = property(lambda self: self._numUAVs)
    """Number of UAVs"""

    state = property(lambda self: self._state)
    """Noiseless state of the UAVs (sequence of :class:`UAVState`)"""

    noisy_state = property(lambda self: self._state)
    """Noisy state of the UAVs (sequence of :class:`NoisyUAVState`)"""

    t = property(lambda self: self._t)
    """Current simulation time [s]"""

    def _needs_open_socket(self):
        assert self.__socket_open, 'Socket has to be opened first.'

    def _needs_initialization(self):
        self._needs_open_socket()
        assert self.__initialized

    def connect_to(self, ip, port):
        """Connects to a QRSim server.

        :param str ip: IP to connect to.
        :param int port: Port to connect to.
        """
        timeout = 20
        self.__sockfd = socket.create_connection((ip, port), timeout)
        self.__socket_open = True

    def init(self, task, realtime=False):
        """Sends to the server the command ot initialize the simulator.

        :param str task: Filename of the task file to load.
        :param boolean realtime: If ``True`` the simulator will run no faster
            than real-time, otherwise as fast as possible.
        """
        self._needs_open_socket()
        msg = qrsim_proto.Message()
        msg.type = qrsim_proto.Message.INIT
        msg.init.task = task
        msg.init.realTime = realtime
        self._send(msg)
        self._receive_state()
        self._receive_info()
        self.__initialized = True

    def reset(self):
        """Sends a reset command to the server.

        This will cause the simulator to set itself to the initial state
        defined in the task.
        """
        self._needs_initialization()
        msg = qrsim_proto.Message()
        msg.type = qrsim_proto.Message.RESET
        msg.reset.value = True
        self._send(msg)
        self._receive_ack()

    def disconnect(self, quit=False):
        """Disconnects from the server.

        :param boolean quit: If ``True`` the simulator will be turned off,
            otherwise it will keep running.
        """
        self._needs_initialization
        msg = qrsim_proto.Message()
        msg.type = qrsim_proto.Message.DISCONNECT
        msg.disconnect.quit = quit
        self._send(msg)
        self._receive_ack()

    def quit(self):
        """Turns off the simulator.

        This will obviously produce a disconnect.
        """
        self.disconnect(quit=True)

    def set_state(self, X):
        """Sets the UAVs noiseless states.

        :param X: State to set for each UAV. The length of the list has to
            match the number of UAVs or the command will fail.
        :type X: sequence of :class:`UAVState` of length :attr:`numUAVs`

        This function will update :attr:`state` right away. However,
        :attr:`noisy_state` will not updated before the simulator is advanced
        by another step.
        """
        self._needs_initialization()
        assert len(X) == self.numUAVs
        msg = qrsim_proto.Message()
        msg.type = qrsim_proto.Message.SETSTATE
        for x in X:
            state_msg = msg.X.add()
            state_msg.value = x.state
        self._send(msg)
        self._receive_ack()
        self._state = tuple(X)

    def step_wp(self, dt, WPs):
        """Steps forward the simulator given some waypoints.

        :param float dt: Amount of time [s] the simulator is stepped forward.
            Has to be a multiple of :attr:`timestep`.
        :param WPs: Waypoints to assign to the UAVs. This has to be a list with
            exactly one waypoint for each UAV. A single waypoint is a sequence
            consisting of (`wx`, `wy`, `wz`, `wpsi`) with `wx`, `wy`, `wz`
            being the position of the waypoint [m] (NED coordinates) and `wpsi`
            being the heading at the waypoint [rad].
        :type WPs: sequence of :attr:`numUAVs` sequences with 4 floats each

        This command will fail if the number of waypoints does not match the
        number UAVs or the dimension of a waypoint is not 4.
        """
        assert len(WPs) == self.numUAVs and all(len(wp) == 4 for wp in WPs)
        self._step(qrsim_proto.Step.WP, dt, WPs)

    def step_ctrl(self, dt, ctrls):
        """Steps forward the simulator given some control inputs.

        :param float dt: Amount of time [s] the simulator is stepped forward.
            Has to be a multiple of :attr:`timestep`.
        :param ctrls: Control inputs to assign to the UAVs. This has to be a
            list with exactly one control input for each UAV. A single control
            input is a sequence consisting of (`pt`, `rl`, `th`, `ya`, `bat`)
            with

            * commanded pitch `pt` [rad in -0.89..0.89],
            * commanded roll `rl` [rad in -0.89..0.89],
            * commanded throttle `th` [0..1],
            * commanded yaw velocity `ya` [rad/s in -4.4..4.4],
            * battery voltage `bat` [volts in 9..12].

        :type WPs: sequence of :attr:`numUAVs` sequences with 5 floats each

        This command will fail if the number of control inputs does not match
        the number UAVs or the dimension of a control input is not 5.
        """
        assert len(ctrls) == self.numUAVs and \
            all(len(ctrl) == 5 for ctrl in ctrls)
        self._step(qrsim_proto.Step.CTRL, dt, ctrls)

    def step_vel(self, dt, velocities):
        """Steps forward the simulator given some velocities.

        :param float dt: Amount of time [s] the simulator is stepped forward.
            Has to be a multiple of :attr:`timestep`.
        :param ctrls: Velocities to assign to the UAVs. This has to be a
            list with exactly one velocity vector for each UAV. A single
            velocity vector consists of 3 velocities [m/s] in body coordinates.
        :type WPs: sequence of :attr:`numUAVs` sequences with 3 floats each

        This command will fail if the number of velocity vectors does not match
        the number UAVs or the dimension of a velocity vector is not 3.
        """
        assert len(velocities) == self.numUAVs and \
            all(len(vel) == 3 for vel in velocities)
        self._step(qrsim_proto.Step.VEL, dt, velocities)

    def rpc(self, target, method):
        self._needs_initialization()
        msg = qrsim_proto.Message()
        msg.type = qrsim_proto.Message.RPC
        msg.rpc.target = getattr(qrsim_proto.Rpc, target)
        msg.rpc.method = method
        self._send(msg)
        return self._receive_return_value()

    def _step(self, type, dt, cmds):
        self._needs_initialization()
        f, unused = modf(dt / self.timestep)
        assert f < self.TOL or abs(1 - f) < self.TOL, \
            'The time increment dt must be a multiple of timestep.'

        msg = qrsim_proto.Message()
        msg.type = qrsim_proto.Message.STEP
        msg.step.dt = dt
        msg.step.type = type
        for single_uav_cmds in cmds:
            msg_cmd = msg.step.cmd.add()
            msg_cmd.value.extend(single_uav_cmds)
        self._send(msg)
        self._receive_state()

    def _receive_ack(self):
        msg = self._receive_msg()
        self._check_msg_type(msg, qrsim_proto.Message.ACK)
        self._check_for_server_side_error(msg)

    def _receive_state(self):
        msg = self._receive_msg()
        self._check_msg_type(msg, qrsim_proto.Message.STATE)
        self._t = msg.state.t
        self._state = tuple(UAVState(tuple(x.value)) for x in msg.state.X)
        self._noisy_state = tuple(
            NoisyUAVState(tuple(x.value)) for x in msg.state.eX)

    def _receive_return_value(self):
        msg = self._receive_msg()
        self._check_msg_type(msg, qrsim_proto.Message.RPC_RETURN_VALUE)
        return msg.return_value.value

    def _receive_info(self):
        msg = self._receive_msg()
        self._check_msg_type(msg, qrsim_proto.Message.TASKINFO)
        self._timestep = msg.taskInfo.timestep
        self._numUAVs = msg.taskInfo.numUAVs

    def _send(self, msg):
        self.__size_msg.value = msg.ByteSize()
        self.__sockfd.send(self.__size_msg.SerializeToString())
        self.__sockfd.send(msg.SerializeToString())

    def _receive_msg(self):
        self.__size_msg.ParseFromString(
            self.__sockfd.recv(self.__size_msg_size))

        msg = qrsim_proto.Message()
        msg.ParseFromString(self.__sockfd.recv(self.__size_msg.value))
        return msg

    @classmethod
    def _check_msg_type(cls, msg, expected):
        if msg.type != expected:
            cls._check_for_server_side_error(msg)
            raise ProtocolError(msg, expected)

    @classmethod
    def _check_for_server_side_error(cls, msg):
        if msg.type == qrsim_proto.Message.ACK and msg.ack.error:
            raise ServerSideError(msg.ack.msg)
