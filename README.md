# Tango Server-ACSA811

## Purpose
The purpose of the TangoServer is control of the _ACS controls_ A811 motor control system. The Tango server is implemented as multichannel (AxisID) parameter.
The Tango Server implements a TCP/IP connection with a socket.

## Notes
Since the controller operates in absolute encoder mode, for the simplification of the calibration, the encoder values are not calibrated, but the PositionOffset attribute is implemented.
It is memorized attribute, still, the issue with this attribute, that it is saved upon a direct write call, but local changes via, e.g. command are not memorized. Saving of the PositionOffset is done via __python.configparser__ functionality.

## TODO
- thread for the socket reconnection

## Implementation
Implemented using Python 3. Code successfully tested on Win10, Linux should work as well.
The following libraries are required.

		pip install numpy pytango