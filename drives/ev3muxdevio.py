# SPDX-License-Identifier: MIT
# Copyright (c) 2017 Laurens Valk

"""Read and write to EV3 sensors through ev3dev sysfs."""
from os import listdir#, path
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import Ev3devSensor
import io


def read_int(infile):
    """Read an integer from a previously opened file descriptor."""
    infile.seek(0)
    return(int(infile.read().decode().strip()))


def write_int(outfile, value):
    """Write an integer to a previously opened file descriptor."""
    outfile.write(str(int(value)))
    outfile.flush()


def read_str(infile):
    """Read a string from a previously opened file descriptor."""
    infile.seek(0)
    return(infile.read().decode().strip())


def write_str(outfile, value):
    """Write a string to a previously opened file descriptor."""
    outfile.write(value)
    outfile.flush()


def get_sensor_path(port, mux, mode, driver_name):
    """Get a path to a device based on port number."""
    
    print(" ")
    print("Finding " + driver_name + "device at EV3 port " + str(port) + ", mux port " + str(mux))
    print(" ")
    
    base_dir = '/sys/class/lego-sensor'
    base_port_dir = '/sys/class/lego-port'
    port_full_dir = " "
    mux_found = False
    mux_dir = " "

    # check base EV3 port mode and set to auto if needed
    print("Checking mode of port " + str(port) + " = 'auto'")
    with open(base_port_dir + "/port" + str(port-1) + "/mode", 'r') as base_mode_read:
        base_mode_read_found = base_mode_read.read().strip('\n')
        if base_mode_read_found != "auto" :
            print("Current Mode: " + base_mode_read_found)
            print("Setting Mode to: auto")
            with open(base_port_dir + "/port" + str(port-1) + "/mode", 'w') as base_mode_write:
                base_mode_write.write("auto")
            wait(3000)
        else:
            print("Mode matched 'auto'")
    print(" ")

    # Find the port number that matches the address of the corresponding mux port
    print("Searching for the port that matches the address for mux " + str(mux))
    # Iterate through ['port0', 'port1', 'port2', ...]
    for port_dir in listdir(base_port_dir):
        #print("Port: " + str(port_dir))
        with open(base_port_dir + '/' + port_dir + '/address', 'r') as port_addr_file:
            port_addr_found = port_addr_file.read().strip('\n')
            #print("Port Addr: " + str(port_addr_found))
            if ('in' + str(port) in port_addr_found) and ('mux' + str(mux) in port_addr_found):
                # save the port address for later
                dev_port_addr = port_addr_found
                # Make the full path
                port_full_dir = base_port_dir + '/' + port_dir + '/'
                print("Port: " + str(port_dir))
                print("Port Addr: " + str(port_addr_found))
                print("Port path: " + port_full_dir)
                # found it so stop searching...
                break
            #print (" ")
            #port_addr_file.close()

    # If no mux device port found, exit
    if port_full_dir == " ":
        raise OSError("Mux device not found on EV3 port " + str(port))   
    else:
        print("EV3 Port Matched - mux connected to port " + str(port))

    print(" ")
    
    # verify the mode on this port is set correctly (analog or uart)
    # if it's wrong, fix it
    print("Checking mode on " + str(port_dir))
    with open(port_full_dir + 'mode', 'r') as mode_file_read:
        mode_file_read_found = mode_file_read.read().strip('\n')
        if mode_file_read_found != mode :
            print("Current Mode: " + mode_file_read_found)
            print("New Mode:" + mode)
            with open(port_full_dir + 'mode', 'w') as mode_file_write:
                mode_file_write.write(mode)
            wait(2000)  # on my EV3 I had to wait a signifcant amount of time for the files to create
        else:
            print("Mode matched - " + mode)

    print(" ")
    
    # find the sensor name for a device with an address that matches the mux port
    # then verify that the driver is set to what we want for this device type
    # if all is good, return the path, which ends this function
    print("Now searching for the device")
    # Iterate through ['sensor0', 'sensor1', 'sensor2', ...]
    for device_dir in listdir(base_dir):
        #print("Device Dir: " + str(device_dir))
        # In each folder, open the address file
        with open(base_dir + '/' + device_dir + '/address', 'r') as addr_file:
            # Read the port string (e.g. 'outB')
            addr_found = addr_file.read().strip('\n')
            #print("Dev Addr: " + str(addr_found))
            # If the port name matches, we are done searching and
            # we know the full path
            if ('in' + str(port) in addr_found) and ('mux' + str(mux) in addr_found) :
                # Make the full path
                full_dir = base_dir + '/' + device_dir + '/'
                print("Device: " + str(device_dir))
                print("Dev Addr: " + str(addr_found))
                print("Dev Path: " + full_dir)                
                with open(full_dir + 'driver_name', 'r') as driver_file:
                    if driver_name in driver_file.read():
                        print("Driver matches - returning path.")
                        print (" ")  
                        return full_dir   
                    else:
                        print("Driver name mismatch.")
                        break
        #print (" ")

    # didn't find a match, so we need to reset the driver for this port
    # this will create a new sensor address
    print("Setting the driver to " + driver_name)
    with open(port_full_dir + 'set_device', 'w') as set_device_write:
        set_device_write.write(driver_name) 
    wait(2000)  # on my EV3 I had to wait a signifcant amount of time for the files to create

    print (" ")
    print("Now searching for the device - again...")

    # find the sensor name for a device with an address that matches the mux port
    # then verify that the driver is set to what we want for this device type
    # if all is good, return the path, which ends this function
    # this time, if it fails, will raise a system error
    # Iterate through ['sensor0', 'sensor1', 'sensor2', ...]
    for device_dir in listdir(base_dir):
        print("Device Dir: " + str(device_dir))
        # In each folder, open the address file
        with open(base_dir + '/' + device_dir + '/address', 'r') as addr_file:
            # Read the port string (e.g. 'outB')
            addr_found = addr_file.read().strip('\n')
            print("Addr Found: " + str(addr_found))
            # If the port name matches, we are done searching and
            # we know the full path
            if ('in' + str(port) in addr_found) and ('mux' + str(mux) in addr_found) :
                # Make the full path
                full_dir = base_dir + '/' + device_dir + '/'
                print("Port Matched! Path: " + full_dir)                
                with open(full_dir + 'driver_name', 'r') as driver_file:
                    if driver_name in driver_file.read():
                        print("Driver matches - returning path.")
                        print (" ")  
                        return full_dir   
                    else:
                        print("Driver name still a mismatch - there's a problem...")
        print (" ")

    raise OSError('No such sensor on Port S' + str(port) + ", mux port " + str(mux))


class Ev3devMuxSensor():
    """Base class for ev3dev sensors operating through sysfs."""

    _ev3dev_driver_name = 'none'
    _mode = "none"
    _number_of_values = 1
    _default_mode = None

    def __init__(self, port, mux):
        """Initialize the sensor."""
        assert ord('1') <= ord(str(port)) <= ord('4')
        assert ord('1') <= ord(str(mux)) <= ord('3')
        self.port = port
        self.mux = mux
        self._open_files()
        if self._default_mode:
            self._mode = self._default_mode

    def _open_files(self):
        """Open the sysfs files for this device."""
        self.path = get_sensor_path(self.port, self.mux, self._mode, self._ev3dev_driver_name)
        self.mode_file = open(self.path + 'mode', 'r+b')
        self.mode_now = read_str(self.mode_file)
        self.value_files = [open(self.path + 'value' + str(num), 'rb') for num in range(self._number_of_values)]

    def _close_files(self):
        """Close the sysfs files for this device."""
        self.mode_file.close()
        for value_file in self.value_files:
            value_file.close()

    def _mode(self, mode_new):
        """Set the sensor mode. Not available to user."""
        if mode_new != self.mode_now:
            write_str(self.mode_file, mode_new)
            self.mode_now = mode_new

    def _value(self, num):
        """Return value in sensor/valueX. Not available to user."""
        return read_int(self.value_files[num])


class Ev3devMuxUartSensor(Ev3devMuxSensor):
    """UART ev3dev sensor operating through sysfs."""

    def _reset_port(self):
        path = "/sys/class/lego-port/port" + str(self.port) + "/"
        with open(path + 'mode', 'w') as rf:
            rf.write('auto')
        with open(path + 'mode', 'w') as rf:
            rf.write('ev3-uart')
        watch = StopWatch()
        while True:
            with open(path + 'status', 'r') as sf:
                status = sf.read().strip()
                if status != "no-sensor":
                    break
            wait(100)
            if watch.time() > 5000:
                raise OSError("Unable to reset sensor.")

    def _reset(self):
        """Force sensor to reset as if disconnecting and reconnecting it."""
        # First, close all files for this sensor
        try:
            self._close_files()
        except ValueError as err:
            pass
        # Reset the sensor
        self._reset_port()
        total = StopWatch()
        sub = StopWatch()
        # Wait for sensor to come back
        wait(2000)
        success = False
        while total.time() < 15000:
            wait(250)
            if sub.time() > 5000:
                self._reset_port()
                sub.reset()
            try:
                self._open_files()
                success = True
            except OSError:
                continue
            break

        if not success:
            raise OSError("Unable to reset sensor.")

        if self._default_mode:
            self._mode(self._default_mode)