import argparse
import atexit
import serial
import struct
import numpy as np
from enum import Enum
import time
import pandas as pd
from struct import *
pd.options.plotting.backend = "plotly"


class ErrorFunction(Enum):
    STOP = b'\x00'
    PYTHON_TORQUE = b'\x0b'
    NO_ERROR = b'\x01'
    ERROR1 = b'\x02'
    ERROR2 = b'\x03'
    ERROR3 = b'\x04'
    ERROR4 = b'\x05'
    ERROR5 = b'\x06'
    ERROR6 = b'\x07'
    ERROR7 = b'\x08'
    ERROR8 = b'\x09'
    ERROR9 = b'\x0a'


receive_data_format = "<LfhhhhhbbBc"
receive_data_columns = ["k", "Rescaled Position", "Torque", "Position", "Error", "Error Integral", "Error Derivative",
                        "Controller", "Disturbance", "Version"]

receive_data_size = calcsize(receive_data_format)

send_data_format = "<hcb"


def run_experiment(**kwargs):
    # set up the serial line
    ser = serial.Serial(kwargs['device'], 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS, timeout=0.1)
    if not ser.isOpen():
        ser.open()
    print('Port is open', ser.isOpen())
    time.sleep(2)

    def stop():
        if ser.is_open:
            ser.write(pack(send_data_format, 0, ErrorFunction.STOP.value, 0))
            ser.close()
    atexit.register(stop)

    # Read and record the data
    duration = int(kwargs['duration'])
    duration_ns = np.timedelta64(duration, 's').astype('timedelta64[ns]')
    data=np.zeros((duration*200, len(receive_data_columns)),dtype=float) #change for number of inputs
    clock=np.zeros(duration*200,dtype='timedelta64[ns]')
    i=0
    start = time.monotonic_ns()
    ser.write(pack(send_data_format, kwargs['position'], ErrorFunction[kwargs['mode']].value, kwargs['torque']))
    time.sleep(0.05)
    ser.reset_input_buffer()
    time.sleep(0.05)
    # throw away first 20 readings since there are hardware serial buffers inbetween
    for j in range(20):
        b = ser.read(receive_data_size)
    stop_byte = b[-1]
    while stop_byte != b'\xfa':
        stop_byte = ser.read(1)
        if np.timedelta64(time.monotonic_ns()-start, 'ns') > np.timedelta64(1, 's'):
            raise ValueError("You might have flashed the arduino with the wrong library version")
    while clock[i-1] < duration_ns and i < data.shape[0]:
        b = ser.read(receive_data_size)
        recv_data = struct.unpack(receive_data_format, b)
        if recv_data[-1] != b'\xfa':
            raise ValueError("Data received from Arduino is inconsistent, try rerunning the experiment, restarting the "
                             "arduino or reflashing it")
        data[i, :] = recv_data[:-1]
        clock[i]=np.timedelta64(time.monotonic_ns()-start, 'ns')
        i+=1

    ser.write(pack(send_data_format,0, ErrorFunction.STOP.value, 0))
    ser.close()
    atexit.unregister(stop)

    # plot the data
    index=pd.TimedeltaIndex(clock[:i])
    df=pd.DataFrame(data[:i], index=index.total_seconds(), columns=receive_data_columns)
    # sanity checks
    diff = df['k'].diff().iloc[1:]
    if (0 > diff).any() or (df['Version'].diff().iloc[1:] != 0).any():
        raise ValueError("Data received from Arduino is inconsistent, try rerunning the experiment, restarting the "
                         "arduino or reflashing it")
    if (df['Version'] != 2).any():
        raise ValueError("Arduino is configured with wrong version of the project library")
    df.drop('Version', axis=1, inplace=True)
    df["Position Derivative"] = df['Position'].diff()
    average_pos_diff = sum([der for der in df['Position Derivative'] if np.abs(der) < 100]) / len([der for der in df['Position Derivative'] if np.abs(der) < 100])
    print(f"Average Deriv::{average_pos_diff} for torque ::{kwargs['torque']}")

    df.index.name = "Experiment-Time in seconds"
    fig = df.plot(kind='line', line_shape='hv')
    fig.show()
    return


def limited_torque(value):
    value = int(value)
    if not -90 <= value <= 90:
        raise argparse.ArgumentTypeError(f"Torque must be between -90 and 90")
    return value

def int16(value):
    value = int(value)
    if not -32768 <= value <= 32767:
        raise argparse.ArgumentTypeError(f"Position must be between -128 and 127")
    return value


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run experiments for the first phase of the semester project WS24/25.')
    parser.add_argument('mode', type=str, choices=list(e.name for e in ErrorFunction
                                                       if e is not ErrorFunction.STOP),
                        help='Set the operating mode for the experiment')
    parser.add_argument("--device", type=str, required=True, help="COM-Port of the arduino")
    parser.add_argument("--duration", type=int, default=10, help="Duration of the experiment")
    parser.add_argument('--torque', default=0, type=limited_torque,
                        help='set a torque value that overwrites the value calculated by your controller in C code')
    parser.add_argument('--position', type=int16, default=0,
                        help='set a desired position that is passed to your implementation of the calculate_error '
                             'function in C')
    args = parser.parse_args()
    run_experiment(**vars(args))
