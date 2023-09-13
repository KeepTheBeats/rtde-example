"""
An example program to move the robotic arm and change digital outputs.
"""

import sys

sys.path.append("..")
import logging
import json
import time
from collections import namedtuple
from threading import Thread

# UR libraries API
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

# We start the motion variable to none
# before getting data from RTDE we need to have something even though it is empty
# Initial position
motion = [0, 0, 0, 0, 0, 0]

# logging.basicConfig(level=logging.INFO)
ROBOT_HOST = "127.0.0.1"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
actualq_names, actualq_types = conf.get_recipe("actualq")

sdo_names, sdo_types = conf.get_recipe("sdo")  # standard digital output
# watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()
print("Connected to RTDE")

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
actual_q = con.send_input_setup(actualq_names, actualq_types)

sdo = con.send_input_setup(sdo_names, sdo_types)  # standard digital output

# We apply the initial position to the joints
actual_q.input_double_register_24 = motion[0]
actual_q.input_double_register_25 = motion[1]
actual_q.input_double_register_26 = motion[2]
actual_q.input_double_register_27 = motion[3]
actual_q.input_double_register_28 = motion[4]
actual_q.input_double_register_29 = motion[5]

# standard digital output
sdo.standard_digital_output_mask = 255  # without this, we cannot change standard digital outputs
sdo.standard_digital_output = 0

# start data synchronization
if not con.send_start():
    sys.exit()

con.send(actual_q)

con.send(sdo)  # standard digital output

# Current rtde data of the arm
current_rtde = None

# Frequency rate for getting robotic arm rtde data
freq_RTDE = 1  # in this example, we get the current robotic arm rtde data once per second


# Read the robotic arm rtde data
def read_rtde():
    global current_rtde
    conf = rtde_config.ConfigFile("record_configuration.xml")
    output_names, output_types = conf.get_recipe("out")

    # get controller version
    con.get_controller_version()

    # setup recipes
    if not con.send_output_setup(
            output_names, output_types, frequency=freq_RTDE):
        logging.error("Unable to configure output")
        sys.exit()

    # start data synchronization
    if not con.send_start():
        logging.error("Unable to start synchronization")
        sys.exit()

    keep_running = True
    while keep_running:
        try:
            # Getting data from the joints from RTDE
            state = con.receive_buffered()

            # If freq_RTDE == 10, it means that in every second, state will be "not None" 10 times
            if state is not None:
                # We convert the data into json object
                data = json.dumps(state.__dict__)  # serialize data object
                # Converting JSON data into Python readable object
                current_rtde = json.loads(
                    data,
                    object_hook=lambda d: namedtuple('a', d.keys())
                    (*d.values()))
                logging.info(
                    "current position is {}, digital outputs is {}".format(
                        current_rtde.actual_q,
                        current_rtde.actual_digital_output_bits))

        except KeyboardInterrupt:
            keep_running = False
        except rtde.RTDEException:
            con.disconnect()
            sys.exit()

    con.send_pause()
    con.disconnect()


# move the robotic arm
def move_arm(motion):

    actual_q.input_double_register_24 = motion[0]
    actual_q.input_double_register_25 = motion[1]
    actual_q.input_double_register_26 = motion[2]
    actual_q.input_double_register_27 = motion[3]
    actual_q.input_double_register_28 = motion[4]
    actual_q.input_double_register_29 = motion[5]

    con.send(actual_q)
    logging.info(
        "sent command to move robotic arm to position {}".format(motion))


# change standard digital outputs
def change_sdo(value: int):
    sdo.standard_digital_output = value
    con.send(sdo)
    logging.info(
        "sent command to change standard digital outputs to {}".format(value))


def loop_move_arm():
    all_positions = []
    all_positions.append([0.0335, -1.8452, -2.0667, -0.7973, 1.61181, 0.12758])
    all_positions.append(
        [0.0335, -1.22470, -2.0667, -0.7973, 1.61181, 0.12758])
    all_positions.append(
        [1.73730, -1.22470, -2.0667, -0.7973, 1.61181, 0.12758])
    all_positions.append(
        [1.73730, -1.22470, -1.98374, -0.7973, 1.61181, 0.12758])
    all_positions.append(
        [1.73730, -1.22470, -1.98374, 0.07505, 1.61181, 0.12758])
    all_positions.append(
        [1.73852, -1.44234, -1.86890, 0.25936, 1.59314, 0.12741])
    all_positions.append(
        [1.73730, -1.22470, -1.98374, 0.07505, 1.61181, 0.12758])
    all_positions.append(
        [1.73730, -1.22470, -1.98374, -0.7973, 1.61181, 0.12758])
    all_positions.append(
        [1.73730, -1.22470, -2.0667, -0.7973, 1.61181, 0.12758])
    all_positions.append(
        [0.0335, -1.22470, -2.0667, -0.7973, 1.61181, 0.12758])
    all_positions.append([0.0335, -1.8452, -2.0667, -0.7973, 1.61181, 0.12758])

    while True:
        for pos in all_positions:
            move_arm(pos)
            time.sleep(1)

        for pos in reversed(all_positions):
            move_arm(pos)
            time.sleep(1)


def loop_change_sdo():
    """
    According to this page:
    https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/

    the "standard_digital_output" is uint8, the range is 0-255
    The UR robot arm has 8 standard digital outputs: 0-7
    
    If we set standard_digital_output as 160, it means that digital output 7 and 5 are True, and others are False, because the decimal 160 is 10100000.

    If we want to set digital output 0 and 1 as True and others as False, we should set standard_digital_output as 3, because decimal 3 is 00000011.

    if we want to set digital output 1 as True and others as False, we should set standard_digital_output as 2, because decimal 2 is 00000010.

    """
    all_values = []
    all_values.append(0)
    all_values.append(1)
    all_values.append(2)
    all_values.append(3)
    # all_values = [0]
    # a = 1
    # while a <= 255:
    #     all_values.append(a)
    #     a = a << 1

    while True:
        for value in all_values:
            change_sdo(value)
            time.sleep(0.75)

        for value in reversed(all_values):
            change_sdo(value)
            time.sleep(0.75)


def main():
    thread_read = Thread(target=read_rtde)
    thread_read.start()

    thread_move = Thread(target=loop_move_arm)
    thread_move.start()

    thread_sdo = Thread(target=loop_change_sdo)
    thread_sdo.start()

    while True:
        pass


if __name__ == "__main__":
    main()
