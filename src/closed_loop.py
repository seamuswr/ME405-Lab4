"""!
@file closed_loop_controller.py
This file contains code for a closed-loop control system.

@Author ME405 team 8
@date 20-Feb-2024
"""

import cqueue
import utime


class ClosedLoop:
    """!
    Represents a closed-loop control system.
    This class implements a closed-loop control system with proportional control.
    """

    def __init__(self, setpoint, Kp):
        """!
        Initializes the ClosedLoop object with setpoint and proportional gain.

        @param setpoint (float): The desired value for the system to achieve.
        @param Kp (float): The proportional gain of the controller.
        """
        self.setpoint = setpoint
        self.Kp = Kp
        self.measured_output = 0
        self.time_queue = cqueue.IntQueue(20)
        self.pos_queue = cqueue.IntQueue(20)
        self.start_time = utime.ticks_ms()

    def run(self, setpoint, measured_output):
        """!
        Runs one iteration of the closed-loop control system.

        This method calculates the actuation value based on the current setpoint
        and measured output, and updates internal data structures with timestamps
        and measured positions.

        @param setpoint (float): The desired value for the system to achieve.
        @param measured_output (float): The current measured output of the system.

        @returns float or str: The actuation value if the queue is not full, otherwise "End".
        """
        self.setpoint = setpoint
        self.measured_output = measured_output
        error = self.setpoint - self.measured_output
        actuation_value = self.Kp * error
        if not self.time_queue.full():
            self.time_queue.put(utime.ticks_ms() - self.start_time)
            self.pos_queue.put(self.measured_output)
        else:
            return "End"
        return actuation_value

    def set_setpoint(self, setpoint):
        """!
        Sets a new setpoint for the control system.

        @param setpoint (float): The new desired value for the system to achieve.
        """
        self.setpoint = setpoint

    def set_Kp(self, Kp):
        """!
        Sets a new proportional gain for the controller.

        @param Kp (float): The new proportional gain value.
        """
        self.Kp = Kp

    def print_values(self):
        """!
        Prints the stored timestamps and measured positions.

        This method prints the contents of the time and position queues.
        """
        for idx in range(200):
            print(f'{self.time_queue.get()}, {self.pos_queue.get()}')
        print('END')


if __name__ == '__main__':
    """
    Entry point of the script.
    """

    # Initialize encoder and motor objects
    enc = encoder_reader.Encoder(pyb.Pin.board.PC6, pyb.Pin.board.PC7, pyb.Timer(8, prescaler=0, period=65535))
    moe = MotorDriver.MotorDriver(pyb.Pin.board.PC1, pyb.Pin.board.PA0, pyb.Pin.board.PA1, pyb.Timer(5, freq=20000))

    # Initialize closed-loop control object
    close = ClosedLoop(0, .1)

    # Main control loop
    while True:
        output = close.run(1000, enc.read())
        moe.set_duty_cycle(output)
