"""!
@file main.py
    This file contains a demonstration program that runs some tasks, an
    inter-task shared variable, and a queue. The tasks don't really @b do
    anything; the example just shows how these elements are created and run.

@author JR Ridgely
@date   2021-Dec-15 JRR Created from the remains of previous example
@copyright (c) 2015-2021 by JR Ridgely and released under the GNU
    Public License, Version 2. 
"""

import gc
import pyb
import cotask
import task_share
import encoder_reader
import MotorDriver
import closed_loop


def task1_fun(shares):
    """!
    Task which puts things into a share and a queue.
    @param shares A list holding the share and queue used by this task
    """
    # Get references to the share and queue which have been passed to this task
    enc = encoder_reader.Encoder(pyb.Pin.board.PC6, pyb.Pin.board.PC7, pyb.Timer(8, prescaler=0, period=65535))
    moe = MotorDriver.MotorDriver(pyb.Pin.board.PC1, pyb.Pin.board.PA0, pyb.Pin.board.PA1, pyb.Timer(5, freq=20000))

    enc.zero()
    close = closed_loop.ClosedLoop(0, .5)
    output = close.run(1024, enc.read())

    while(output != "End"):
        output = close.run(1024, enc.read())
        moe.set_duty_cycle(output)
        
        yield 0
    close.print_values()


def task2_fun(shares):
    """!
    Task which takes things out of a queue and share and displays them.
    @param shares A tuple of a share and queue from which this task gets data
    """
    # Get references to the share and queue which have been passed to this task
    enc = encoder_reader.Encoder(pyb.Pin.board.PB6, pyb.Pin.board.PB7, pyb.Timer(4, prescaler=0, period=65535))
    moe = MotorDriver.MotorDriver(pyb.Pin.board.PA10, pyb.Pin.board.PB4, pyb.Pin.board.PB5, pyb.Timer(3, freq=20000))

    enc.zero()
    close = closed_loop.ClosedLoop(0, .5)
    output = close.run(1024, enc.read())

    while(output != "End"):
        output = close.run(2048, enc.read())
        moe.set_duty_cycle(output)
        
        yield 0
    close.print_values()


# This code creates a share, a queue, and two tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops and
# printouts show diagnostic information about the tasks, share, and queue.
if __name__ == "__main__":
    print("Testing ME405 stuff in cotask.py and task_share.py\r\n"
          "Press Ctrl-C to stop and show diagnostics.")

    # Create a share and a queue to test function and diagnostic printouts
    share0 = task_share.Share('h', thread_protect=False, name="Share 0")
    q0 = task_share.Queue('L', 16, thread_protect=False, overwrite=False,
                          name="Queue 0")

    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for 
    # debugging and set trace to False when it's not needed
    task1 = cotask.Task(task1_fun, name="Task_1", priority=1, period=100,
                        profile=True, trace=False, shares=(share0, q0))
    task2 = cotask.Task(task2_fun, name="Task_2", priority=2, period=100,
                        profile=True, trace=False, shares=(share0, q0))
    cotask.task_list.append(task1)
    cotask.task_list.append(task2)

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()

    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break

    # Print a table of task data and a table of shared information data
    print('\n' + str (cotask.task_list))
    print(task_share.show_all())
    print(task1.get_trace())
    print('')
