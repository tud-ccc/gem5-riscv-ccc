'''
Full system script
'''

import argparse
import fssystem

import m5
from m5.objects import *

m5.util.addToPath('../..')
from common import MemConfig

cpu_types = {
    'atomic': AtomicSimpleCPU,
    'timing': TimingSimpleCPU,
    'minor': MinorCPU
}


def create(args):
    '''Create the system and configure it'''
    cpu_class = cpu_types[args.cpu]
    mem_mode = cpu_class.memory_mode()

    system = fssystem.SimpleSystem(cpu_class=cpu_class,
                                   wfgdb=args.wait_for_gdb,
                                   mem_mode=mem_mode,
                                   bootloader=args.binary)

    # some required stuff
    mem_type = 'DDR3_1600_8x8'
    mem_channels = 1
    args.mem_type = mem_type
    args.mem_channels = mem_channels

    MemConfig.config_mem(args, system)

    system.connect()

    return system


def run():
    '''Run the simulation'''
    exit_event = m5.simulate()
    print('Exiting because %s @ %d ' % (exit_event.getCause(), m5.curTick()))


def main():
    parser = argparse.ArgumentParser(epilog=__doc__)

    parser.add_argument('-b',
                        '--binary',
                        type=str,
                        default=None,
                        required=True,
                        help='The binary to run')
    parser.add_argument('--cpu',
                        type=str,
                        default='atomic',
                        help='CPU model to use')
    parser.add_argument('-w',
                        '--wait-for-gdb',
                        action='store_true',
                        help='Wait for remote gdb connection '
                        'before starting simulation')

    args = parser.parse_args()

    root = Root(full_system=True)
    root.system = create(args)

    m5.instantiate()

    run()


if __name__ == '__m5_main__':
    main()
