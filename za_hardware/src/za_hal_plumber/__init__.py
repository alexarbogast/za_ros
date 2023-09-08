import os
from enum import IntEnum
import rospy
from hal_hw_interface import loadrt_local, rtapi, hal

###########################################
# HALPlumberBase
#
# Base class for everything else; basically exists to provide a
# mechanism for adding HAL functions in the right order


class HALPlumberBase:
    class Prio(IntEnum):
        # Conventions for where to add update commands in thread
        LIST_HEAD = 0  # The head of the thread linked list
        DRIVE_READ_FB = 100  # Read fb from drive
        SAFETY_CHAIN = 300  # Check safety conditions
        FB_CHAIN = 400  # Process fb
        ROS_CONTROL = 500  # ros_control read(); update(); write()
        CMD_CHAIN = 700  # Process cmd
        DRIVE_WRITE_CMD = 900  # Write cmd to drive
        LIST_TAIL = 10000  # The tail of the thread linked list

    # A dict with prio:func_name mappings
    thread_functs = dict()

    def func_config(self, name, prio):
        if prio in self.thread_functs:
            raise RuntimeError(
                "Function '%s' already set with prio %d"
                % ((self.thread_functs[prio], prio))
            )
        self.thread_functs[prio] = name

    def add_funcs(self):
        # Call HAL addf for each function in correct thread order
        for prio in sorted(self.thread_functs.keys()):
            rospy.loginfo("Adding HAL function %s" % self.thread_functs[prio])
            hal.addf(self.thread_functs[prio], self.thread_name)


###########################################
# HALJointPlumber and subclasses
#
# This does the per-joint configuration; subclass for specific drives


class HALJointPlumber(HALPlumberBase):

    # ========== Feedback chain:
    #
    # - The drive_fb signal input comes from drive subclasses
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    # *_drive_fb            -> *_scale_pos_fb             -> *_ros_pos_fb
    # *_ros_pos_fb          -> hal_hw_interface.*.pos-fb
    #
    #
    # ========== Command chain:
    #
    # - The ros_pos_cmd signal output goes to drive subclasses
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    # (N/A)                    hal_hw_interface.*.pos-cmd -> *_ros_pos_cmd
    # *_ros_pos_cmd         -> *_jlimits                  -> *_jlimits-out
    # *_jlimits-out         -> *_scale_pos_cmd            -> *_drive_cmd
    #
    #
    # ========== Reset:
    #
    # - The hal_402_mgr sets the reset signal high from just before
    #   enabling the drives to just after.
    #
    # - The reset signal connects to the hal_hw_interface reset pin to
    #   set command to feedback.
    #
    # - The reset signal connects to the jlimits.load pin to set
    #   output to input.
    #
    # ========== Safety input chain
    #
    # signal                -> comp/pin                  -> signal
    # ------                   --------                     ------
    # (TBD)
    #

    #########################
    # Convenience functions to save repeatedly typing and reading
    # `"joint%i_foo" % self.num`
    def jname(self, suffix):
        return "joint%i_%s" % (self.num, suffix)

    def newinst(self, comp_name, suffix, *args, **kwargs):
        return rtapi.newinst(comp_name, self.jname(suffix), *args, **kwargs)

    def newsig(self, suffix, haltype):
        return hal.newsig(self.jname(suffix), haltype)

    def pin(self, suffix):
        return hal.Pin(self.jname(suffix))

    def link_pin(self, from_suffix, to_suffix):
        self.pin(from_suffix).link(self.jname(to_suffix))

    def signal(self, suffix):
        return hal.Signal(self.jname(suffix))

    def link_global_signal(self, signal, *pin_suffixes):
        if isinstance(signal, str):
            signal = hal.Signal(signal)
        for s in pin_suffixes:
            signal.link(self.jname(s))

    def link_signal(self, sig_suffix, *pin_suffixes):
        self.link_global_signal(self.signal(sig_suffix), *pin_suffixes)

    def set_pin(self, suffix, value):
        self.pin(suffix).set(value)

    def set_signal(self, suffix, value):
        self.signal(suffix).set(value)

    def get_pin(self, suffix):
        return self.pin(suffix).get()

    def func_config(self, base_name, base_prio):
        name = self.jname(base_name)
        prio = base_prio + (0.1 * self.num)
        super().func_config(name, prio)

    #########################
    # Set up HAL for a joint
    def __init__(self, thread_name, joint_name, config):
        # Set base object attributes
        self.thread_name = thread_name
        self.joint_name = joint_name
        for k, v in config.items():
            setattr(self, k, v)

    def setup_hal(self):
        # This represents the high-level work of per-joint HAL setup
        # of feedback and command pipeline comps, signals, settings
        #
        # Set up feedback and command pipeline comps, signals, settings
        self.init_plumbing()
        # - Connect drive-specific config
        self.connect_drive()
        self.init_max_vel_safety()
        # - Per-joint misc stuff
        # self.setup_jlimits()
        self.connect_hal_ros_control()
        # - Drive-specific final operations, optionally implemented in
        #   subclasses
        self.finish_config()

    cw_bits = [
        'switch_on',
        'enable_voltage',
        'quick_stop',
        'enable_operation',
        'mode_1',
        'mode_2',
        'mode_3',
        'fault_reset',
        'halt',
        # Bits 9-15 reserved
    ]

    def init_plumbing(self):
        # joint limits
        # self.newinst("limit3v3", "jlimits")
        # Safety input
        self.newsig("max_vel_safety", hal.HAL_FLOAT)

        self.newsig('ros_pos_cmd', hal.HAL_FLOAT)
        self.newsig('ros_vel_cmd', hal.HAL_FLOAT)
        self.newsig('ros_acc_cmd', hal.HAL_FLOAT)

        self.newsig('pos_cmd', hal.HAL_FLOAT)
        self.newsig('pos_fb', hal.HAL_FLOAT)
        self.newsig('pos_err', hal.HAL_FLOAT)
        self.newsig('vel_cmd', hal.HAL_FLOAT)
        self.newsig('vel_fb', hal.HAL_FLOAT)
        self.newsig('acc_cmd', hal.HAL_FLOAT)
        self.newsig('acc_fb', hal.HAL_FLOAT)
        self.newsig('torque_cmd', hal.HAL_FLOAT)
        self.newsig('torque_fb', hal.HAL_FLOAT)

        # Drive control:  Link hal_402_mgr pins to signals
        # - raw_control-word sig:  raw control-word
        cwr_sig = self.newsig('raw_control-word', hal.HAL_U32)
        cwr_sig.link(f"hal_402_mgr.drive_{self.num}.control-word")
        cwr_sig.link(f"drive_safety.raw-control-word{self.slavenum}")
        # - control-word sig:  raw control-word w/emergency stop override
        cw_sig = self.newsig('control-word', hal.HAL_U32)
        cw_sig.link(hal.Pin(f'drive_safety.control-word{self.slavenum}'))
        cw_sig.link(f"hal_402_mgr.drive_{self.num}.control-word-fb")
        # - status-word, drive-mode-cmd, drive-mode-fb, aux-error-code:
        #   link hal_402_mgr pins to signals
        for name in (
            'status-word',
            'drive-mode-cmd',
            'drive-mode-fb',
            'aux-error-code',
        ):
            sig = self.newsig(name, hal.HAL_U32)
            sig.link(f"hal_402_mgr.drive_{self.num}.{name}")
        for name in (
            'slave-online',
            'slave-oper',
            'slave-state-init',
            'slave-state-preop',
            'slave-state-safeop',
            'slave-state-op',
        ):
            sig = self.newsig(name, hal.HAL_BIT)
            sig.link(f"hal_402_mgr.drive_{self.num}.{name}")
        # - status-word:  link to drive_safety
        sw_sig = self.signal('status-word')
        sw_sig.link(f'drive_safety.status-word{self.slavenum}')
        # - control-word bits:  break out (safe) control word bits into signals
        self.newinst('bitslicev2', 'cw_bits', pincount=9)
        self.func_config("cw_bits.funct", self.Prio.SAFETY_CHAIN + 90)
        cw_sig.link(self.pin('cw_bits.in'))
        for idx, name in enumerate(self.cw_bits):
            s = self.newsig(f'cw_bit_{name}', hal.HAL_BIT)
            s.link(self.pin(f"cw_bits.out-{idx:02d}"))

    def init_max_vel_safety(self):
        # Set up joint max_vel_safety signal from the configured joint
        # max_vel * global max_vel_safety_scale signal.
        # max_vel_safety_scale is normally 1.0, and changes to
        # something closer to 0.0 when the safety_input goes low.
        self.newinst("mult2v2", "max_vel_safety")

        self.link_global_signal('max_vel_safety_scale', "max_vel_safety.in0")
        self.set_pin("max_vel_safety.in1", self.max_vel)
        self.link_signal("max_vel_safety", "max_vel_safety.out")

        self.func_config("max_vel_safety.funct", self.Prio.SAFETY_CHAIN + 20)

    def connect_drive(self):
        # Override in classes
        raise RuntimeError("Subclasses must implement connect_drive() method")

    def connect_hal_ros_control(self):
        # ROS commanded position
        self.signal('ros_pos_cmd').link(
            f"hal_hw_interface.{self.joint_name}.pos-cmd")

        # Joint feedback to ROS
        self.signal('pos_fb').link(
            f'hal_hw_interface.{self.joint_name}.pos-fb')
        self.signal('vel_fb').link(
            f'hal_hw_interface.{self.joint_name}.vel-fb')

        self.signal('torque_fb').link(
            f'hal_hw_interface.{self.joint_name}.eff-fb')

    def finish_config(self):
        pass


class HALJointPlumberEC(HALJointPlumber):
    # Quick stop chain:
    #
    # When any drive faults, force low QUICK STOP bit 2 of all drives
    # control word.  The hal_402_mgr is then responsible for stopping
    # enabled drives and waiting for the command to reset the fault.
    #
    # Control word quick stop can be high only if:
    # - no drives in fault AND
    # - hal_402_mgr control word quick stop bit is high
    #
    # Components:
    # - any_fault:  or, 6-inputs, SAFETY_CHAIN + 50
    # - any_fault_s32:  conv_bit_s32, SAFETY_CHAIN + 60
    # - control-word_mask:  muxn_u32, pincount=2, SAFETY_CHAIN + 70
    # - *-control-word_safe:  bitwise, SAFETY_CHAIN + 80
    # - *-cw_bits:  bitslice, SAFETY_CHAIN + 90
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    #                          lcec.0.*.fault -------------> *-fault
    # *-fault -------------+-> any_fault ------------------> any_fault
    # quick_stop ---------/
    # any_fault ----------\
    # 0xffff --------------+-> control-word_mask ----------> control-word_mask
    # 0xfffb -------------/
    #                          hal_402_mgr.*-control-word -> *-control-word
    # control-word_mask --+--> *-control-word_safe --------> *-control-word_safe
    # *-control-word ----<
    #                     \--> *-cw_bits ------------------> *-cw_bit_*
    # *-control-word      ---> lcec.0.*.control-word
    #
    #
    # Feedback chain:
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    # (N/A)                lcec.0.*.position-actual-value -> *position-actual-value
    # *position-actual-value -> *_fb_flt                  -> *_drive_fb
    #
    #
    # Command chain:
    #
    # signal                -> comp/pin                   -> signal
    # ------                   --------                      ------
    # *_ros_pos_cmd       -> *_cmd_s32                  -> *_cmd_s32-out
    # *_cmd_s32-out         -> lcec.0.*.target-position
    #
    #
    # 402 mgr chain:
    #
    # - Pins with names from lcec_to_402_mgr_pins on hal_402_mgr.drive* and
    #   lcec.0.* connected

    def connect_drive(self):
        # fb from drive
        self.newsig('drive_pos_fb', hal.HAL_FLOAT)
        # self.newsig('drive_vel_fb', hal.HAL_FLOAT)
        # self.newsig('drive_acc_fb', hal.HAL_FLOAT)
        # self.newsig('torque_loop_cmd', hal.HAL_FLOAT)
        # self.newsig('torque_cmd', hal.HAL_FLOAT)
        # self.newsig('load', hal.HAL_FLOAT)
        # self.newsig('inertia', hal.HAL_FLOAT)
        # self.newsig('friction', hal.HAL_FLOAT)
        # self.newsig('damping', hal.HAL_FLOAT)

        # calculate vel and acc cmd from pos cmd
        self.newinst("pll", "cmd_pll")
        self.func_config("cmd_pll.funct", self.Prio.CMD_CHAIN + 1)

        self.newinst("qc", "qc")
        self.func_config("qc.funct", self.Prio.CMD_CHAIN + 2)

        # load parameters from hardware_settings.yaml
        self.pin('cmd_pll.bandwidth').set(self.cmd_bandwidth)
        self.pin('cmd_pll.max-pos').set(self.max_pos)
        self.pin('cmd_pll.min-pos').set(self.min_pos)
        self.pin('cmd_pll.max-vel').set(self.max_vel * 20.0)
        self.pin('cmd_pll.max-acc').set(self.max_acc * 20.0)
        self.pin('cmd_pll.mode').set(2)

        self.signal("cw_bit_enable_operation").link(self.pin("cmd_pll.en"))
        self.signal("ros_pos_cmd").link(self.pin("cmd_pll.pos-in"))
        self.signal("ros_vel_cmd").link(self.pin("cmd_pll.vel-in"))
        self.signal("ros_acc_cmd").link(self.pin("cmd_pll.acc-in"))

        self.newsig("pos_c", hal.HAL_FLOAT)
        self.newsig("vel_c", hal.HAL_FLOAT)
        self.newsig("acc_c", hal.HAL_FLOAT)
        self.signal("pos_c").link(self.pin("cmd_pll.pos-out"))
        self.signal("vel_c").link(self.pin("cmd_pll.vel-out"))
        self.signal("acc_c").link(self.pin("cmd_pll.acc-out"))
        self.signal("pos_c").link(self.pin("qc.pos-in"))
        self.signal("vel_c").link(self.pin("qc.vel-in"))
        self.signal("acc_c").link(self.pin("qc.acc-in"))
        self.signal("pos_cmd").link(self.pin("qc.pos-out"))
        self.signal("pos_fb").link(self.pin("qc.pos-fb"))
        self.signal("vel_cmd").link(self.pin("qc.vel-out"))
        self.signal("vel_fb").link(self.pin("qc.vel-fb"))
        self.signal("acc_cmd").link(self.pin("qc.acc-out"))
        self.signal("acc_fb").link(self.pin("qc.acc-fb"))
        self.signal("torque_cmd").link(self.pin("qc.torque-ff"))
        self.signal("torque_fb").link(self.pin("qc.torque-fb"))
        hal.Signal("curr_periodf").link(self.pin("qc.periodf"))
        self.signal("cw_bit_enable_operation").link(self.pin("qc.enable"))
        self.pin('qc.max-pos').set(0.0)
        self.pin('qc.min-pos').set(0.0)
        self.pin('qc.max-vel').set(self.max_vel)
        self.pin('qc.max-acc').set(self.max_acc)

        # calculate vel and acc fb from pos fb
        self.newinst("pll", "fb_pll")
        self.func_config("fb_pll.funct", self.Prio.CMD_CHAIN + 3)

        # load parameters from hardware_settings.yaml
        self.pin('fb_pll.bandwidth').set(self.fb_bandwidth)
        self.pin('fb_pll.max-pos').set(-1.0)
        self.pin('fb_pll.min-pos').set(1.0)
        self.pin('fb_pll.max-vel').set(self.max_vel * 10)
        self.pin('fb_pll.max-acc').set(self.max_acc * 50)
        self.pin('fb_pll.mode').set(3)

        self.signal("cw_bit_enable_operation").link(self.pin("fb_pll.en"))
        self.signal("drive_pos_fb").link(self.pin("fb_pll.pos-in"))
        # self.signal("drive_vel_fb").link(self.pin("fb_pll.vel-in"))
        # self.signal("drive_acc_fb").link(self.pin("fb_pll.acc-in"))
        self.signal("pos_fb").link(self.pin("fb_pll.pos-out"))
        self.signal("vel_fb").link(self.pin("fb_pll.vel-out"))
        self.signal("acc_fb").link(self.pin("fb_pll.acc-out"))
        hal.Signal("curr_periodf").link(self.pin("fb_pll.periodf"))

        # joint feedforward parameter estimation
        # self.newinst("est", "est")
        # self.func_config("est.funct", self.Prio.CMD_CHAIN + 3)

        # self.pin('est.load').set(self.load)
        # self.pin('est.inertia').set(self.inertia)
        # self.pin('est.friction').set(self.friction)
        # self.pin('est.damping').set(self.damping)
        # self.pin('est.load-in').set(self.load)
        # self.pin('est.inertia-in').set(self.inertia)
        # self.pin('est.friction-in').set(self.friction)
        # self.pin('est.damping-in').set(self.damping)
        # self.pin('est.clear').set(self.online_estimation <= 0.0)
        # self.pin('est.max-delta').set(0.25)  # allow +-25% estimation
        # self.pin('est.ji').set(0.0)  # disable inertia estimation
        # self.pin('est.li').set(0.0)  # disable load estimation

        # self.signal("cw_bit_enable_operation").link(self.pin("est.en"))
        # self.signal("load").link(self.pin("est.load"))
        # self.signal("inertia").link(self.pin("est.inertia"))
        # self.signal("friction").link(self.pin("est.friction"))
        # self.signal("damping").link(self.pin("est.damping"))
        # self.signal("vel_cmd").link(self.pin("est.vel-fb"))
        # self.signal("acc_cmd").link(self.pin("est.acc-fb"))
        # self.signal("torque_loop_cmd").link(self.pin("est.torque-fb"))
        # hal.Signal("curr_periodf").link(self.pin("est.periodf"))

        # control loop
        # self.newinst("ppi", "ppi")
        # self.func_config("ppi.funct", self.Prio.CMD_CHAIN + 4)

        # load parameters from hardware_settings.yaml
        # self.pin('ppi.pos-p').set(self.pos_p)
        # self.pin('ppi.vel-p').set(self.vel_p)
        # self.pin('ppi.vel-i').set(self.vel_i)
        # self.pin('ppi.acc-p').set(self.acc_p)
        # self.pin('ppi.acc-i').set(self.acc_i)
        # self.pin('ppi.pos-g').set(self.pos_g)
        # self.pin('ppi.max-pos').set(self.max_pos)
        # self.pin('ppi.min-pos').set(self.min_pos)
        # self.pin('ppi.max-vel').set(self.max_vel * 1.1)
        # self.pin('ppi.max-acc').set(self.max_acc * 1.1)
        # self.pin('ppi.max-torque').set(self.max_torque)
        # self.pin('ppi.torque-boost').set(self.torque_boost)
        # self.pin('ppi.max-pos-error').set(0.1)
        # self.pin('ppi.max-vel-error').set(0.3)
        # self.pin('ppi.max-sat').set(0.5)

        # self.signal("cw_bit_enable_operation").link(self.pin("ppi.en"))
        # self.link_signal('pos_cmd', 'ppi.pos-ff')
        # self.link_signal('vel_cmd', 'ppi.vel-ff')
        # self.link_signal('acc_cmd', 'ppi.acc-ff')
        # self.link_signal('pos_fb', 'ppi.pos-fb')
        # self.link_signal('vel_fb', 'ppi.vel-fb')
        # self.link_signal('acc_fb', 'ppi.acc-fb')
        # self.link_signal('torque_cmd', 'ppi.torque-cmd')
        # self.signal("load").link(self.pin("ppi.load"))
        # self.signal("inertia").link(self.pin("ppi.inertia"))
        # self.signal("friction").link(self.pin("ppi.friction"))
        # self.signal("damping").link(self.pin("ppi.damping"))
        # self.signal("torque_loop_cmd").link(self.pin("ppi.torque-loop-cmd"))
        # hal.Signal("curr_periodf").link(self.pin("ppi.periodf"))

        # connect drive
        self.signal('pos_cmd').link(
            'lcec.0.%i.position-reference' % self.slavenum
        )
        self.signal('vel_cmd').link(
            'lcec.0.%i.velocity-reference' % self.slavenum
        )
        self.signal('torque_cmd').link(
            'lcec.0.%i.torque-reference' % self.slavenum
        )

        # self.signal('drive_pos_fb').link(
        #     'lcec.0.%i.position-actual-value' % self.slavenum
        # )
        # self.signal('drive_vel_fb').link(
        #     'lcec.0.%i.velocity-actual-value' % self.slavenum
        # )
        self.signal('torque_fb').link(
            'lcec.0.%i.torque-actual-value' % self.slavenum
        )

        self.signal('drive_pos_fb').link(
            'lcec.0.%i.position-actual-value' % self.slavenum
        )

        self.signal('pos_err').link(
            'lcec.0.%i.following-error-actual-value' % self.slavenum
        )

        # hal.Pin("lcec.0.%i.velocity-actual-value" % self.slavenum).link(
        #    self.pin("vel_fb_flt.in")
        # self.signal('torque_cmd').link(
        #    'lcec.0.%i.torque-reference' % self.slavenum
        # )

        # Drive control:  Link lcec pins to 402_mgr
        for name in (
            'control-word',
            'status-word',
            'drive-mode-cmd',
            'drive-mode-fb',
            'aux-error-code',
        ):
            self.signal(name).link(f"lcec.0.{self.slavenum}.{name}")
        for name in (
            'control-word',
            'status-word',
            'drive-mode-cmd',
            'drive-mode-fb',
            'aux-error-code',
            'slave-online',
            'slave-oper',
            'slave-state-init',
            'slave-state-preop',
            'slave-state-safeop',
            'slave-state-op',
        ):
            self.signal(name).link(f"lcec.0.{self.slavenum}.{name}")


class HALJointPlumberSim(HALJointPlumber):
    # For sim mode, set up components to simulate the drive moving
    # to the commanded position
    #
    # Simulated motion:
    # - A limit3 comp pretends to move to command position
    #   w/velocity and accel limits
    #
    # Enable chain:
    #
    # - When disabled and zero velocity (sim_drive_load and2 comp),
    #   enable sim motion load pin to accommodate startup position changes
    #
    # - This still allows pretending to be real hardware that takes
    #   time to stop after enable goes low
    #
    # - Sim drive operation_enabled set to True; False doesn't make
    #   sense
    #
    # signal                 -> comp/pin            -> signal
    # ------                    --------               ------
    # (N/A)                sim_startup_selector.load -> sim_drive_load
    #
    # sim_drive_load         -> *_sim_drive.load
    #
    # enable                 -> sim_startup_selector.enable

    def connect_drive(self):
        # Set up simulated start position
        self.newinst("muxnv2", "sim_pos_channel")
        # - Net sim start-up position to sim drive input
        self.newsig("sim_cmd_or_start_pos", hal.HAL_FLOAT)
        self.link_signal("sim_cmd_or_start_pos", "sim_pos_channel.out")
        # - Selector inputs
        hal.Signal("sim_pos_sel").link(self.pin("sim_pos_channel.sel"))
        self.link_signal("ros_pos_cmd", "sim_pos_channel.in0")  # cmd pos
        self.set_pin(  # start-up pos
            "sim_pos_channel.in1", self.sim_startup_position
        )
        # - Add update function at drive fb position
        self.func_config("sim_pos_channel.funct", self.Prio.DRIVE_READ_FB + 2)

        # Set up simulated drive motion
        # - Use limit3 comp
        self.newinst("limit3", "sim_drive")
        # - Sim drive joint limits
        self.set_pin("sim_drive.maxv", self.max_vel)
        self.set_pin("sim_drive.maxa", self.max_acc)
        # - Plumb input & output
        self.link_signal("sim_cmd_or_start_pos", "sim_drive.in")
        # self.link_signal("drive_pos_fb", "sim_drive.out")
        # self.link_signal("drive_vel_fb", "sim_drive.out-vel")
        self.link_signal("pos_fb", "sim_drive.out")
        self.link_signal("vel_fb", "sim_drive.out-vel")

        # - Link start pin to enable_operation
        self.link_signal('cw_bit_enable_operation', 'sim_drive.start')
        # - Add update function after sim start position
        self.func_config("sim_drive.funct", self.Prio.DRIVE_READ_FB + 10)

        # Set up simulated drive state, mode, fault signals
        sw_sig = self.signal('status-word')
        sw_sig.link(f"hal_402_mgr.drive_{self.num}.status-word-sim")
        drive_mode_pin = hal.Pin(f"hal_402_mgr.drive_{self.num}.drive-mode-fb")
        drive_mode_pin.unlink()
        del hal.signals[self.jname('drive-mode-fb')]
        dmc_sig = self.signal('drive-mode-cmd')
        dmc_sig.link(drive_mode_pin)

        for name, val in {
            'slave-online': True,
            'slave-oper': True,
            'slave-state-init': False,
            'slave-state-preop': False,
            'slave-state-safeop': False,
            'slave-state-op': True,
        }.items():
            self.signal(name).set(val)


###########################################
# HALPlumber and subclasses
#
# This does the top-level configuration, including running the above
# joint-level configuration; subclass for specific drives


class HALPlumber(HALPlumberBase):
    # Subclasses must define these
    mode_name = "Invalid"
    joint_plumber_class = None

    def __init__(self, config):
        # Set base object attributes
        for k, v in config.items():
            setattr(self, k, v)

        # Set up joints
        self.joints = []
        
        rospy.loginfo("========================================================")
        rospy.loginfo(self.joint_config.keys())

        for jname, jconfig in self.joint_config.items():
            self.joints.append(
                self.joint_plumber_class(self.thread_name, jname, jconfig)
            )


    @property
    def num_joints(self):
        return len(self.joints)

    def setup_hal(self):
        # This represents the high-level work of setting up HAL
        # - Initialize plumbing
        self.init_plumbing()

        # - Set up safety input max_vel_safety scaling
        self.setup_drive_safety()
        # - Load the 402 manager
        self.load_402_mgr()
        # - Load the hal_hw_interface comp
        self.load_hal_hw_interface()
        # - Top-level drive set-up (in subclasses)
        self.setup_drive()
        # - Per-joint set-up
        for joint in self.joints:
            joint.setup_hal()

        # - Quick stop drives upon fault
        self.setup_quick_stop()
        # - Load the latency comp
        self.setup_latency()

        # - Set up thread, add functions, start thread
        self.instantiate_threads()

    def init_plumbing(self):
        # Load locally-build icomps; instances to be created later
        # loadrt_local('limit3v3')
        # loadrt_local('ppi')
        loadrt_local('pll')
        # loadrt_local('est')
        loadrt_local('latency')
        loadrt_local('qc')
        hal.newsig("curr_period", hal.HAL_S32)
        hal.newsig("curr_periodf", hal.HAL_FLOAT)

        # Create DIO signals
        for ix in range(1, 17):
            # FIXME this should be pruned down
            hal.newsig(f"digital_in_{ix}", hal.HAL_BIT)
            hal.newsig(f"digital_out_{ix}", hal.HAL_BIT)

    def setup_drive_safety(self):
        # Desired behavior:
        # - Safety input high:  Drives run at full speed at next motion
        # - Safety input low, falling edge:  Drives quick stop
        # - Safety input low:
        #   - Enabling input low:  Drives cannot start
        #   - Enabling input high:  Drives can start & run at 10% speed

        loadrt_local('drive_safety')
        rtapi.newinst(
            'drive_safety', 'drive_safety', drivecount=self.num_joints
        )
        self.func_config("drive_safety", self.Prio.SAFETY_CHAIN + 10)

        #  Safety input signal:  Safe high
        safety_input_sig = hal.newsig("safety_input", hal.HAL_BIT)
        safety_input_sig.link("drive_safety.safety-input")
        safety_input_sig.set(1)  # For sim

        #  Enabling (deadman) device input:  Enable high
        enabling_input_sig = hal.newsig("enabling_input", hal.HAL_BIT)
        enabling_input_sig.link("drive_safety.enabling-input")
        enabling_input_sig.set(1)  # For sim

        #  Velocity safety scale:  reduce velocity to 10% when safety input low
        max_vel_safety_scale = rospy.get_param(
            '/hardware_interface/max_vel_safety_scale', 0.1
        )
        hal.Pin("drive_safety.unsafe-vel-limit").set(max_vel_safety_scale)
        max_vel_scale_sig = hal.newsig("max_vel_safety_scale", hal.HAL_FLOAT)
        max_vel_scale_sig.link(hal.Pin("drive_safety.vel-limit"))

        # State command for 402_mgr
        sc_sig = hal.newsig('state_cmd', hal.HAL_U32)
        sc_sig.link('drive_safety.state-cmd')

        # External quick stop signal
        qs_sig = hal.newsig('quick_stop', hal.HAL_BIT)
        qs_sig.link(hal.Pin("drive_safety.quick-stop"))

        # External soft stop signal
        ss_sig = hal.newsig('soft_stop', hal.HAL_BIT)
        ss_sig.link(hal.Pin("drive_safety.soft-stop"))

    def load_402_mgr(self):
        # Load user comp from ROS package
        cmd = 'rosrun --debug hal_402_device_mgr hal_402_mgr'
        rospy.loginfo("Starting hal_402_mgr component, cmd: '%s'" % cmd)
        hal.loadusr(cmd, wait=True, wait_name='hal_402_mgr', wait_timeout=20.0)
        rospy.loginfo("Successfully started hal_402_mgr component")

        # Link quick_stop input, reset output, state_cmd IO signals
        hal.Signal('quick_stop').link('hal_402_mgr.quick_stop')
        reset_sig = hal.newsig('reset', hal.HAL_BIT)
        reset_sig.link('hal_402_mgr.reset')
        sc_sig = hal.Signal('state_cmd')
        sc_sig.link('hal_402_mgr.state-cmd')

    def load_hal_hw_interface(self):
        rtapi.loadrt("%s/hal_hw_interface" % os.environ["COMP_DIR"])
        # Run the hal_hw_interface function right in the middle
        self.func_config("hal_hw_interface", self.Prio.ROS_CONTROL)

        hal.Signal('reset').link('hal_hw_interface.reset')

        # hal.Signal('soft_stop').link('hal_hw_interface.stop')
        # hal.Signal('quick_stop').link('hal_hw_interface.estop')

        # Controller error code signal
        cec_sig = hal.newsig('controller_error_code', hal.HAL_S32)

    def setup_drive(self):
        # Override in classes that need a drive setup routine
        raise RuntimeError("Subclasses must implement setup_drive() method")

    def setup_quick_stop(self):
        raise NotImplementedError

    def setup_latency(self):
        rtapi.newinst("latency", "latency")
        self.func_config("latency.funct", self.Prio.CMD_CHAIN + 20)
        hal.Signal('curr_period').link("latency.curr-period")
        hal.Signal('curr_periodf').link("latency.curr-periodf")

    def instantiate_threads(self):
        # Init HAL thread, maybe setting cgroup
        kwargs = {}
        if self.cgname is not None:
            kwargs["cgname"] = self.cgname
        rtapi.newthread(self.thread_name, self.thread_period, fp=True, **kwargs)
        # Add functions to thread in correct order
        self.add_funcs()
        hal.Signal('curr_period').link("robot_hw_thread.curr-period")


class HALPlumberEC(HALPlumber):
    mode_name = "EtherCAT"
    joint_plumber_class = HALJointPlumberEC

    def setup_drive(self):
        # Configure lcec component
        rospy.loginfo(
            "Loading LCEC with config file %s" % self.lcec_config_file
        )
        # load ethercat config parser
        hal.loadusr(
            "lcec_conf %s" % self.lcec_config_file, wait=True, wait_timeout=10.0
        )
        # load ethercat realtime module
        rtapi.loadrt("lcec")

        # Read fb at beginning of cycle, write cmd at end
        self.func_config("lcec.0.read", self.Prio.DRIVE_READ_FB)
        self.func_config("lcec.0.write", self.Prio.DRIVE_WRITE_CMD + 10)

    def setup_quick_stop(self):
        rtapi.newinst('ornv2', 'ext_fault', pincount=(self.num_joints))
        self.func_config('ext_fault.funct', self.Prio.DRIVE_READ_FB + 5)
        s = hal.newsig("ext_fault", hal.HAL_BIT)
        s.link('ext_fault.out')
        s.link('drive_safety.quick-stop-ext')
        # for joint in self.joints:
        #    s = hal.newsig('joint' + str(joint.num) + '_error', hal.HAL_BIT)
        #    s.link("joint" + str(joint.num) + "_ppi.error")
        #    s.link("ext_fault.in" + str(joint.slavenum))


class HALPlumberSim(HALPlumber):
    mode_name = "sim"
    joint_plumber_class = HALJointPlumberSim

    def setup_drive(self):
        # See HALJointPlumberSim.connect_drive() for info

        # sim_start_pos signal:  when True, load start position
        # - oneshot for triggering load once at startup
        rtapi.newinst('oneshot', 'sim_start_pos')
        self.func_config('sim_start_pos.funct', self.Prio.DRIVE_READ_FB)
        # - go high for a short time, only once
        hal.Pin('sim_start_pos.width').set(0.1)
        hal.Pin('sim_start_pos.retriggerable').set(False)
        hal.Pin('sim_start_pos.in').set(True)
        # - signal will connect to sim_drive.load and to sim_pos_sel input
        ssp_sig = hal.newsig('sim_start_pos', hal.HAL_BIT)
        ssp_sig.link('sim_start_pos.out')

        # sim_pos_sel signal:  switch btw. start and ROS cmd positions
        rtapi.newinst("conv_bit_s32", "sim_pos_sel")
        self.func_config('sim_pos_sel.funct', self.Prio.DRIVE_READ_FB + 1)
        ssp_sig.link('sim_pos_sel.in')
        # - signal will connect to sim_pos_channel inputs
        sps_sig = hal.newsig('sim_pos_sel', hal.HAL_S32)
        sps_sig.link('sim_pos_sel.out')

    def setup_quick_stop(self):
        pass
