import targets
import testlib

class sargantana_hart(targets.Hart):
    name = "sargantana"
    xlen = 64
    ram = 0x80000000
    ram_size = 0x40000000
    bad_address = 0x00020000
    instruction_hardware_breakpoint_count = 0
    reset_vectors = [0x100]
    link_script_path = "sargantana.lds"
    misa = 0x8000000000341129
    honors_tdata1_hmode = False

class sargantana(targets.Target):
    harts = [sargantana_hart()]
    openocd_config_path = "sargantana.cfg"
    timeout_sec = 600
    implements_custom_test = False
    freertos_binary = "bin/RTOSDemo64.axf"
    support_unavailable_control = False
    supports_clint_mtime = False
    invalid_memory_returns_zero = False
    support_manual_hwbp = False
    test_semihosting = False
    support_memory_sampling = False

    def create(self):
        return testlib.VerilatorSim(sim_cmd=self.sim_cmd, debug=False)
    