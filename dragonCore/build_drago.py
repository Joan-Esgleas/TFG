from siliconcompiler import Design, FPGADevice
from siliconcompiler import Lint, Sim
from siliconcompiler import ASIC, FPGA

from siliconcompiler.flows.lintflow import LintFlow
from siliconcompiler.flows.dvflow import DVFlow
from siliconcompiler.flows.fpgaflow import FPGAXilinxFlow
from siliconcompiler.flows.highresscreenshotflow import HighResScreenshotFlow

from siliconcompiler.targets import asic_target
from siliconcompiler.tools.verilator.compile import CompileTask
from siliconcompiler.tools.builtin.importfiles import ImportFilesTask
from siliconcompiler.tools.klayout.screenshot import ScreenshotTask

class DracoDesign(Design):
    def __init__(self):
        """Initializes the HeartbeatDesign object.

        This method sets up all the necessary filesets for RTL,
        simulation testbenches (Icarus and Verilator), and technology-specific
        constraint files (SDC for ASIC, XDC for FPGA).
        """
        super().__init__()
        # Set the design's name.
        self.set_name("drago")

        # Establish the root directory for all design-related files.
        self.set_dataroot("drago", __file__)


        # Configure filesets within the established data root.
        with self.active_dataroot("drago"):
            # RTL sources
            with self.active_fileset("rtl"):
                self.set_topmodule("top_drac")

                # Package files (order matters - packages should come first)
                self.add_file("./rtl/core/drago/includes/riscv_pkg.sv")
                self.add_file("./rtl/core/drago/includes/drac_pkg.sv")
                self.add_file("./rtl/core/drago/includes/def_pkg.sv")
                
                # Datapath
                self.add_file("./rtl/core/drago/rtl/datapath.sv")
                
                # IF stage
                self.add_file("./rtl/core/drago/rtl/if_stage/branch_predictor.sv")
                self.add_file("./rtl/core/drago/rtl/if_stage/bimodal_predictor.sv")
                self.add_file("./rtl/core/drago/rtl/if_stage/if_stage_1.sv")
                self.add_file("./rtl/core/drago/rtl/if_stage/if_stage_2.sv")
                
                # Top module
                self.add_file("./rtl/core/drago/rtl/top_drac.sv")
                
                # RR stage
                self.add_file("./rtl/core/drago/rtl/rr_stage/regfile.sv")
                
                # CSR interface
                self.add_file("./rtl/core/drago/rtl/csr_interface.sv")
                
                # ID stage
                self.add_file("./rtl/core/drago/rtl/id_stage/immediate.sv")
                self.add_file("./rtl/core/drago/rtl/id_stage/decoder.sv")
                
                # Control unit
                self.add_file("./rtl/core/drago/rtl/control_unit.sv")
                
                # Register
                self.add_file("./rtl/core/drago/rtl/register.sv")
                
                # EXE stage
                self.add_file("./rtl/core/drago/rtl/exe_stage/branch_unit.sv")
                self.add_file("./rtl/core/drago/rtl/exe_stage/div_unit.sv")
                self.add_file("./rtl/core/drago/rtl/exe_stage/mul_unit.sv")
                self.add_file("./rtl/core/drago/rtl/exe_stage/exe_stage.sv")
                self.add_file("./rtl/core/drago/rtl/exe_stage/alu.sv")
                self.add_file("./rtl/core/drago/rtl/exe_stage/div_4bits.sv")
                self.add_file("./rtl/core/drago/rtl/exe_stage/mem_unit.sv")
                self.add_file("./rtl/core/drago/rtl/exe_stage/branch_prediction_checker.sv")
                
                # CSR BSC
                self.add_file("./rtl/core/drago/rtl/csr_bsc/rtl/csr_bsc.sv")
                self.add_file("./rtl/core/drago/rtl/csr_bsc/rtl/hpm_counters.sv")

            # Testbench for Icarus Verilog
            #with self.active_fileset("testbench.icarus"):
            #    self.add_file("testbench.v")
            #    self.set_topmodule("heartbeat_tb")
            #    self.set_param("N", "8")

            # C++ Testbench for Verilator
           # with self.active_fileset("testbench.verilator"):
            #    self.set_topmodule("heartbeat")
            #    self.add_file("testbench.cc")
            #    self.set_param("N", "8")

            # ASIC timing constraints for the FreePDK45 technology.
            #with self.active_fileset("sdc.freepdk45"):
            #    self.add_file("heartbeat.sdc")

            # ASIC timing constraints for the ASAP7 technology.
            #with self.active_fileset("sdc.asap7"):
            #    self.add_file("heartbeat_asap7.sdc")

            # FPGA timing and pin constraints for a Xilinx Artix-7 device.
            #with self.active_fileset("fpga.xc7a100tcsg324"):
            #    self.add_file("heartbeat.xdc")

def asic(pdk: str = "freepdk45", N: str = None, design: Design = DracoDesign()):
    # Create a project instance for an ASIC flow.
    project = ASIC()

    project.set_design(design)

    # Add the necessary filesets.
    project.add_fileset("rtl")
    project.set('option', 'remote', False)
    #project.add_fileset(f"sdc.{pdk}")

    # Optionally override the 'N' parameter.
    if N is not None:
        design.set_param("N", N, fileset="rtl")

    # Load the target, which automatically selects the default 'asicflow'.
    asic_target(project, pdk=pdk)

    # Run the full place-and-route flow.
    project.run()
    # Display a summary of timing, power, and area results.
    project.summary()
    # Save the final layout and project configuration.
    project.snapshot()


def sim(N: str = None, tool: str = "verilator", design: Design = DracoDesign()):
    """Runs a simulation of the Heartbeat design.

    After the simulation completes, it attempts to open the generated
    waveform file (VCD) for viewing.

    Args:
        N (str, optional): The value for the Verilog parameter 'N'.
            Defaults to None, which uses the value set in the design schema.
        tool (str, optional): The simulation tool to use ('verilator' or
            'icarus'). Defaults to "verilator".
    """
    # Create a project instance tailored for simulation.
    project = Sim()

    # Instantiate and configure the design.
    project.set_design(design)

    # Add the tool-specific testbench and the RTL design files.
    #project.add_fileset(f"testbench.{tool}")
    project.add_fileset("rtl")
    # Set the appropriate design verification flow.
    project.set_flow(DVFlow(tool=tool))

    # Optionally override the 'N' parameter for the testbench.
    if N is not None:
        design.set_param("N", N, fileset=f"testbench.{tool}")

    if tool == "verilator":
        # Add trace to verilator
        CompileTask.find_task(project).set_verilator_trace(True)

    # Run the simulation.
    project.run()
    project.summary()

    vcd = None
    if tool == "icarus":
        # Find the VCD (Value Change Dump) waveform file from the results.
        vcd = project.find_result(step='simulate', index='0',
                                  directory="reports",
                                  filename="heartbeat_tb.vcd")
    else:
        # Find the VCD (Value Change Dump) waveform file from the results.
        vcd = project.find_result(step='simulate', index='0',
                                  directory="reports",
                                  filename="heartbeat.vcd")
    # If a VCD file is found, open it with the default waveform viewer.
    if vcd:
        project.show(vcd)


def main():
    draco = DracoDesign()
    #sim(design=draco)
    asic(pdk="freepdk45", design=draco)



if __name__ == '__main__':
    main()
