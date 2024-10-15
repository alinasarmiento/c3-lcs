from pydrake.geometry import StartMeshcat
from pydrake.systems.framework import DiagramBuilder
import pydrake.multibody.parsing as parsing
from manipulation.station import LoadScenario, MakeHardwareStation

meshcat = StartMeshcat()

pushbot_model = parsing.AddModel("home/grey/research/c3-lcs/examples/pushbot/urdf/pushbot.urdf")

scenario_data = """
directives:
- add_model:
    name: pushbot
    file: 
"""

scenario_data += pushbot_model.file
scenario_data += """
    default_joint_positions:
        base_joint: [0]
        push_joint: [0]
- add_weld:
    parent: world
    child: pushbot::base
"""

scenario = LoadScenario(data=scenario_data)
builder = DiagramBuilder()

station = builder.AddSystem(MakeHardwareStation(scenario, meshcat))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(0)
simulator.AdvanceTo(0.1)
