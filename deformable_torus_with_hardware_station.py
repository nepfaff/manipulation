from pydrake.all import (
    AddContactMaterial,
    Box,
    CoulombFriction,
    DeformableBodyConfig,
    DrakeVisualizer,
    GeometryInstance,
    MakePhongIllustrationProperties,
    Mesh,
    ProximityProperties,
    RigidTransform,
    RotationMatrix,
    Simulator,
    np,
)

from manipulation.station import LoadScenario, MakeHardwareStation

# Note that this requires meldis `python3 -m pydrake.visualization.meldis` to be started
# in a different terminal for visualization.

# See https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1fem_1_1_deformable_body_config.html
# Different configs determine whether the torus passes around the red block or not.

# Desired duration of the simulation [s].
simulation_time = 4.0
# Desired real time rate.
realtime_rate = 0.1
# Young's modulus of the deformable body [Pa].
E = 2e3
# Poisson's ratio of the deformable body, unitless.
nu = 0.49
# Mass density of the deformable body [kg/m³].
density = 1e3
# Stiffness damping coefficient for the deformable body [1/s].
beta = 0.05


def demo_with_hardware_station():
    # Minimum required proximity properties for rigid bodies to interact with
    # deformable bodies.
    # 1. A valid Coulomb friction coefficient, and
    # 2. A resolution hint. (Rigid bodies need to be tesselated so that collision
    # queries can be performed against deformable geometries.)
    model_directive = """
    directives:
    - add_directives:
        file: package://manipulation/clutter.dmd.yaml
    
    model_drivers:
        # NOTE: `IiwaDriver` with deformable fails with `RuntimeError: Passed in array is not properly sized.`
        # iiwa: !IiwaDriver {}
        #     hand_model_name: wsg
        #     control_mode: position_only
        iiwa: !InverseDynamicsDriver {}
        wsg: !SchunkWsgDriver {}

    plant_config:
        time_step: 1e-3
        contact_model: "hydroelastic"
        discrete_contact_approximation: "lagged"
    """

    def parser_prefinalize_callback(parser, plant):
        deformable_config = DeformableBodyConfig()
        deformable_config.set_youngs_modulus(E)
        deformable_config.set_poissons_ratio(nu)
        deformable_config.set_mass_density(density)
        deformable_config.set_stiffness_damping_coefficient(beta)

        scale = 0.65
        torus_mesh = Mesh("./torus.vtk", scale)
        X_WB = RigidTransform([0.5, 0, 0.2])
        torus_instance = GeometryInstance(X_WB, torus_mesh, "deformable_torus")

        # Minimumly required proximity properties for deformable bodies:
        # A valid Coulomb friction coefficient.
        deformable_proximity_props = ProximityProperties()
        # Set the friction coefficient close to that of rubber against rubber.
        surface_friction = CoulombFriction(1.15, 1.15)
        AddContactMaterial(
            friction=surface_friction, properties=deformable_proximity_props
        )
        torus_instance.set_proximity_properties(deformable_proximity_props)
        torus_instance.set_illustration_properties(
            MakePhongIllustrationProperties([0.0, 0.0, 1.0, 1.0])
        )

        # Registration of all deformable geometries ostensibly requires a resolution
        # hint parameter that dictates how the shape is tesselated. In the case of a
        # `Mesh` shape, the resolution hint is unused because the shape is already
        # tessellated.
        # Though unused, we still asserts the resolution hint is positive.
        unused_resolution_hint = 1.0
        deformable_model = plant.mutable_deformable_model()
        deformable_model.RegisterDeformableBody(
            torus_instance, deformable_config, unused_resolution_hint
        )

        # Put a small block below the torus.
        rigid_proximity_props = ProximityProperties()
        rigid_proximity_props.AddProperty("hydroelastic", "resolution_hint", 1.0)
        AddContactMaterial(friction=surface_friction, properties=rigid_proximity_props)
        block = Box(0.06, 0.06, 0.06)
        X_WG = RigidTransform(RotationMatrix(), [0.5, 0, 0.1])
        plant.RegisterCollisionGeometry(
            plant.world_body(), X_WG, block, "block_collision", rigid_proximity_props
        )
        plant.RegisterVisualGeometry(
            plant.world_body(), X_WG, block, "block_visual", [0.7, 0.5, 0.4, 0.8]
        )

    def prebuild_callback(builder, scene_graph):
        DrakeVisualizer.AddToBuilder(builder, scene_graph)

    scenario = LoadScenario(data=model_directive)
    station = MakeHardwareStation(
        scenario,
        parser_prefinalize_callback=parser_prefinalize_callback,
        prebuild_callback=prebuild_callback,
    )

    # TODO: Replace these with a controller.
    context = station.CreateDefaultContext()
    # station.GetInputPort("iiwa.position").FixValue(context, np.zeros(7))
    station.GetInputPort("iiwa.desired_state").FixValue(context, np.zeros(14))
    station.GetInputPort("wsg.position").FixValue(context, [0.1])

    simulator = Simulator(station, context)
    simulator.set_target_realtime_rate(realtime_rate)
    simulator.AdvanceTo(simulation_time)


if __name__ == "__main__":
    demo_with_hardware_station()
