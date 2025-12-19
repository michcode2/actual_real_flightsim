use std::f32;

use bevy::{DefaultPlugins, prelude::*};

use flight_dynamics_lib::{cockpit::Cockpit, plane::Plane};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, update_state)
        .add_systems(Update, overlay)
        .insert_resource(PlaneConnector::new())
        .insert_resource(CockpitConnector::new())
        .run();
}

#[derive(Resource)]
struct CockpitConnector {
    controls: Cockpit,
}

impl CockpitConnector {
    fn new() -> CockpitConnector {
        CockpitConnector {
            controls: Cockpit::new(),
        }
    }
}

#[derive(Resource)]
struct PlaneConnector {
    plane: flight_dynamics_lib::plane::Plane,
}

impl PlaneConnector {
    fn new() -> PlaneConnector {
        PlaneConnector {
            plane: Plane::new_solid_guess(),
        }
    }

    fn run(&mut self, time: f64, controls: &Cockpit) {
        self.plane.run_physics(time, controls);
    }
}

#[derive(Component)]
struct RunwayMarker;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // cube
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(550.0, 0.1, 12.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.0, 1.0, 0.0),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        RunwayMarker,
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(500.0, 0.1, 10.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.2, 0.2),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.05, 0.0),
        RunwayMarker,
    ));

    for i in (-250..250).step_by(10) {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(5.0, 0.1, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.9, 0.9, 0.9),
                ..default()
            })),
            Transform::from_xyz(i as f32, 0.06, 0.0),
            RunwayMarker,
        ));
    }

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(50000.0, 0.01, 50000.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.9, 0.0),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // light
    commands.spawn((
        PointLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));

    // camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-2.5, 4.5, 9.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Text::new("pitch\nyaw\nroll"),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(5.0),
            ..default()
        },
    ));
}

fn update_state(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut plane: ResMut<PlaneConnector>,
    mut controls: ResMut<CockpitConnector>,
    camera: Query<(&Camera, &mut Transform), Without<RunwayMarker>>,
) {
    controls.controls.zero();
    if keyboard_input.pressed(KeyCode::KeyZ) {
        controls.controls.throttle = 1.0;
    }

    if keyboard_input.pressed(KeyCode::KeyX) {
        controls.controls.throttle = 0.0;
    }

    controls.controls.throttle = controls.controls.throttle.min(1.0).max(0.0);

    if keyboard_input.pressed(KeyCode::KeyA) {
        controls.controls.yaw = 0.01;
    }

    if keyboard_input.pressed(KeyCode::KeyD) {
        controls.controls.yaw = -0.01;
    }

    if keyboard_input.pressed(KeyCode::KeyW) {
        controls.controls.elevator = -0.01;
    }

    if keyboard_input.pressed(KeyCode::KeyS) {
        controls.controls.elevator = 0.01;
    }

    if keyboard_input.pressed(KeyCode::KeyQ) {
        controls.controls.roll = 0.01;
    }

    if keyboard_input.pressed(KeyCode::KeyE) {
        controls.controls.roll = -0.01;
    }

    for (_, mut transform) in camera {
        transform.translation.x = plane.plane.position.x as f32;
        transform.translation.y = 2.0 - 1.0 * plane.plane.position.z as f32;
        transform.translation.z = -1.0 * plane.plane.position.y as f32;

        let (roll, pitch, yaw) = plane.plane.pointing.euler_angles();

        transform.rotation = Quat::from_euler(
            EulerRot::YXZ,
            yaw as f32 - f32::consts::FRAC_PI_2,
            -pitch as f32,
            roll as f32,
        );
    }

    plane.run(time.delta_secs_f64(), &controls.controls);
}

fn overlay(text: Query<&mut Text>, plane: ResMut<PlaneConnector>) {
    for mut words in text {
        let (_, ppitch, _) = plane.plane.pointing.euler_angles();
        **words = format!(
            "pitch angle deg {:.2},\nvelocity{:?}\naltitude {:.1}\nclimb rate {:.1}",
            ppitch * -180.0 / std::f64::consts::PI,
            plane.plane.velocity,
            -plane.plane.position.z,
            -plane.plane.velocity.z
        );
        println!("p{:?}", plane.plane.position);
        println!("v{:?}", plane.plane.velocity);
        println!();
    }
}
