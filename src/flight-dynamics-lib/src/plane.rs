use nalgebra::{UnitQuaternion, Vector3};

use crate::{cockpit::Cockpit, engine::Engine, wing::Wing};

pub struct Plane {
    wings: Vec<Wing>,
    mass: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    acceleration: Vector3<f64>,
    pub pointing: UnitQuaternion<f64>,
    angular_velocity: UnitQuaternion<f64>,
    engine: Engine,
}

impl Plane {
    /// coordinate system: x forwards, y right, z down

    pub fn new_solid_guess() -> Plane {
        let wings = vec![Wing::new_area_only(17.16)];
        let mass = 1160.0;
        //let position = Vector3::new(0.0, 0.0, -0.35);
        let position = Vector3::new(0.0, 0.0, 0.35);
        let velocity = Vector3::new(0.0, 0.0, 0.0);
        let acceleration = Vector3::new(0.0, 0.0, 0.0);
        let angular_velocity = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        let pointing = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        Plane {
            wings,
            mass,
            position,
            velocity,
            acceleration,
            angular_velocity,
            pointing,
            engine: Engine::new(),
        }
    }

    pub fn run_physics(&mut self, dt: f64, controls: &Cockpit) {
        let mut forces = self
            .wings
            .iter()
            .map(|wing| {
                self.pointing.inverse().to_rotation_matrix()
                    * wing.calculate(self.pointing.to_rotation_matrix() * self.velocity)
            })
            .sum::<Vector3<f64>>();

        forces.z += self.mass * 9.81;
        println!(
            "wheel {:.2}, weight: {}",
            self.ground_force(),
            self.mass * 9.81
        );
        forces.z -= self.ground_force();
        forces += self.pointing.to_rotation_matrix() * self.engine_force(controls);
        println!("forces {:?}", forces);

        self.acceleration = forces / self.mass;
        self.velocity += self.acceleration * dt;
        self.position += self.velocity * dt;

        let control_quaternion =
            UnitQuaternion::from_euler_angles(controls.roll, -controls.elevator, controls.yaw);

        self.angular_velocity *= control_quaternion;
        self.pointing = self.pointing.slerp(&self.angular_velocity, dt);
    }

    fn ground_force(&self) -> f64 {
        let k = 15000.0;
        let k_d = 1500.0;
        if self.position.z < -1.0 {
            return 0.0;
        } else {
            let spring = k * (self.position.z + 1.0);
            let damper = k_d * self.velocity.z;
            return spring + damper;
        }
    }

    fn engine_force(&self, controls: &Cockpit) -> Vector3<f64> {
        Vector3::new(controls.throttle * self.engine.max_thrust, 0.0, 0.0)
    }
}

#[cfg(test)]
mod test {
    use std::f64;

    use nalgebra::{UnitQuaternion, Vector3};

    use crate::{
        physics::{engine::Engine, plane::Plane, wing::Wing},
        ui_things::cockpit::Cockpit,
    };

    #[test]
    fn no_thrust() {
        let mut under_test = Plane {
            wings: vec![Wing::new_area_only(1.0)],
            mass: 3.0,
            position: Vector3::new(0.0, 0.0, -100.0),
            velocity: Vector3::new(10.0, 0.0, 0.0),
            acceleration: Vector3::new(0.0, 0.0, 0.0),
            angular_velocity: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            engine: Engine::new(),
            pointing: nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        };

        let handsfree = Cockpit::new();

        under_test.run_physics(0.01, &handsfree);

        println!(
            "{:?}\n{:?}\n{:?}",
            under_test.position, under_test.velocity, under_test.acceleration
        );

        assert!(under_test.position.x == 0.01 * 10.0);
        assert!(under_test.position.y == 0.0);
        assert!((under_test.position.z + 99.999).abs() < 1e-3);
    }

    #[test]
    fn no_thrust_rotated() {
        let mut under_test = Plane {
            wings: vec![Wing::new_area_only(1.0)],
            mass: 3.0,
            position: Vector3::new(0.0, 0.0, -100.0),
            velocity: Vector3::new(0.0, 10.0, 0.0),
            acceleration: Vector3::new(0.0, 0.0, 0.0),
            angular_velocity: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            engine: Engine::new(),
            pointing: nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, f64::consts::FRAC_PI_2),
        };

        let handsfree = Cockpit::new();

        under_test.run_physics(0.01, &handsfree);

        println!(
            "{:?}\n{:?}\n{:?}",
            under_test.position, under_test.velocity, under_test.acceleration
        );

        assert!(under_test.position.y == 0.01 * 10.0);
        assert!(under_test.position.x == 0.0);
        assert!((under_test.position.z + 99.999).abs() < 1e-3);
    }

    #[test]
    fn some_thrust() {
        let mut under_test = Plane {
            wings: vec![Wing::new_area_only(1.0)],
            mass: 3.0,
            position: Vector3::new(0.0, 0.0, -100.0),
            velocity: Vector3::new(10.0, 0.0, 0.0),
            acceleration: Vector3::new(0.0, 0.0, 0.0),
            angular_velocity: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            engine: Engine::new(),
            pointing: nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        };
        let mut handsfree = Cockpit::new();
        handsfree.throttle = 0.1;

        under_test.run_physics(0.01, &handsfree);

        println!(
            "{:?}\n{:?}\n{:?}",
            under_test.position, under_test.velocity, under_test.acceleration
        );

        assert!(under_test.position.y == 0.0);
        assert!(under_test.velocity.y == 0.0);
        assert!(under_test.acceleration.y == 0.0);
        assert!(under_test.position.x == under_test.velocity.x * 0.01);
    }

    #[test]
    fn some_alpha() {
        let mut under_test = Plane {
            wings: vec![Wing::new_area_only(1.0)],
            mass: 3.0,
            position: Vector3::new(0.0, 0.0, -100.0),
            velocity: Vector3::new(10.0, 0.0, 0.0),
            acceleration: Vector3::new(0.0, 0.0, 0.0),
            angular_velocity: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            engine: Engine { max_thrust: 10.0 },
            pointing: nalgebra::UnitQuaternion::from_euler_angles(
                0.0,
                f64::consts::PI / 180.0,
                0.0,
            ),
        };
        let mut handsfree = Cockpit::new();
        handsfree.throttle = 0.1;

        under_test.run_physics(0.01, &handsfree);

        println!(
            "{:?}\n{:?}\n{:?}",
            under_test.position, under_test.velocity, under_test.acceleration
        );

        assert!((under_test.acceleration.z - 7.7625).abs() < 1e-4);
    }
}
