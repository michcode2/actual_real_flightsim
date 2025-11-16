use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::{physics::engine::Engine, physics::wing::Wing, ui_things::cockpit::Cockpit};

pub struct Plane {
    wings: Vec<Wing>,
    mass: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    acceleration: Vector3<f64>,
    pub pointing: UnitQuaternion<f64>,
    engine: Engine,
}

impl Plane {
    pub fn new_solid_guess() -> Plane {
        let wings = vec![Wing::new_area_only(17.16)];
        let mass = 1000.0;
        let position = Vector3::new(0.0, 0.0, 0.5);
        let velocity = Vector3::new(0.0, 0.0, 0.0);
        let acceleration = Vector3::new(0.0, 0.0, 0.0);
        let pointing = UnitQuaternion::from_euler_angles(0.0, 0.0, -1.54);
        Plane {
            wings,
            mass,
            position,
            velocity,
            acceleration,
            pointing,
            engine: Engine::new(),
        }
    }

    pub fn run_physics(&mut self, dt: f64, controls: &Cockpit) {
        self.acceleration.x = controls.throttle;
        let mut forces = self
            .wings
            .iter()
            .map(|wing| {
                self.pointing.inverse().to_rotation_matrix()
                    * wing.calculate(self.pointing.to_rotation_matrix() * self.velocity)
            })
            .sum::<Vector3<f64>>();

        forces.z -= self.mass * 9.81;
        println!("wheel{}, weight: {}", self.ground_force(), self.mass * 9.81);
        forces.z += self.ground_force();
        forces += self.pointing.inverse().to_rotation_matrix() * self.engine_force(controls);
        println!("forces {:?}", forces);
        println!("rotation matrix {:?}", self.pointing.to_rotation_matrix());

        self.acceleration = forces / self.mass;
        self.velocity += self.acceleration * dt;
        self.position += self.velocity * dt;

        let (mut roll, mut pitch, mut yaw) = self.pointing.euler_angles();
        roll += controls.roll;
        pitch += controls.elevator;
        yaw += controls.yaw;

        self.pointing = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    }

    fn ground_force(&self) -> f64 {
        let k = -20000.0;
        let k_d = -2000.0;
        if self.position.z > 1.0 {
            return 0.0;
        } else {
            let spring = k * (self.position.z - 1.0);
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
    use nalgebra::{Quaternion, Vector3};

    use crate::{
        physics::{engine::Engine, plane::Plane, wing::Wing},
        ui_things::cockpit::Cockpit,
    };

    #[test]
    fn hello() {
        let mut under_test = Plane {
            wings: vec![Wing::new_area_only(1.0)],
            mass: 3.0,
            position: Vector3::new(0.0, 0.0, 100.0),
            velocity: Vector3::new(10.0, 0.0, 0.0),
            acceleration: Vector3::new(0.0, 0.0, 0.0),
            engine: Engine::new(),
            pointing: nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        };

        let handsfree = Cockpit::new();

        under_test.run_physics(0.01, &handsfree);
    }
}
