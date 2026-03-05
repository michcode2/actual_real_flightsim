use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector, Vector3};

use crate::{cockpit::Cockpit, engine::Engine, wing::Wing};

pub struct Plane {
    wings: Vec<Wing>,
    mass: f64,
    pub transform_in_world: Isometry3<f64>,
    pub transform_rate_in_world: Isometry3<f64>,
    engine: Engine,
}

impl Plane {
    /// coordinate system: x forwards, y right, z down

    pub fn new_solid_guess() -> Plane {
        let wings = vec![
            Wing::new_area_location(17.16, Vector3::new(0.25, 0.0, 0.0)),
            Wing::new_area_location(5.0, Vector3::new(-5.0, 0.0, 0.0)),
        ];
        let mass = 1160.0;
        //let position = Vector3::new(0.0, 0.0, -0.35);
        let position = Vector3::new(-150.0, 0.0, 0.25);
        let velocity = Vector3::new(0.0, 0.0, 0.0);
        Plane {
            wings,
            mass,
            transform_in_world: Isometry3::new(position, nalgebra::zero()),
            transform_rate_in_world: Isometry3::new(velocity, nalgebra::zero()),
            engine: Engine::new(),
        }
    }

    pub fn new_in_flight() -> Plane {
        let wings = vec![
            Wing::new_area_location(17.16, Vector3::new(0.25, 0.0, 0.0)),
            Wing::new_area_location(5.0, Vector3::new(-5.0, 0.0, 0.0)),
        ];
        let mass = 1160.0;
        //let position = Vector3::new(0.0, 0.0, -0.35);
        let position = Vector3::new(-150.0, 0.0, -50.25);
        let velocity = Vector3::new(40.0, 0.0, 0.0);
        let angular_velocity = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        let pointing = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        Plane {
            wings,
            mass,
            transform_in_world: Isometry3::new(position, nalgebra::zero()),
            transform_rate_in_world: Isometry3::new(velocity, nalgebra::zero()),
            engine: Engine::new(),
        }
    }

    pub fn run_physics(&mut self, dt: f64, controls: &Cockpit) {
        let aero_forces: Vector3<f64> = self
            .wings
            .iter()
            .map(|wing| wing.calculate_forces(&self.plane_velocity()))
            .sum();
    }

    fn calculate_all_forces(&mut self) -> Vector3<f64> {
        return nalgebra::zero();
    }

    pub fn plane_velocity(&self) -> Vector3<f64> {
        return (self.transform_in_world.rotation.inverse()
            * self.transform_rate_in_world.translation)
            .translation
            .vector;
    }

    fn ground_moments(&self) -> Vector3<f64> {
        if self.transform_in_world.translation.z < -1.0 {
            return Vector3::zeros();
        }
        let pointing_contrib = Vector3::new(
            (&self.transform_in_world.rotation.euler_angles().1 + 0.1) * -100000.0,
            0.0,
            0.0,
        );

        let rotating_contrib = Vector3::new(
            &self.transform_rate_in_world.rotation.euler_angles().1 * -10.0,
            0.0,
            0.0,
        );

        pointing_contrib + rotating_contrib
    }

    fn ground_force(&self) -> f64 {
        let k = 15000.0;
        let k_d = 1500.0;
        if self.transform_in_world.translation.z < -1.0 {
            return 0.0;
        } else {
            let spring = k * (self.transform_in_world.translation.z + 1.0);
            let damper = k_d * self.transform_rate_in_world.translation.z;
            return spring + damper;
        }
    }

    fn engine_force(&self, controls: &Cockpit) -> Vector3<f64> {
        Vector3::new(controls.throttle * self.engine.max_thrust, 0.0, 0.0)
    }
}

#[cfg(test)]
mod test {
    use std::f64::consts::PI;

    use nalgebra::{Isometry3, UnitQuaternion, Vector3};

    use crate::{engine::Engine, plane::Plane, wing::Wing};

    fn make_default_aircraft() -> Plane {
        Plane {
            wings: vec![Wing::new_area_location(5.0, Vector3::new(-1.0, 0.0, 0.0))],
            mass: 10.0,
            engine: Engine::new(),
            transform_in_world: Isometry3::new(nalgebra::zero(), nalgebra::zero()),
            transform_rate_in_world: Isometry3::new(Vector3::new(10.0, 0.0, 0.0), nalgebra::zero()),
        }
    }

    #[test]
    fn plane_velocity_pointing_north() {
        let under_test = make_default_aircraft();
        assert_eq!(under_test.plane_velocity(), Vector3::new(10.0, 0.0, 0.0));
    }

    #[test]
    fn plane_velocity_pointing_east_with_translation() {
        let mut under_test = make_default_aircraft();
        under_test.transform_in_world.rotation =
            UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 2.0);
        under_test.transform_in_world.translation.vector = Vector3::new(10.0, 64.2, 1.5);
        assert!((under_test.plane_velocity().y - -10.0).abs() < 1e-3);
    }

    #[test]
    fn plane_velocity_pitched_45_degrees() {
        let mut under_test = make_default_aircraft();
        under_test.transform_in_world.rotation =
            UnitQuaternion::from_euler_angles(0.0, PI / 4.0, 0.0);
        let velocity = under_test.plane_velocity();
        assert!((velocity.z - velocity.x).abs() < 1e-6);
    }
}
