use std::f64::{self, consts};

use nalgebra::{Isometry3, Point3, Transform3, Vector, Vector3};

pub struct Wing {
    area: f64,
    pub transform_on_plane: Isometry3<f64>,
    aspect_ratio: f64,
}

#[allow(non_snake_case)]
impl Wing {
    pub fn new_area_only(area: f64) -> Wing {
        Wing {
            area,
            transform_on_plane: Isometry3::new(Vector3::new(0.0, 0.0, 0.0), nalgebra::zero()),
            aspect_ratio: 5.0,
        }
    }

    pub fn new_area_location(area: f64, location_on_plane: Vector3<f64>) -> Wing {
        Wing {
            area,
            transform_on_plane: Isometry3::new(location_on_plane, nalgebra::zero()),
            aspect_ratio: 7.0,
        }
    }

    /// take in velocity in the aircraft coordinates, transform it to be in wing coordinates, do lift and drag, transform it back to aircraft
    /// cl and cd are crude for now
    pub fn calculate_forces(&self, velocity_body: &Vector3<f64>) -> Vector3<f64> {
        let velocity_wing = self.transform_on_plane.rotation * velocity_body;
        let U_inf = velocity_wing.magnitude();
        let alpha = (velocity_wing.z / velocity_wing.x).tan().to_degrees();

        //println!("{}, {}, {:?}", U_inf, alpha, velocity_wing);

        let cl = 0.1 * alpha;
        let cd = cl.powi(2);
        let lift_body = 0.5 * 1.225 * U_inf.powi(2) * self.area * cl;
        let drag_body = 0.5 * 1.225 * U_inf.powi(2) * self.area * cd;
        let forces_body = Vector3::new(-drag_body, 0.0, lift_body);
        return self.transform_on_plane.inverse() * forces_body;
    }

    pub fn calculate_moments(&self, velocity_body: &Vector3<f64>) -> Vector3<f64> {
        let forces = self.calculate_forces(&velocity_body);
        return self
            .transform_on_plane
            .translation
            .vector
            .component_mul(&forces);
    }

    fn dcl_dalpha(&self) -> f64 {
        let littlefrac = 2.0 / self.aspect_ratio;
        return 2.0 * f64::consts::PI / (1.0 + littlefrac);
    }
}

#[cfg(test)]
mod test {
    use std::f64;

    use nalgebra::{Transform3, Translation3, UnitQuaternion, Vector3};

    use crate::wing::Wing;

    #[test]
    fn zero_lift_zero_rotation() {
        let mut under_test = Wing::new_area_only(1.0);
        under_test.transform_on_plane.translation = Translation3::new(1.0, 0.0, 0.0);
        let U_inf = Vector3::new(10.0, 0.0, 0.0);

        assert_eq!(
            under_test.calculate_forces(&U_inf),
            Vector3::new(0.0, 0.0, 0.0)
        );

        let moments = under_test.calculate_moments(&U_inf);
        assert_eq!(moments, Vector3::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn alpha_is_1() {
        let pi = 3.14159_f64;
        let mut under_test = Wing::new_area_only(1.0);
        let U_inf = Vector3::new(10.0 * (pi / 180.0).cos(), 0.0, -10.0 * (pi / 180.0).sin());

        let forces = under_test.calculate_forces(&U_inf);

        assert!((-forces.z - (0.5 * 1.225 * 10.0)).abs() < 1e-2);

        let moments = under_test.calculate_moments(&U_inf);

        assert_eq!(moments, Vector3::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn setting_angle_is_1() {
        let pi = 3.14159_f64;
        let mut under_test = Wing::new_area_only(1.0);
        under_test.transform_on_plane.rotation =
            UnitQuaternion::from_euler_angles(0.0, pi / 180.0, 0.0);
        let U_inf = Vector3::new(10.0, 0.0, 0.0);

        let forces = under_test.calculate_forces(&U_inf);

        println!("{:?}", forces);

        assert!((-forces.z - (0.5 * 1.225 * 10.0)).abs() < 1e-1);
    }

    #[test]
    fn setting_angle_and_alpha_are_1() {
        let pi = 3.14159_f64;
        let mut under_test = Wing::new_area_only(1.0);
        under_test.transform_on_plane.rotation =
            UnitQuaternion::from_euler_angles(0.0, pi / 180.0, 0.0);
        let U_inf = Vector3::new(10.0 * (pi / 180.0).cos(), 0.0, -10.0 * (pi / 180.0).sin());

        let forces = under_test.calculate_forces(&U_inf);

        assert!((-forces.z - (0.5 * 1.225 * 100.0 * 0.2)).abs() < 1e-1);
    }

    #[test]
    fn rotated_wing_gives_equal_x_and_y() {
        let pi = 3.14159_f64;
        let mut under_test = Wing::new_area_only(1.0);
        under_test.transform_on_plane.rotation =
            UnitQuaternion::from_euler_angles(45.0 * pi / 180.0, 0.0, 0.0);
        let U_inf = Vector3::new(10.0 * (pi / 180.0).cos(), 0.0, -10.0 * (pi / 180.0).sin());

        let forces = under_test.calculate_forces(&U_inf);

        println!("{:?}", forces);
    }
}
