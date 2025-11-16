use std::f64;

use nalgebra::Vector3;

pub struct Wing {
    area: f64,
}
#[allow(non_snake_case)]
impl Wing {
    pub fn new_area_only(area: f64) -> Wing {
        Wing { area }
    }

    pub fn calculate(&self, velocity: Vector3<f64>) -> Vector3<f64> {
        let mut alpha_rad = f64::atan(velocity.z / velocity.x);
        if alpha_rad.is_nan() {
            alpha_rad = 0.0;
        }
        println!("alpha {}", alpha_rad * 180.0 / 3.14159);
        let Uinf = velocity.magnitude();
        let lift = self.calculate_lift(Uinf, alpha_rad);
        let drag = self.calculate_drag(Uinf * velocity.x.signum(), alpha_rad);

        Vector3::new(-drag, 0.0, lift)
    }

    fn calculate_lift(&self, velocity: f64, alpha: f64) -> f64 {
        let CL = self.calculate_CL(alpha);
        println!("CL: {CL}");
        return 0.5 * 1.225 * velocity.powi(2) * self.area * CL;
    }

    fn calculate_drag(&self, velocity: f64, alpha: f64) -> f64 {
        let CD = self.calculate_CD(alpha);
        println!("CD: {CD}");
        return 0.5 * 1.225 * velocity.powi(2) * self.area * CD;
    }

    ///alpha comes in in radians
    fn calculate_CL(&self, alpha_rad: f64) -> f64 {
        let alpha = alpha_rad * 180.0 / f64::consts::PI;
        if alpha.abs() < 14.0 {
            return alpha * 0.1;
        }
        return 0.5;
    }

    fn calculate_CD(&self, alpha_rad: f64) -> f64 {
        let alpha = alpha_rad * 180.0 / f64::consts::PI;
        if alpha.abs() < 14.0 {
            return 0.1 * (0.1 * alpha).powi(2);
        }
        return 1.5;
    }
}

#[cfg(test)]
mod test {
    use std::f64;

    use nalgebra::Vector3;

    use crate::physics::wing::Wing;

    #[test]
    fn unit_wing_right_numbers() {
        let under_test = Wing::new_area_only(1.0);
        let straight_ahead = under_test.calculate(Vector3::new(1.0, 0.0, 0.0));
        assert!(straight_ahead.x.abs() < 1e-6);
        assert!(straight_ahead.y.abs() < 1e-6);
        assert!(straight_ahead.z.abs() < 1e-6);

        let deg_rad_conversion = f64::consts::PI / 180.0;

        let velocity_alpha_1 = Vector3::new(
            1.0 * deg_rad_conversion.cos(),
            0.0,
            1.0 * deg_rad_conversion.sin(),
        );
        let alpha_1 = under_test.calculate(velocity_alpha_1);
        println!("{:?}", alpha_1);
        assert!((alpha_1.x + 0.0006125).abs() < 1e-6);
        assert!((alpha_1.z - 0.06125).abs() < 1e-6);
    }
}
