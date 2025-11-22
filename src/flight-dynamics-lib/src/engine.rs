pub struct Engine {
    pub max_thrust: f64,
}

impl Engine {
    pub fn new() -> Engine {
        return Engine { max_thrust: 2000.0 };
    }
}
