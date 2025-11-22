pub struct Cockpit {
    pub throttle: f64,
    pub elevator: f64,
    pub roll: f64,
    pub yaw: f64,
}

impl Cockpit {
    pub fn new() -> Cockpit {
        Cockpit {
            throttle: 0.0,
            elevator: 0.0,
            roll: 0.0,
            yaw: 0.0,
        }
    }
    pub fn zero(&mut self) {
        self.elevator = 0.0;
        self.roll = 0.0;
        self.yaw = 0.0;
    }
}
