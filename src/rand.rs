use rand::rngs::OsRng;
use rand::RngCore;

pub struct Rng {
    rng: OsRng,
}

impl Rng {
    pub fn new() -> Self {
        Self {
            rng: OsRng::new().unwrap(),
        }
    }
    pub fn gen_range(&mut self, min: i32, max: i32) -> i32 {
        let r = self.rng.next_u32();
        (r % (max - min + 1) as u32) as i32 + min
    }
}
