extern crate num_traits;
pub use math::num_traits::float::Float;
pub use math::num_traits::identities::Zero;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Copy, Clone, PartialEq)]
pub struct Vector2d<T: Float> {
    pub x: T,
    pub y: T,
}

impl<T: Float> Vector2d<T> {
    pub fn new(x: T, y: T) -> Self {
        Self { x, y }
    }

    pub fn len(&self) -> T {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn len_square(&self) -> T {
        self.x * self.x + self.y * self.y
    }

    pub fn normalize(&mut self) {
        let length = self.len();
        self.x = self.x / length;
        self.y = self.y / length;
    }

    pub fn set(&mut self, x: T, y: T) {
        self.x = x;
        self.y = y;
    }

    pub fn rotate(&mut self, radians: T) {
        let c = radians.cos();
        let s = radians.sin();
        let x = self.x;
        let y = self.y;
        self.x = x * c - y * s;
        self.y = x * s + y * c;
    }
}

impl<T: Float> Mul for Vector2d<T> {
    type Output = T;
    fn mul(self, rhs: Self) -> T {
        self.x * rhs.x + self.y * rhs.y
    }
}

impl<T: Float> Mul<T> for Vector2d<T> {
    type Output = Self;
    fn mul(self, rhs: T) -> Self {
        Self {
            x: self.x * rhs.clone(),
            y: self.y * rhs.clone(),
        }
    }
}

impl<T: Float> Div<T> for Vector2d<T> {
    type Output = Self;
    fn div(self, rhs: T) -> Self {
        Self {
            x: self.x / rhs.clone(),
            y: self.y / rhs.clone(),
        }
    }
}

impl<T: Float> Add for Vector2d<T> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<T: Float> Sub for Vector2d<T> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl<T: Float> AddAssign for Vector2d<T> {
    fn add_assign(&mut self, other: Self) {
        self.x = self.x + other.x;
        self.y = self.y + other.y;
    }
}

impl<T: Float> SubAssign for Vector2d<T> {
    fn sub_assign(&mut self, other: Self) {
        self.x = self.x - other.x;
        self.y = self.y - other.y;
    }
}
impl<T: Float> Neg for Vector2d<T> {
    type Output = Vector2d<T>;
    fn neg(self) -> Vector2d<T> {
        Vector2d {
            x: -self.x,
            y: -self.y,
        }
    }
}

pub trait CrossProduct<RHS> {
    type Output;
    fn cross_product(self, rhs: RHS) -> Self::Output;
}

impl<T: Float> CrossProduct<Vector2d<T>> for Vector2d<T> {
    type Output = T;
    fn cross_product(self, rhs: Vector2d<T>) -> T {
        self.x * rhs.y - self.y * rhs.x
    }
}

impl<T: Float> CrossProduct<T> for Vector2d<T> {
    type Output = Vector2d<T>;
    fn cross_product(self, rhs: T) -> Vector2d<T> {
        Vector2d::new(rhs * self.y, -rhs * self.x)
    }
}

impl<T: Float> CrossProduct<Vector2d<T>> for T {
    type Output = Vector2d<T>;
    fn cross_product(self, rhs: Vector2d<T>) -> Vector2d<T> {
        Vector2d::new(-self * rhs.y, self * rhs.x)
    }
}
