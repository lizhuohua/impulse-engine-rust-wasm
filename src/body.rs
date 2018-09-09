use math::*;
use rand::*;
use scene::*;
use std::cell::RefCell;
use std::f64::consts::PI;
use std::rc::Rc;

use downcast_rs::Downcast;

const GRAVITY: Vector2d<f64> = Vector2d { x: 0.0, y: 9.8 };

#[derive(Clone)]
pub struct Color {
    r: u8,
    g: u8,
    b: u8,
}

#[derive(Clone)]
pub struct Object {
    pub position: Vector2d<f64>,
    pub velocity: Vector2d<f64>,
    pub force: Vector2d<f64>,

    pub density: f64,
    pub inertia: f64,
    pub inverse_inertia: f64,
    pub mass: f64,
    pub inverse_mass: f64,

    pub angular_velocity: f64,
    pub torque: f64,
    pub orient: f64,

    pub static_friction: f64,
    pub dynamic_friction: f64,
    pub restitution: f64,

    pub color: Color,
}

impl Object {
    fn new(x: f64, y: f64) -> Self {
        let mut rng = Rng::new();
        Self {
            position: Vector2d::new(x, y),
            velocity: Vector2d::zero(),
            force: Vector2d::zero(),
            density: 1.0,
            inertia: 1.0,
            inverse_inertia: 1.0,
            mass: 1.0,
            inverse_mass: 1.0,
            angular_velocity: 0.0,
            torque: 0.0,
            orient: 0.0,
            static_friction: 0.5,
            dynamic_friction: 0.3,
            restitution: 0.2,
            color: Color {
                r: rng.gen_range(0, 200) as u8,
                g: rng.gen_range(0, 200) as u8,
                b: rng.gen_range(0, 200) as u8,
            },
        }
    }

    //fn apply_force(&mut self, f: Vector2d<f64>) {
    //self.force += f;
    //}

    pub fn apply_impulse(&mut self, impulse: Vector2d<f64>, contact_vector: Vector2d<f64>) {
        self.velocity += impulse * self.inverse_mass;
        self.angular_velocity += contact_vector.cross_product(impulse) * self.inverse_inertia;
        // console!(log, "angular_velocity: %f", self.angular_velocity);
    }

    fn set_static(&mut self) {
        self.inertia = INFINITY;
        self.inverse_inertia = 0.0;
        self.mass = INFINITY;
        self.inverse_mass = 0.0;
    }

    fn integrate_forces(&mut self, dt: f64) {
        if self.inverse_mass != 0.0 {
            self.velocity += (self.force * self.inverse_mass + GRAVITY) * (dt);
            self.angular_velocity += self.torque * self.inverse_inertia * dt;
        }
    }

    fn integrate_velocity(&mut self, dt: f64) {
        if self.inverse_mass != 0.0 {
            self.position += self.velocity * dt;
            self.orient += self.angular_velocity * dt;
            //self.integrate_forces(dt);
        }
    }
}

pub trait RigidBody: Downcast {
    fn draw(&self, canvas: &mut Canvas);

    fn integrate_forces(&mut self, dt: f64);

    fn integrate_velocity(&mut self, dt: f64);

    fn clear_forces(&mut self);

    fn object(&self) -> Rc<RefCell<Object>>;

    fn radius(&self) -> f64;
}
impl_downcast!(RigidBody);

pub struct Circle {
    pub radius: f64,
    pub object: Rc<RefCell<Object>>,
}

impl RigidBody for Circle {
    fn radius(&self) -> f64 {
        return self.radius;
    }
    fn clear_forces(&mut self) {
        self.object.borrow_mut().force.set(0.0, 0.0);
        self.object.borrow_mut().torque = 0.0;
    }
    fn object(&self) -> Rc<RefCell<Object>> {
        self.object.clone()
    }
    fn integrate_forces(&mut self, dt: f64) {
        self.object.borrow_mut().integrate_forces(dt);
    }

    fn integrate_velocity(&mut self, dt: f64) {
        self.object.borrow_mut().integrate_velocity(dt);
    }
    fn draw(&self, canvas: &mut Canvas) {
        let object = self.object.borrow();
        // Handle scale
        let mut position = object.position;
        position.x *= canvas.scaled_width;
        position.y *= canvas.scaled_height;
        // console!(log, "draw a circle at %f, %f", position.x,position.y);
        let radius = self.radius * canvas.scaled_width;

        let k_segments = 30;
        let mut theta = 0.0;
        let inc = 2.0 * PI / k_segments as f64;
        let mut begin = Vector2d::new(theta.cos(), theta.sin());
        begin = begin * radius;
        begin += position;
        canvas.context.begin_path();
        let color = format!(
            "#{:02x}{:02x}{:02x}",
            object.color.r, object.color.g, object.color.b
        );
        canvas.context.set_stroke_style_color(&color);
        canvas.context.move_to(begin.x, begin.y);
        for _ in 0..k_segments {
            theta += inc;
            let mut point = Vector2d::new(theta.cos(), theta.sin());
            point = point * radius;
            point += position;
            canvas.context.line_to(point.x, point.y);
        }
        let r = Vector2d::new(0.0, radius).rotate(object.orient) + position;
        canvas.context.move_to(position.x, position.y);
        canvas.context.line_to(r.x, r.y);
        canvas.context.stroke();
    }
}

impl Circle {
    pub fn new(x: f64, y: f64, r: f64) -> Circle {
        let mut c = Circle {
            radius: r,
            object: Rc::new(RefCell::new(Object::new(x, y))),
        };
        c.initialize();
        c
    }
    fn initialize(&mut self) {
        let mut object = self.object.borrow_mut();
        object.mass = PI * self.radius * self.radius * object.density;
        object.inverse_mass = 1.0 / object.mass;
        object.inertia = object.mass * self.radius * self.radius / 2.0;
        object.inverse_inertia = 1.0 / object.inertia;
    }
    pub fn set_static(&mut self) {
        self.object.borrow_mut().set_static();
    }
}

pub struct Polygon {
    pub radius: f64,
    pub vertices: Vec<Vector2d<f64>>,
    pub normals: Vec<Vector2d<f64>>,
    pub object: Rc<RefCell<Object>>,
}

impl Polygon {
    pub fn get_support(&self, direction: Vector2d<f64>) -> Vector2d<f64> {
        let mut best_projection = NEG_INFINITY;
        let mut best_vertex = Vector2d::zero();
        for &v in &self.vertices {
            let projection = v * direction;
            if projection > best_projection {
                best_vertex = v;
                best_projection = projection;
            }
        }
        best_vertex
    }

    fn initialize(&mut self) {
        let mut object = self.object.borrow_mut();
        // Calculate face normals
        self.normals.clear();
        let n = self.vertices.len();
        for i1 in 0..n {
            let i2 = if i1 + 1 < n { i1 + 1 } else { 0 };
            let face = self.vertices[i2] - self.vertices[i1];
            let normal = Vector2d::new(face.y, -face.x).normalize();
            self.normals.push(normal);
        }

        // Calculate centroid and moment of interia
        let mut centroid = Vector2d::zero();
        let mut area = 0.0;
        let mut inertia = 0.0;
        let n = self.vertices.len();
        for i1 in 0..n {
            // Triangle vertices, the third vertex is (0, 0)
            let p1 = self.vertices[i1];
            let i2 = if i1 + 1 < n { i1 + 1 } else { 0 };
            let p2 = self.vertices[i2];
            let triangle_area = 0.5 * p1.cross_product(p2).abs();
            area += triangle_area;
            centroid += (p1 + p2) * (1.0 / 3.0 * triangle_area);
            inertia +=
                triangle_area * object.density * (p1.len_square() + p2.len_square() + p1 * p2)
                    / 6.0;
        }
        centroid /= area;
        // console!(log, "centroid: %f, %f", centroid.x, centroid.y);

        // Make the centroid (0, 0)
        for v in &mut self.vertices {
            *v -= centroid;
        }

        // Calculate mass and interia
        object.mass = area * object.density;
        object.inverse_mass = 1.0 / object.mass;
        object.inertia = inertia - object.mass * centroid.len_square();
        // console!(log, "inertia: %f", object.inertia);
        object.inverse_inertia = 1.0 / object.inertia;
    }

    pub fn set_static(&mut self) {
        self.object.borrow_mut().set_static();
    }

    pub fn set_vertices(&mut self, vertices: &Vec<Vector2d<f64>>) {
        self.vertices = vertices.clone();
        self.initialize();
    }

    pub fn new(x: f64, y: f64, r: f64) -> Self {
        let mut rng = Rng::new();
        let count = rng.gen_range(3, 64);
        let mut vertices = Vec::new();
        for _ in 0..count {
            let x = (rng.gen_range(0, 20000) - 10000) as f64 / 10000.0 * r as f64;
            let y = (rng.gen_range(0, 20000) - 10000) as f64 / 10000.0 * r as f64;
            vertices.push(Vector2d::new(x, y));
        }
        // Test these vertices to ensure it is really a convex polygon
        let mut right_most = vertices[0];
        for &v in &vertices {
            if v.x > right_most.x {
                right_most = v;
            } else if v.x == right_most.x {
                if v.y < right_most.y {
                    right_most = v;
                }
            }
        }
        let mut result_vertices = Vec::new();
        result_vertices.push(right_most);
        let mut next_index = vertices[0].clone();
        let mut index = right_most;
        loop {
            for &v in &vertices {
                if next_index == index {
                    next_index = v;
                }
                let e1 = next_index - index;
                let e2 = v - index;
                let c = e1.cross_product(e2);
                if c < 0.0 {
                    next_index = v;
                } else if c == 0.0 && e2.len_square() > e1.len_square() {
                    next_index = v;
                }
            }

            if next_index == right_most {
                break;
            }

            result_vertices.push(next_index);
            index = next_index;
        }

        let mut polygon = Self {
            radius: r * 1.5,
            vertices: result_vertices,
            normals: Vec::new(),
            object: Rc::new(RefCell::new(Object::new(x, y))),
        };
        polygon.initialize();
        polygon
    }
}

impl RigidBody for Polygon {
    fn radius(&self) -> f64 {
        return self.radius;
    }
    fn clear_forces(&mut self) {
        self.object.borrow_mut().force.set(0.0, 0.0);
        self.object.borrow_mut().torque = 0.0;
    }
    fn object(&self) -> Rc<RefCell<Object>> {
        self.object.clone()
    }
    fn integrate_forces(&mut self, dt: f64) {
        self.object.borrow_mut().integrate_forces(dt);
    }

    fn integrate_velocity(&mut self, dt: f64) {
        self.object.borrow_mut().integrate_velocity(dt);
    }
    fn draw(&self, canvas: &mut Canvas) {
        let object = self.object.borrow();
        let mut position = object.position;
        position.x *= canvas.scaled_width;
        position.y *= canvas.scaled_height;

        let begin = (self.vertices[0] * canvas.scaled_width).rotate(object.orient) + position;
        canvas.context.begin_path();
        let color = format!(
            "#{:02x}{:02x}{:02x}",
            object.color.r, object.color.g, object.color.b
        );
        canvas.context.set_stroke_style_color(&color);
        canvas.context.move_to(begin.x, begin.y);
        for &v in &self.vertices {
            let point = (v * canvas.scaled_width).rotate(object.orient) + position;
            canvas.context.line_to(point.x, point.y);
        }
        canvas.context.close_path();
        canvas.context.stroke();

        canvas.context.fill_rect(
            position.x,
            position.y,
            0.05 * canvas.scaled_width,
            0.05 * canvas.scaled_width,
        );
    }
}
