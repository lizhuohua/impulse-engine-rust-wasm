use body::*;
use manifold::*;
use math::*;
use rand::*;
use stdweb::traits::*;
use stdweb::unstable::TryInto;
use stdweb::web::html_element::CanvasElement;
use stdweb::web::{document, CanvasRenderingContext2d};

pub struct Canvas {
    pub canvas: CanvasElement,
    pub context: CanvasRenderingContext2d,
    pub scaled_width: f64,
    pub scaled_height: f64,

    width: f64,
    height: f64,
}

impl Canvas {
    pub fn new(attr_id: &str, width: f64, height: f64) -> Self {
        let document = document();
        let canvas: CanvasElement = document
            .query_selector(attr_id)
            .unwrap()
            .unwrap()
            .try_into()
            .unwrap();
        let context: CanvasRenderingContext2d = canvas.get_context().unwrap();
        let scaled_width = canvas.width() as f64 / width;
        let scaled_height = canvas.height() as f64 / height;
        Self {
            canvas,
            context,
            scaled_width,
            scaled_height,
            width,
            height,
        }
    }
}

pub struct Scene {
    canvas: Canvas,
    m_dt: f64,
    m_iterations: u32,
    bodies: Vec<Box<dyn RigidBody>>,
    contacts: Vec<Manifold>,
    rng: Rng,
}

impl Scene {
    pub fn canvas(&self) -> &CanvasElement {
        &self.canvas.canvas
    }
    pub fn resize(&mut self, height: u32, width: u32) {
        let h = ::std::cmp::min(800, height);
        let w = ::std::cmp::min(800, width);
        self.canvas.canvas.set_height(h);
        self.canvas.canvas.set_width(w);
        self.canvas.scaled_width = w as f64 / self.canvas.width;
        self.canvas.scaled_height = h as f64 / self.canvas.height;
    }
    pub fn new() -> Scene {
        let canvas = Canvas::new("#canvas", 20.0, 20.0);
        let mut bodies = Vec::new();
        let mut fixed_circle = Circle::new(10.0, 10.0, 1.0);
        fixed_circle.set_static();
        bodies.push(Box::new(fixed_circle) as Box<dyn RigidBody>);

        let mut fixed_rectangle = Polygon::new(10.0, 17.0, 18.0);
        let top_right = Vector2d::new(9.0, -0.5);
        let top_left = Vector2d::new(-9.0, -0.5);
        let bottom_left = Vector2d::new(-9.0, 0.5);
        let bottom_right = Vector2d::new(9.0, 0.5);
        let rectangle_vertices = vec![top_left, top_right, bottom_right, bottom_left];
        fixed_rectangle.set_vertices(&rectangle_vertices);
        fixed_rectangle.set_static();
        bodies.push(Box::new(fixed_rectangle) as Box<dyn RigidBody>);

        Scene {
            canvas: canvas,
            m_dt: 1.0 / 60.0,
            m_iterations: 10,
            bodies: bodies,
            contacts: Vec::new(),
            rng: Rng::new(),
        }
    }
    pub fn add_circle(&mut self, x: f64, y: f64) {
        let c = Circle::new(
            x / self.canvas.scaled_width,
            y / self.canvas.scaled_height,
            // Random float from 0.3~0.8
            self.rng.gen_range(0, 10000) as f64 / 20000.0 + 0.3,
        );
        self.bodies.push(Box::new(c) as Box<dyn RigidBody>);
    }
    pub fn add_polygon(&mut self, x: f64, y: f64) {
        let p = Polygon::new(
            x / self.canvas.scaled_width,
            y / self.canvas.scaled_height,
            // Random float from 0.5~1.5
            self.rng.gen_range(0, 10000) as f64 / 10000.0 + 0.5,
        );
        self.bodies.push(Box::new(p));
    }
    fn render_string(&mut self, text: &str, x: f64, y: f64, max_width: f64) {
        self.canvas.context.set_font("20px sans-serif");
        self.canvas.context.set_fill_style_color("black");
        self.canvas.context.fill_text(
            text,
            x * self.canvas.scaled_width,
            y * self.canvas.scaled_height,
            Some(max_width * self.canvas.scaled_width),
        );
    }
    pub fn render(&mut self) {
        // Clear canvas
        self.canvas.context.clear_rect(
            0.0,
            0.0,
            self.canvas.canvas.width() as f64,
            self.canvas.canvas.height() as f64,
        );

        // Draw texts
        self.render_string("Left click to spawn a polygon.", 0.5, 1.0, 9.0);
        self.render_string("Right click to spawn a circle.", 0.5, 2.0, 9.0);

        // Draw rigid bodies
        for body in &self.bodies {
            body.draw(&mut self.canvas);
        }

        // Draw manifolds
        for manifold in &self.contacts {
            manifold.draw(&mut self.canvas);
        }
    }
    pub fn step(&mut self) {
        // Generate new collision info
        self.contacts.clear();
        for (i, body_a) in self.bodies.iter().enumerate() {
            for body_b in self.bodies.iter().skip(i + 1) {
                let object_a = body_a.object();
                let object_b = body_b.object();
                let r1 = body_a.radius() * self.canvas.scaled_width;
                let r2 = body_b.radius() * self.canvas.scaled_width;
                if (object_a.borrow().position - object_b.borrow().position).len_square() > r1 * r2
                {
                    continue;
                }
                if object_a.borrow().inverse_mass == 0.0 && object_b.borrow().inverse_mass == 0.0 {
                    continue;
                }
                if let Some(m) = Manifold::solve_collision(body_a, body_b) {
                    self.contacts.push(m);
                }
            }
        }

        // Integrate forces
        for body in &mut self.bodies {
            body.integrate_forces(self.m_dt);
        }

        // Initialize collision
        for _ in 0..self.m_iterations {
            for contact in &mut self.contacts {
                contact.apply_impulse();
            }
        }

        // Integrate velocities
        for body in &mut self.bodies {
            body.integrate_velocity(self.m_dt);
        }

        // Correct positions
        for contact in &mut self.contacts {
            contact.position_correction();
        }
    }
}
