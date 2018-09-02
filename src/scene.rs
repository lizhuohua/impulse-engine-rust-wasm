use body::*;
use manifold::*;
use math::*;
use rand::*;
use stdweb::traits::*;
use stdweb::unstable::{TryFrom, TryInto};
use stdweb::web::html_element::{CanvasElement, ImageElement};
use stdweb::web::{document, CanvasRenderingContext2d, Element};

pub struct Canvas {
    pub canvas: CanvasElement,
    pub context: CanvasRenderingContext2d,
    //pub offscreen_canvas: CanvasElement,
    //pub offscreen_context: CanvasRenderingContext2d,

    // actual size / 20
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
        //let offscreen_canvas: CanvasElement = document
        //.create_element("canvas")
        //.unwrap()
        //.try_into()
        //.unwrap();
        //offscreen_canvas.set_width(canvas.width());
        //offscreen_canvas.set_height(canvas.height());
        //let offscreen_context: CanvasRenderingContext2d = offscreen_canvas.get_context().unwrap();
        Self {
            canvas,
            context,
            //offscreen_canvas,
            //offscreen_context,
            scaled_width,
            scaled_height,
            width,
            height,
        }
    }
    pub fn clear(&mut self) {
        self.context.set_fill_style_color("white");
        self.context.fill_rect(
            0.0,
            0.0,
            self.width * self.scaled_width,
            self.height * self.scaled_height,
        );
    }
    //pub fn present(&mut self) {
    //let e: Element = self.offscreen_canvas.clone().into();
    //let image: ImageElement = e.try_into().unwrap();
    //self.context.draw_image(image,0.0,0.0).unwrap();
    //}
}

pub struct Scene {
    pub canvas: Canvas,
    m_dt: f64,
    m_iterations: u32,
    bodies: Vec<Box<dyn RigidBody>>,
    contacts: Vec<Manifold>,
    rng: Rng,
}

impl Scene {
    pub fn resize(&mut self, height: u32, width: u32) {
        self.canvas.canvas.set_height(height);
        self.canvas.canvas.set_width(width);
        self.canvas.scaled_width = self.canvas.canvas.width() as f64 / self.canvas.width;
        self.canvas.scaled_height = self.canvas.canvas.height() as f64 / self.canvas.height;
    }
    pub fn new() -> Scene {
        let canvas = Canvas::new("#canvas", 20.0, 20.0);
        let mut bodies = Vec::new();
        let mut fixed_circle = Circle::new(10.0, 10.0, 1.0);
        fixed_circle.set_static();
        bodies.push(Box::new(fixed_circle) as Box<dyn RigidBody>);

        let mut fixed_rectangle = Polygon::new(10.0, 17.0, 1.0);
        let top_right = Vector2d::new(9.0, -0.5);
        let top_left = Vector2d::new(-9.0, -0.5);
        let bottom_left = Vector2d::new(-9.0, 0.5);
        let bottom_right = Vector2d::new(9.0, 0.5);
        let rectangle_vertices = vec![top_right, top_left, bottom_left, bottom_right];
        fixed_rectangle.set_vertices(&rectangle_vertices);
        fixed_rectangle.set_static();
        bodies.push(Box::new(fixed_rectangle) as Box<dyn RigidBody>);

        Scene {
            canvas: canvas,
            m_dt: 1.0 / 60.0,
            m_iterations: 0,
            bodies: bodies,
            contacts: Vec::new(),
            rng: Rng::new(),
        }
    }
    pub fn add_circle(&mut self, x: f64, y: f64) {
        let c = Circle::new(
            x / self.canvas.scaled_width,
            y / self.canvas.scaled_height,
            // Random float from 0.5~2.5
            self.rng.gen_range(0, 1000) as f64 / 500.0 + 0.5,
        );
        self.bodies.push(Box::new(c) as Box<dyn RigidBody>);
    }
    pub fn add_polygon(&mut self, x: f64, y: f64) {
        let p = Polygon::new(
            x / self.canvas.scaled_width,
            y / self.canvas.scaled_height,
            // Random float from 0.5~2.5
            self.rng.gen_range(0, 1000) as f64 / 500.0 + 0.5,
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
        self.canvas.clear();

        self.render_string("Left click to spawn a polygon.", 0.5, 1.0, 9.0);
        self.render_string("Right click to spawn a circle.", 0.5, 2.0, 9.0);
        for body in &self.bodies {
            body.draw(&mut self.canvas);
        }
        //self.canvas.present();

        // Draw manifolds
        self.canvas.context.set_stroke_style_color("red");
        self.canvas.context.set_fill_style_color("red");
        for manifold in &self.contacts {
            for contact in &manifold.contacts {
                self.canvas.context.fill_rect(
                    contact.x * self.canvas.scaled_width,
                    contact.y * self.canvas.scaled_height,
                    0.05 * self.canvas.scaled_width,
                    0.05 * self.canvas.scaled_width,
                );
            }
        }
        self.canvas.context.set_stroke_style_color("green");
        for manifold in &self.contacts {
            let n = manifold.normal;
            for contact in &manifold.contacts {
                self.canvas.context.begin_path();
                self.canvas.context.move_to(
                    contact.x * self.canvas.scaled_width,
                    contact.y * self.canvas.scaled_height,
                );
                let c = Vector2d::new(
                    contact.x * self.canvas.scaled_width,
                    contact.y * self.canvas.scaled_height,
                ) + n * 0.5 * self.canvas.scaled_width;
                self.canvas.context.line_to(c.x, c.y);
                self.canvas.context.stroke();
            }
        }
    }
    pub fn step(&mut self) {
        // Generate new collision info
        self.contacts.clear();
        for (i, body_a) in self.bodies.iter().enumerate() {
            for body_b in self.bodies.iter().skip(i + 1) {
                let object_a = body_a.object();
                let object_b = body_b.object();
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
        for _ in 0..10 {
            for contact in &mut self.contacts {
                contact.apply_impulse();
            }
        }

        // Integrate velocities
        for body in &mut self.bodies {
            body.integrate_velocity(self.m_dt);
        }

        // Clear all forces
        //for body in &mut self.bodies {
            //body.clear_forces();
        //}
    }
}
