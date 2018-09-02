use body::*;
use math::*;

use std::cell::RefCell;
use std::rc::Rc;

pub struct Manifold {
    object_a: Rc<RefCell<Object>>,
    object_b: Rc<RefCell<Object>>,
    penetration: f64,
    pub normal: Vector2d<f64>,
    pub contacts: Vec<Vector2d<f64>>,
    mixed_restitution: f64,
    mixed_dynamic_friction: f64,
    mixed_static_friction: f64,
}

impl Manifold {
    pub fn apply_impulse(&mut self) {
        let mut object_a = self.object_a.borrow_mut();
        let mut object_b = self.object_b.borrow_mut();
        if object_a.inverse_mass + object_b.inverse_mass < 0.01 {
            object_a.velocity.set(0.0, 0.0);
            object_b.velocity.set(0.0, 0.0);
        }

        for &contact in &self.contacts {
            // Vectors from center of mas to contact point
            let r_a = contact - object_a.position;
            let r_b = contact - object_b.position;

            // Relative velocity at contact point
            let v_ab = object_b.velocity + object_b.angular_velocity.cross_product(r_b)
                - object_a.velocity
                - object_a.angular_velocity.cross_product(r_a);

            // Relative velocity along the normal
            let contact_velocity = v_ab * self.normal;

            // Do not resolve if velocities are separating
            if contact_velocity > 0.0 {
                return;
            }
            let ra_cross_n = r_a.cross_product(self.normal);
            let rb_cross_n = r_b.cross_product(self.normal);
            let inv_mass_inertia = object_a.inverse_mass
                + object_b.inverse_mass
                + ra_cross_n * ra_cross_n * object_a.inverse_inertia
                + rb_cross_n * rb_cross_n * object_b.inverse_inertia;

            let j = -(1.0 + self.mixed_restitution) * contact_velocity
                / inv_mass_inertia
                / self.contacts.len() as f64;

            let impulse = self.normal * j;
            object_a.apply_impulse(-impulse, r_a);
            object_b.apply_impulse(impulse, r_b);

            // Friction impulse
            let mut t = v_ab - (self.normal * (v_ab * self.normal));
            if t.len() < 0.01 {
                return;
            }
            t.normalize();

            // j tangent magnitude
            let jt = -v_ab * t / inv_mass_inertia / self.contacts.len() as f64;
            console!(log, "jt: %f", jt);
            if jt.abs() < 0.01 {
                return;
            }

            // Coulumb's law
            let tangent_impulse;
            if jt.abs() < j * self.mixed_static_friction {
                tangent_impulse = t * jt;
            } else {
                tangent_impulse = t * (-j) * self.mixed_dynamic_friction;
            }
            console!(log, "tangent_impulse: %f, %f", tangent_impulse.x, tangent_impulse.y);

            object_a.apply_impulse(-tangent_impulse, r_a);
            object_b.apply_impulse(tangent_impulse, r_b);
        }
    }
    fn circle_to_circle(a: &Circle, b: &Circle) -> Option<Manifold> {
        let object_a = a.object.borrow_mut();
        let object_b = b.object.borrow_mut();
        let mixed_restitution = object_a.restitution.min(object_b.restitution);
        let mixed_static_friction = (object_a.static_friction * object_b.static_friction).sqrt();
        let mixed_dynamic_friction = (object_a.dynamic_friction * object_b.dynamic_friction).sqrt();
        let normal = object_b.position - object_a.position;
        let radius_sum = a.radius + b.radius;
        if normal.len_square() >= radius_sum * radius_sum {
            return None;
        }
        let distance = normal.len();
        if distance == 0.0 {
            Some(Manifold {
                object_a: a.object.clone(),
                object_b: b.object.clone(),
                penetration: a.radius,
                normal: Vector2d::new(1.0, 0.0),
                contacts: vec![object_a.position],
                mixed_restitution: mixed_restitution,
                mixed_dynamic_friction: mixed_dynamic_friction,
                mixed_static_friction: mixed_static_friction,
            })
        } else {
            Some(Manifold {
                object_a: a.object.clone(),
                object_b: b.object.clone(),
                penetration: radius_sum - distance,
                normal: normal / distance,
                contacts: vec![normal / distance * a.radius + object_a.position],
                mixed_restitution: mixed_restitution,
                mixed_dynamic_friction: mixed_dynamic_friction,
                mixed_static_friction: mixed_static_friction,
            })
        }
    }

    // If there is a collision, return the manifold, otherwise return None
    pub fn solve_collision<'b>(a: &Box<RigidBody + 'b>, b: &Box<RigidBody + 'b>) -> Option<Self> {
        if let Some(circle_a) = a.downcast_ref::<Circle>() {
            if let Some(circle_b) = b.downcast_ref::<Circle>() {
                Self::circle_to_circle(circle_a, circle_b)
            } else if let Some(polygon_b) = b.downcast_ref::<Polygon>() {
                //Self::circle_to_polygon(circle_a, polygon_b)
                None
            } else {
                panic!("Unknown RigidBody.");
            }
        } else if let Some(polygon_a) = a.downcast_ref::<Polygon>() {
            if let Some(circle_b) = b.downcast_ref::<Circle>() {
                //Self::polygon_to_circle(polygon_a, circle_b)
                None
            } else if let Some(polygon_b) = b.downcast_ref::<Polygon>() {
                //Self::polygon_to_polygon(polygon_a, polygon_b)
                None
            } else {
                panic!("Unknown RigidBody.");
            }
        } else {
            panic!("Unknown RigidBody.");
        }
    }
}
