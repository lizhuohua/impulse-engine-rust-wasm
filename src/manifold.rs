use body::*;
use math::*;

use std::cell::RefCell;
use std::rc::Rc;

use std::f64::NEG_INFINITY;

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
    pub fn position_correction(&mut self) {
        let mut object_a = self.object_a.borrow_mut();
        let mut object_b = self.object_b.borrow_mut();
        let k_slop = 0.05; // Penetration allowance
        let percent = 0.4; // Penetration percentage to correct
        let correction = self.normal
            * percent
            * ((self.penetration - k_slop).max(0.0)
                / (object_a.inverse_inertia + object_b.inverse_inertia));
        object_a.position -= correction * object_a.inverse_mass;
        object_b.position += correction * object_b.inverse_mass;
    }

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
            console!(
                log,
                "tangent_impulse: %f, %f",
                tangent_impulse.x,
                tangent_impulse.y
            );

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

    fn circle_to_polygon(a: &Circle, b: &Polygon) -> Option<Manifold> {
        let object_a = a.object.borrow_mut();
        let object_b = b.object.borrow_mut();

        let mixed_restitution = object_a.restitution.min(object_b.restitution);
        let mixed_static_friction = (object_a.static_friction * object_b.static_friction).sqrt();
        let mixed_dynamic_friction = (object_a.dynamic_friction * object_b.dynamic_friction).sqrt();

        // Transform circle center to polygon model space
        let mut center = object_a.position - object_b.position;
        center.rotate(-object_b.orient);

        // Find edge with minimum penetration
        let mut separation = NEG_INFINITY;
        let mut face_normal = 0;
        let count = b.vertices.len();
        for i in 0..count {
            let s = b.normals[i] * (center - b.vertices[i]);
            // No collision
            if s > a.radius {
                return None;
            }
            if s > separation {
                separation = s;
                face_normal = i;
            }
        }

        // Check to see if center is within polygon
        if separation < 0.01 {
            let mut normal = -(b.normals[face_normal]);
            normal.rotate(object_b.orient);
            return Some(Manifold {
                object_a: a.object.clone(),
                object_b: b.object.clone(),
                penetration: a.radius,
                normal: normal,
                contacts: vec![normal * a.radius + object_a.position],
                mixed_restitution: mixed_restitution,
                mixed_dynamic_friction: mixed_dynamic_friction,
                mixed_static_friction: mixed_static_friction,
            });
        }

        // Grab face's vertices
        let v1 = b.vertices[face_normal];
        let i2 = if face_normal + 1 < count {
            face_normal + 1
        } else {
            0
        };
        let v2 = b.vertices[i2];

        // Determine which region of the edge center of circle lies within
        let dot1 = (center - v1) * (v2 - v1);
        let dot2 = (center - v2) * (v1 - v2);
        let penetration = a.radius - separation;

        // Closest to v1
        if dot1 <= 0.0 {
            if (center - v1).len() > a.radius {
                return None;
            }
            let mut normal = v1 - center;
            normal.rotate(object_b.orient);
            normal.normalize();
            let mut contact = v1;
            contact.rotate(object_b.orient);
            contact += object_b.position;
            return Some(Manifold {
                object_a: a.object.clone(),
                object_b: b.object.clone(),
                penetration: penetration,
                normal: normal,
                contacts: vec![contact],
                mixed_restitution: mixed_restitution,
                mixed_dynamic_friction: mixed_dynamic_friction,
                mixed_static_friction: mixed_static_friction,
            });
        }
        // Closest to v2
        else if dot2 <= 0.0 {
            if (center - v2).len() > a.radius {
                return None;
            }
            let mut normal = v2 - center;
            normal.rotate(object_b.orient);
            normal.normalize();
            let mut contact = v2;
            contact.rotate(object_b.orient);
            contact += object_b.position;
            return Some(Manifold {
                object_a: a.object.clone(),
                object_b: b.object.clone(),
                penetration: penetration,
                normal: normal,
                contacts: vec![contact],
                mixed_restitution: mixed_restitution,
                mixed_dynamic_friction: mixed_dynamic_friction,
                mixed_static_friction: mixed_static_friction,
            });
        } else {
            let mut normal = b.normals[face_normal];
            normal.rotate(object_b.orient);
            normal = -normal;
            //if (center-v1)*n>a.radius{
            //return None;
            //}
            return Some(Manifold {
                object_a: a.object.clone(),
                object_b: b.object.clone(),
                penetration: penetration,
                normal: normal,
                contacts: vec![normal * a.radius + object_a.position],
                mixed_restitution: mixed_restitution,
                mixed_dynamic_friction: mixed_dynamic_friction,
                mixed_static_friction: mixed_static_friction,
            });
        }
    }

    fn polygon_to_circle(a: &Polygon, b: &Circle) -> Option<Manifold> {
        match Self::circle_to_polygon(b, a) {
            Some(mut m) => {
                m.normal = -m.normal;
                Some(m)
            }
            None => None,
        }
    }

    // If there is a collision, return the manifold, otherwise return None
    pub fn solve_collision<'b>(a: &Box<RigidBody + 'b>, b: &Box<RigidBody + 'b>) -> Option<Self> {
        if let Some(circle_a) = a.downcast_ref::<Circle>() {
            if let Some(circle_b) = b.downcast_ref::<Circle>() {
                Self::circle_to_circle(circle_a, circle_b)
            } else if let Some(polygon_b) = b.downcast_ref::<Polygon>() {
                Self::circle_to_polygon(circle_a, polygon_b)
            } else {
                panic!("Unknown RigidBody.");
            }
        } else if let Some(polygon_a) = a.downcast_ref::<Polygon>() {
            if let Some(circle_b) = b.downcast_ref::<Circle>() {
                Self::polygon_to_circle(polygon_a, circle_b)
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
