use body::*;
use math::*;
use scene::*;

use std::cell::RefCell;
use std::rc::Rc;

struct Face {
    v1: Vector2d<f64>,
    v2: Vector2d<f64>,
    normal: Vector2d<f64>,
}

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
    pub fn draw(&self, canvas: &mut Canvas) {
        canvas.context.set_stroke_style_color("red");
        canvas.context.set_fill_style_color("red");
        for contact in &self.contacts {
            canvas.context.fill_rect(
                contact.x * canvas.scaled_width,
                contact.y * canvas.scaled_height,
                0.08 * canvas.scaled_width,
                0.08 * canvas.scaled_width,
            );
        }
        //canvas.context.set_stroke_style_color("green");
        //let n = self.normal;
        //for contact in &self.contacts {
            //canvas.context.begin_path();
            //let b = Vector2d::new(
                //contact.x * canvas.scaled_width,
                //contact.y * canvas.scaled_height,
            //) + (-n) * 0.5 * canvas.scaled_width;
            //canvas.context.move_to(b.x, b.y);
            //let e = Vector2d::new(
                //contact.x * canvas.scaled_width,
                //contact.y * canvas.scaled_height,
            //) + n * 0.5 * canvas.scaled_width;
            //canvas.context.line_to(e.x, e.y);
            //canvas.context.stroke();
        //}
    }
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
            t = t.normalize();

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
        let object_a = a.object.borrow();
        let object_b = b.object.borrow();
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
        let object_a = a.object.borrow();
        let object_b = b.object.borrow();

        let mixed_restitution = object_a.restitution.min(object_b.restitution);
        let mixed_static_friction = (object_a.static_friction * object_b.static_friction).sqrt();
        let mixed_dynamic_friction = (object_a.dynamic_friction * object_b.dynamic_friction).sqrt();

        // Transform circle center to polygon model space
        let center = (object_a.position - object_b.position).rotate(-object_b.orient);

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
            let normal = (-b.normals[face_normal]).rotate(object_b.orient);
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
            let normal = (v1 - center).rotate(object_b.orient).normalize();
            let contact = v1.rotate(object_b.orient) + object_b.position;
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
            let normal = (v2 - center).rotate(object_b.orient).normalize();
            let contact = v2.rotate(object_b.orient) + object_b.position;
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
            let normal = -(b.normals[face_normal].rotate(object_b.orient));
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
        Self::circle_to_polygon(b, a)
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
                Self::polygon_to_polygon(polygon_a, polygon_b)
            } else {
                panic!("Unknown RigidBody.");
            }
        } else {
            panic!("Unknown RigidBody.");
        }
    }

    fn polygon_to_polygon(a: &Polygon, b: &Polygon) -> Option<Manifold> {
        let object_a = a.object.borrow();
        let object_b = b.object.borrow();

        let mixed_restitution = object_a.restitution.min(object_b.restitution);
        let mixed_static_friction = (object_a.static_friction * object_b.static_friction).sqrt();
        let mixed_dynamic_friction = (object_a.dynamic_friction * object_b.dynamic_friction).sqrt();

        let (faceA, penetrationA) = Self::find_axis_least_penetration(a, b);
        if penetrationA >= 0.0 {
            return None;
        }
        let (faceB, penetrationB) = Self::find_axis_least_penetration(b, a);
        if penetrationB >= 0.0 {
            return None;
        }

        let ref_poly;
        let inc_poly;
        let ref_object;
        let inc_object;
        let ref_face;
        let flip;

        if penetrationA >= penetrationB {
            ref_poly = a;
            inc_poly = b;
            ref_object = object_a;
            inc_object = object_b;
            ref_face = faceA;
            flip = false;
        } else {
            ref_poly = b;
            inc_poly = a;
            ref_object = object_b;
            inc_object = object_a;
            ref_face = faceB;
            flip = true;
        }

        let inc_face = Self::find_incident_face(a, b, &ref_face);

        let v1 = ref_face.v1.rotate(ref_object.orient) + ref_object.position;
        let v2 = ref_face.v2.rotate(ref_object.orient) + ref_object.position;

        let side_plane_normal = (v2 - v1).normalize();
        let ref_face_normal = Vector2d::new(side_plane_normal.y, -side_plane_normal.x);

        let ref_C = ref_face_normal * v1;
        let neg_side = -side_plane_normal * v1;
        let pos_side = side_plane_normal * v2;

        let mut clipped_face = match Self::clip(-side_plane_normal, neg_side, inc_face) {
            None => return None,
            Some(face) => face,
        };
        clipped_face = match Self::clip(side_plane_normal, pos_side, clipped_face) {
            None => return None,
            Some(face) => face,
        };

        let normal = if flip {
            -ref_face_normal
        } else {
            ref_face_normal
        };
        let mut contacts = Vec::new();
        let mut penetration = 0.0;

        let mut separation = (ref_face_normal * clipped_face.v1) - ref_C;
        if separation <= 0.0 {
            contacts.push(clipped_face.v1);
            penetration = -separation;
        }

        separation = (ref_face_normal * clipped_face.v2) - ref_C;
        if separation <= 0.0 {
            contacts.push(clipped_face.v2);
            penetration += -separation;
        }
        penetration /= contacts.len() as f64;
        Some(Manifold {
            object_a: a.object.clone(),
            object_b: b.object.clone(),
            penetration: penetration,
            normal: normal,
            contacts: contacts,
            mixed_restitution: mixed_restitution,
            mixed_dynamic_friction: mixed_dynamic_friction,
            mixed_static_friction: mixed_static_friction,
        })
    }

    fn clip(normal: Vector2d<f64>, c: f64, face: Face) -> Option<Face> {
        let mut out = Vec::new();
        let d1 = normal * face.v1 - c;
        let d2 = normal * face.v2 - c;
        if d1 <= 0.0 {
            out.push(face.v1);
        }
        if d2 <= 0.0 {
            out.push(face.v2);
        }
        if d1 * d2 < 0.0 {
            out.push(face.v1 + (face.v2 - face.v1) * (d1 / (d1 - d2)));
        }
        if out.len() != 2 {
            None
        } else {
            Some(Face {
                v1: out[0],
                v2: out[1],
                normal: Vector2d::new(0.0, 0.0),
            })
        }
    }

    fn find_incident_face(ref_poly: &Polygon, inc_poly: &Polygon, ref_face: &Face) -> Face {
        let ref_object = ref_poly.object.borrow();
        let inc_object = inc_poly.object.borrow();
        let ref_normal = ref_face
            .normal
            .rotate(ref_object.orient)
            .rotate(-inc_object.orient);

        let mut inc_face_index = 0;
        let mut min_dot = INFINITY;
        for (i, &normal) in inc_poly.normals.iter().enumerate() {
            let dot = ref_normal * normal;
            if dot < min_dot {
                min_dot = dot;
                inc_face_index = i;
            }
        }

        let v1 = inc_poly.vertices[inc_face_index].rotate(inc_object.orient) + inc_object.position;
        let inc_face_index2 = if inc_face_index + 1 < inc_poly.vertices.len() {
            inc_face_index + 1
        } else {
            0
        };
        let v2 = inc_poly.vertices[inc_face_index2].rotate(inc_object.orient) + inc_object.position;
        Face {
            v1: v1,
            v2: v2,
            normal: inc_poly.normals[inc_face_index].rotate(inc_object.orient),
        }
    }

    fn find_axis_least_penetration(a: &Polygon, b: &Polygon) -> (Face, f64) {
        let mut best_distance = NEG_INFINITY;
        let mut face_index = 0;
        let object_a = a.object.borrow();
        let object_b = b.object.borrow();
        for (i, vertex) in a.vertices.iter().enumerate() {
            let n = a.normals[i]
                .rotate(object_a.orient)
                .rotate(-object_b.orient);
            let s = b.get_support(-n);

            let v = (vertex.rotate(object_a.orient) + object_a.position - object_b.position)
                .rotate(-object_b.orient);

            let d = n * (s - v);
            if d > best_distance {
                best_distance = d;
                face_index = i;
            }
        }
        let face_index2 = if face_index + 1 < a.vertices.len() {
            face_index + 1
        } else {
            0
        };
        (
            Face {
                v1: a.vertices[face_index],
                v2: a.vertices[face_index2],
                normal: a.normals[face_index],
            },
            best_distance,
        )
    }
}
