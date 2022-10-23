use crate::objects::Drawable;
use glam::{Mat3, Mat4, Quat, Vec3, Vec4};
use std::any::Any;

trait Shape {
    fn shape_type(&self) -> ShapeEnum;
    fn color(&self) -> Vec4;
    fn radius(&self) -> f32;
    fn as_any(&self) -> &dyn Any;
    fn get_centre_of_mass(&self) -> Vec3;
    fn inertial_tensor(&self) -> Mat3;
}

#[derive(PartialEq)]
enum ShapeEnum {
    Sphere,
}

struct Sphere {
    radius: f32,
    color: Vec4,
}

impl Shape for Sphere {
    fn shape_type(&self) -> ShapeEnum {
        ShapeEnum::Sphere
    }

    fn color(&self) -> Vec4 {
        self.color
    }

    fn radius(&self) -> f32 {
        self.radius
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn get_centre_of_mass(&self) -> Vec3 {
        Vec3::ZERO
    }

    fn inertial_tensor(&self) -> Mat3 {
        let f = 2. * self.radius * self.radius() / 5.;
        Mat3::from_diagonal(Vec3::new(f, f, f))
    }
}

pub struct Body {
    pub position: Vec3,
    pub orientation: Quat,
    pub inv_mass: f32,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub elasticity: f32,
    shape: Box<dyn Shape>,
}

impl Body {
    fn radius(&self) -> f32 {
        self.shape.radius()
    }

    fn get_centre_of_mass_world_space(&self) -> Vec3 {
        let com = self.shape.get_centre_of_mass();
        self.position + self.orientation.mul_vec3(com)
    }

    fn get_centre_of_mass_model_space(&self) -> Vec3 {
        self.shape.get_centre_of_mass()
    }

    fn world_space_to_body_space(&self, world_pt: Vec3) -> Vec3 {
        self.orientation
            .inverse()
            .mul_vec3(world_pt - self.get_centre_of_mass_world_space())
    }

    fn body_space_to_world_space(&self, body_pt: Vec3) -> Vec3 {
        self.get_centre_of_mass_world_space() + self.orientation.mul_vec3(body_pt)
    }

    fn apply_impulse_linear(&mut self, impulse: Vec3) {
        if self.inv_mass == 0. {
            return;
        }
        self.linear_velocity += impulse * self.inv_mass;
    }

    fn get_inverse_inertial_tensor_body_space(&self) -> Mat3 {
        self.shape.inertial_tensor().inverse() * self.inv_mass
    }

    fn get_inverse_inertial_tensor_world_space(&self) -> Mat3 {
        let inv_t = self.get_inverse_inertial_tensor_body_space();
        let orient = Mat3::from_quat(self.orientation);
        orient * inv_t * orient.transpose()
    }

    fn apply_impluse_angular(&mut self, impulse: Vec3) {
        if self.inv_mass == 0. {
            return;
        }

        // L = I w = r x p
        // dL = I dw = r x J
        // => dw = I^-1 * ( r x J )
        self.angular_velocity += self.get_inverse_inertial_tensor_world_space() * impulse;

        // 30 rad/s is fast enough for us. But feel free to adjust.
        let max_angular_speed = 30.;
        if self.angular_velocity.length_squared() > max_angular_speed * max_angular_speed {
            self.angular_velocity = self.angular_velocity.normalize() * max_angular_speed;
        }
    }

    fn apply_impulse(&mut self, impulse_point: Vec3, impulse: Vec3) {
        if self.inv_mass == 0. {
            return;
        }
        // impulsePoint is the world space location of the application of the impulse
        // impulse is the world space direction and magnitude of the impulse
        self.apply_impulse_linear(impulse);
        // applying impulses must produce torques through the center of mass
        let r = impulse_point - self.get_centre_of_mass_world_space();
        // this is in world space
        self.apply_impluse_angular(r.cross(impulse));
    }

    fn update(&mut self, dt_sec: f32) {
        self.position += self.linear_velocity * dt_sec;

        // okay, we have an angular velocity around the center of mass, this needs to be
        // converted somehow to relative to model position.  This way we can properly update
        // the orientation of the model.
        let position_cm = self.get_centre_of_mass_world_space();
        let cm_to_pos = self.position - position_cm;

        // Total Torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 ( w x I * w )
        let orient = Mat3::from_quat(self.orientation);
        let inertial_tensor = orient * self.shape.inertial_tensor() * orient.transpose();
        let alpha = inertial_tensor.inverse()
            * self
                .angular_velocity
                .cross(inertial_tensor * self.angular_velocity);
        self.angular_velocity += alpha * dt_sec;

        // Update orientation.
        let d_angle = self.angular_velocity * dt_sec;
        let d_q = Quat::from_scaled_axis(d_angle);
        self.orientation = (d_q * self.orientation).normalize();

        // Now get the new model position
        self.position = position_cm + d_q.mul_vec3(cm_to_pos);
    }
}

pub struct Scene {
    pub bodies: Vec<Body>,
}

impl Scene {
    pub fn new_simple_scene() -> Scene {
        let mut bodies = Vec::<Body>::new();
        // dynamic bodies
        let radius = 2.0;
        let x = 1.;
        let z = 1.;
        let y = 10.;
        let color = Vec4::new(1.0, 0.3, 1.0, 1.0);
        bodies.push(Body {
            position: Vec3::new(z, y, x),
            orientation: Quat::IDENTITY,
            inv_mass: 1.0,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            elasticity: 0.5,
            shape: Box::new(Sphere { radius, color }),
        });

        // static "floor"
        let radius = 100.;
        let x = 0.;
        let z = 0.;
        let y = -radius;
        let color = Vec4::new(0.2, 1.0, 0.2, 1.0);
        bodies.push(Body {
            position: Vec3::new(z, y, x),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.,
            elasticity: 1.0,
            shape: Box::new(Sphere { radius, color }),
        });
        Scene { bodies }
    }

    pub fn drawables(&self) -> Vec<Drawable> {
        let mut objects = Vec::<Drawable>::new();
        for body in &self.bodies {
            let rot = Mat4::from_quat(body.orientation);
            let scale = Mat4::from_scale(Vec3::splat(body.shape.radius()));
            let trans = Mat4::from_translation(body.position);
            objects.push(Drawable {
                model: trans * rot * scale,
                color: body.shape.color(),
            });
        }
        objects
    }

    pub fn update(&mut self, dt_sec: f32) {
        // Gravity impulse.
        for body in &mut self.bodies {
            let mass = 1. / body.inv_mass;
            let impulse_gravity = Vec3::new(0., -10., 0.) * mass * dt_sec;
            body.apply_impulse_linear(impulse_gravity);
        }

        // Check for collisions with other bodies.

        for i in 0..self.bodies.len() {
            for j in (i + 1)..self.bodies.len() {
                let body_a = &self.bodies[i];
                let body_b = &self.bodies[j];

                if body_a.inv_mass == 0.0 && body_b.inv_mass == 0.0 {
                    continue;
                }

                if let Some(contact) = intersect(&mut self.bodies, i, j) {
                    resolve_contact(&mut self.bodies, &contact);
                }
            }
        }

        // position update
        for body in &mut self.bodies {
            body.update(dt_sec);
        }
    }
}

struct Contact {
    pt_on_a_world_space: Vec3,
    pt_on_b_world_space: Vec3,
    pt_on_a_local_space: Vec3,
    pt_on_b_local_space: Vec3,
    normal: Vec3,
    separation_distance: f32,
    time_of_impact: f32,
    body_a: usize,
    body_b: usize,
}

fn intersect(bodies: &mut [Body], i: usize, j: usize) -> Option<Contact> {
    let body_a = &bodies[i];
    let body_b = &bodies[j];

    let ab = (body_b.position - body_a.position);
    let normal = ab.normalize();

    let sphere_a = &body_a.shape;
    let sphere_b = &body_b.shape;

    let pt_on_a_world_space = body_a.position + normal * sphere_a.radius();
    let pt_on_b_world_space = body_b.position - normal * sphere_b.radius();

    let radius_ab = sphere_a.radius() + sphere_b.radius();
    let length_square = ab.length();

    if length_square <= (radius_ab) {
        return Some(Contact {
            pt_on_a_world_space,
            pt_on_b_world_space,
            pt_on_a_local_space: Default::default(),
            pt_on_b_local_space: Default::default(),
            normal,
            separation_distance: 0.0,
            time_of_impact: 0.0,
            body_a: i,
            body_b: j,
        });
    }
    None
}

fn resolve_contact(bodies: &mut [Body], contact: &Contact) {
    let vec_impulse_j: Vec3;
    let impulse_friction: Vec3;
    let pt_on_a = contact.pt_on_a_world_space;
    let pt_on_b = contact.pt_on_b_world_space;
    {
        let (body_a, body_b) = (&bodies[contact.body_a], &bodies[contact.body_b]);

        let elasticity = body_a.elasticity * body_b.elasticity;
        let inv_world_inertia_a = body_a.get_inverse_inertial_tensor_world_space();
        let inv_world_inertia_b = body_b.get_inverse_inertial_tensor_world_space();
        let n = contact.normal;

        let ra = pt_on_a - body_a.get_centre_of_mass_world_space();
        let rb = pt_on_b - body_b.get_centre_of_mass_world_space();
        let angular_ja = (inv_world_inertia_a * ra.cross(n)).cross(ra);
        let angular_jb = (inv_world_inertia_b * rb.cross(n)).cross(rb);
        let angular_factor = (angular_ja + angular_jb).dot(n);

        // Get the world space velocity of the motion and rotation.
        let vel_a = body_a.linear_velocity + body_a.angular_velocity.cross(ra);
        let vel_b = body_b.linear_velocity + body_b.angular_velocity.cross(rb);

        // Calculate the collision impulse.
        let vab = body_a.linear_velocity - body_b.linear_velocity;
        let impulse_j = -(1.0 + elasticity) * vab.dot(n) / (body_a.inv_mass + body_b.inv_mass);
        vec_impulse_j = n * impulse_j;
    }

    bodies[contact.body_a].apply_impulse_linear(vec_impulse_j * 1.0);
    bodies[contact.body_b].apply_impulse_linear(vec_impulse_j * -1.0);

    // Let's also move our colliding objects to just outside of each other
    let (body_a, body_b) = (&bodies[contact.body_a], &bodies[contact.body_b]);

    let t_a = body_a.inv_mass / (body_a.inv_mass + body_b.inv_mass);
    let t_b = body_b.inv_mass / (body_a.inv_mass + body_b.inv_mass);
    let ds = contact.pt_on_b_world_space - contact.pt_on_a_world_space;

    bodies[contact.body_a].position += ds * t_a;
    bodies[contact.body_b].position -= ds * t_b;
}
