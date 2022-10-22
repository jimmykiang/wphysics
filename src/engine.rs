use crate::objects::Drawable;
use glam::{Mat3, Mat4, Quat, Vec3, Vec4};
use std::any::Any;

trait Shape {
    fn shape_type(&self) -> ShapeEnum;
    fn color(&self) -> Vec4;
    fn radius(&self) -> f32;
    fn as_any(&self) -> &dyn Any;
    fn get_centre_of_mass(&self) -> Vec3;
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
}

pub struct Body {
    pub position: Vec3,
    pub orientation: Quat,
    pub inv_mass: f32,
    pub linear_velocity: Vec3,
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

    fn update(&mut self, dt_sec: f32) {
        self.position += self.linear_velocity * dt_sec;
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
        let x = 0.;
        let z = 0.;
        let y = 10.;
        let color = Vec4::new(1.0, 0.3, 1.0, 1.0);
        bodies.push(Body {
            position: Vec3::new(z, y, x),
            orientation: Quat::IDENTITY,
            inv_mass: 1.0,
            linear_velocity: Vec3::ZERO,
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
            inv_mass: 0.,
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

                if let Some(contact) = intersect(body_a, body_b) {
                    self.bodies[i].linear_velocity = Vec3::ZERO;
                    self.bodies[j].linear_velocity = Vec3::ZERO;
                }
            }
        }

        // position update
        for body in &mut self.bodies {
            body.update(dt_sec);
        }
    }
}

struct Contact {}

fn intersect(body_a: &Body, body_b: &Body) -> Option<Contact> {
    let ab = body_a.position - body_b.position;
    let radius_ab = body_a.radius() + body_b.radius();
    let length_square = ab.length();

    if length_square <= (radius_ab) {
        return Some(Contact {});
    }
    None
}
