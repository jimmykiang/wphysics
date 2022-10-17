use crate::objects::Drawable;
use glam::{Mat3, Mat4, Quat, Vec3, Vec4};
use std::any::Any;

trait Shaped {
    fn shape_type(&self) -> Shape;
    fn color(&self) -> Vec4;
    fn radius(&self) -> f32;
    fn as_any(&self) -> &dyn Any;
}

#[derive(PartialEq)]
enum Shape {
    Sphere,
}

struct Sphere {
    radius: f32,
    color: Vec4,
}

impl Shaped for Sphere {
    fn shape_type(&self) -> Shape {
        Shape::Sphere
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
}

pub struct Body {
    pub position: Vec3,
    pub orientation: Quat,
    shape: Box<dyn Shaped>,
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
        let color = Vec4::new(1.0, 0.2, 0.2, 1.0);
        bodies.push(Body {
            position: Vec3::new(z, y, x),
            orientation: Quat::IDENTITY,
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

    pub fn update(&mut self, dt_sec: f32) {}
}
