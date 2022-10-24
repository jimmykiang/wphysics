use crate::engine::Body;
use glam::Vec3;

pub struct Contact {
    pub pt_on_a_world_space: Vec3,
    pub pt_on_b_world_space: Vec3,
    pub pt_on_a_local_space: Vec3,
    pub pt_on_b_local_space: Vec3,
    pub normal: Vec3,
    pub separation_distance: f32,
    pub time_of_impact: f32,
    pub body_a: usize,
    pub body_b: usize,
}

pub fn resolve_contact(bodies: &mut [Body], contact: &Contact) {
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
        let vab = vel_a - vel_b;
        let impulse_j =
            (1.0 + elasticity) * vab.dot(n) / (body_a.inv_mass + body_b.inv_mass + angular_factor);
        vec_impulse_j = n * impulse_j;

        // Calculate the impulse caused by friction.
        let friction = body_a.friction * body_b.friction;

        // Find the normal direction of the velocity with respect to the normal of the collision.
        let vel_norm = n * n.dot(vab);

        // Find the tangent direction of the velocity with respect to the normal of the collision.
        let vel_tang = vab - vel_norm;

        // Get the tangential velocities relative to the other body.
        let relative_vel_tang = vel_tang.normalize_or_zero();

        let inertia_a = (inv_world_inertia_a * ra.cross(relative_vel_tang)).cross(ra);
        let inertia_b = (inv_world_inertia_b * rb.cross(relative_vel_tang)).cross(rb);
        let inv_inertia = (inertia_a + inertia_b).dot(relative_vel_tang);

        // Calculate the tangential impulse for friction.
        let reduced_mass = 1. / (body_a.inv_mass + body_b.inv_mass + inv_inertia);
        impulse_friction = vel_tang * reduced_mass * friction;
    }

    bodies[contact.body_a].apply_impulse(pt_on_a, vec_impulse_j * -1.0);
    bodies[contact.body_b].apply_impulse(pt_on_b, vec_impulse_j * 1.0);

    // Apply kinetic friction.
    bodies[contact.body_a].apply_impulse(pt_on_a, impulse_friction * -1.);
    bodies[contact.body_b].apply_impulse(pt_on_b, impulse_friction * 1.);

    // Let's also move our colliding objects to just outside of each other
    let (body_a, body_b) = (&bodies[contact.body_a], &bodies[contact.body_b]);

    let t_a = body_a.inv_mass / (body_a.inv_mass + body_b.inv_mass);
    let t_b = body_b.inv_mass / (body_a.inv_mass + body_b.inv_mass);
    let ds = contact.pt_on_b_world_space - contact.pt_on_a_world_space;

    bodies[contact.body_a].position += ds * t_a;
    bodies[contact.body_b].position -= ds * t_b;
}
