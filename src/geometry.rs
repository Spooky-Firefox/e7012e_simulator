#[derive(Clone, Copy, Debug)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Segment {
    pub a: Vec2,
    pub b: Vec2,
}

impl Vec2 {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn add(self, rhs: Vec2) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }

    pub fn sub(self, rhs: Vec2) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }

    pub fn mul(self, scalar: f32) -> Self {
        Self::new(self.x * scalar, self.y * scalar)
    }

    pub fn dot(self, rhs: Vec2) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    pub fn norm(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n <= 1e-8 {
            return Self::new(1.0, 0.0);
        }
        Self::new(self.x / n, self.y / n)
    }

    pub fn from_angle_rad(angle: f32) -> Self {
        Self::new(angle.cos(), angle.sin())
    }
}

fn cross(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

pub fn ray_segment_intersection_distance(origin: Vec2, dir: Vec2, seg: Segment) -> Option<f32> {
    let r = dir.normalized();
    let s = seg.b.sub(seg.a);
    let qp = seg.a.sub(origin);
    let denom = cross(r, s);

    if denom.abs() <= 1e-7 {
        return None;
    }

    let t = cross(qp, s) / denom;
    let u = cross(qp, r) / denom;

    if t >= 0.0 && (0.0..=1.0).contains(&u) {
        Some(t)
    } else {
        None
    }
}

pub fn point_segment_distance(point: Vec2, seg: Segment) -> f32 {
    let ab = seg.b.sub(seg.a);
    let ap = point.sub(seg.a);
    let denom = ab.dot(ab);
    if denom <= 1e-8 {
        return point.sub(seg.a).norm();
    }

    let t = (ap.dot(ab) / denom).clamp(0.0, 1.0);
    let projection = seg.a.add(ab.mul(t));
    point.sub(projection).norm()
}

pub fn wrap_angle_rad(angle: f32) -> f32 {
    let mut a = angle;
    while a > core::f32::consts::PI {
        a -= 2.0 * core::f32::consts::PI;
    }
    while a < -core::f32::consts::PI {
        a += 2.0 * core::f32::consts::PI;
    }
    a
}

pub fn wrap_angle_deg(angle: f32) -> f32 {
    let mut a = angle;
    while a > 180.0 {
        a -= 360.0;
    }
    while a < -180.0 {
        a += 360.0;
    }
    a
}
