//! Things related to our augmented path structure

use crate::utils::AugmentedDistance;
use lyon_path::builder::{FlatPathBuilder, PathBuilder};
use lyon_path::math::{Angle, Point, Vector};
use lyon_path::Segment;

bitflags::bitflags! {
    #[doc("Represents the color channels affected by the edge")]
    pub struct ColorFlags: u32 {
        const K = 0b000;
        const R = 0b001;
        const G = 0b010;
        const B = 0b100;
        const Y = 0b011;
        const M = 0b101;
        const C = 0b110;
        const W = 0b111;
    }
}

impl ColorFlags {
    pub(crate) fn switch(self, seed: &mut u64) -> Self {
        match self {
            ColorFlags::W | ColorFlags::K => {
                const START: [ColorFlags; 3] = [ColorFlags::C, ColorFlags::M, ColorFlags::Y];
                let tr = START[(*seed % 3) as usize];
                *seed /= 3;
                tr
            }
            ColorFlags::R | ColorFlags::G | ColorFlags::B => self ^ ColorFlags::W,
            _ => {
                let v = self.bits();
                let v = (v << (1 + (*seed & 1))) & 0b111;
                let v = match v.count_ones() {
                    0 => 0b11,           /* Somehow we lost all the bits. Default to yellow */
                    1 => v | 0b001, /* We just shifted a bit off the left side, add one on the right */
                    2 => v,         /* We already have 2 bits, nothing to do */
                    _ => unreachable!(), /* There should never be 3+ bits set */
                };
                *seed >>= 1;

                Self::from_bits_truncate(v)
            }
        }
    }

    pub(crate) fn switch_banned(self, seed: &mut u64, banned: ColorFlags) -> Self {
        let combined = self & banned;
        match combined {
            ColorFlags::R | ColorFlags::G | ColorFlags::B => combined ^ ColorFlags::W,
            _ => self.switch(seed),
        }
    }

    pub fn float_color(self) -> [f32; 3] {
        match self {
            ColorFlags::K => [0.0f32, 0.0f32, 0.0f32],
            ColorFlags::R => [1.0f32, 0.0f32, 0.0f32],
            ColorFlags::G => [0.0f32, 1.0f32, 0.0f32],
            ColorFlags::B => [0.0f32, 0.0f32, 1.0f32],
            ColorFlags::C => [0.0f32, 1.0f32, 1.0f32],
            ColorFlags::M => [1.0f32, 0.0f32, 1.0f32],
            ColorFlags::Y => [1.0f32, 1.0f32, 0.0f32],
            ColorFlags::W => [1.0f32, 1.0f32, 1.0f32],
            _ => [0.5, 0.7, 0.5],
        }
    }
}

/// A list of path elements forming a closed loop
#[derive(Clone, Debug)]
pub struct Contour {
    pub elements: Vec<PathElement>,
}

impl Contour {
    pub fn winding(&self) -> f32 {
        let shoelace = |a: Point, b: Point| (b.x - a.x) * (a.y + b.y);
        let n = self.elements.len();
        match n {
            0 => 0.0,
            1 => {
                let a = self.elements[0].sample(0.0);
                let b = self.elements[0].sample(1.0 / 3.0);
                let c = self.elements[0].sample(2.0 / 3.0);

                shoelace(a, b) + shoelace(b, c) + shoelace(c, a)
            }
            2 => {
                let a = self.elements[0].sample(0.0);
                let b = self.elements[0].sample(0.5);
                let c = self.elements[1].sample(0.0);
                let d = self.elements[1].sample(0.5);

                shoelace(a, b) + shoelace(b, c) + shoelace(c, d) + shoelace(d, a)
            }
            _ => {
                let mut total = 0.0;
                let mut prev = self.elements[n - 1].sample(0.0);

                for e in &self.elements {
                    let curr = e.sample(0.0);
                    total += shoelace(prev, curr);
                    prev = curr;
                }

                total
            }
        }
        .signum()
    }
}

#[derive(Clone, Debug, Copy)]
pub struct PathElement {
    pub segment: Segment,
    pub color: ColorFlags,
}

impl PathElement {
    fn new(segment: Segment, color: ColorFlags) -> PathElement {
        Self { segment, color }
    }

    pub fn sample(&self, t: f32) -> Point {
        match self.segment {
            Segment::Line(s) => s.sample(t),
            Segment::Quadratic(s) => s.sample(t),
            Segment::Cubic(s) => s.sample(t),
            Segment::Arc(s) => s.sample(t),
        }
    }

    pub fn direction(&self, f: f32) -> Vector {
        use lyon_geom::Segment as SegmentTrait;
        let f = f.min(1.0).max(0.0);
        match self.segment {
            Segment::Line(s) => s.derivative(f),
            Segment::Quadratic(s) => s.derivative(f),
            Segment::Cubic(s) => s.derivative(f),
            Segment::Arc(s) => s.derivative(f),
        }
    }

    /// Split a path element into 3rds
    pub fn split_in_thirds(&self) -> [PathElement; 3] {
        macro_rules! segment_case {
            ($i:expr, $s:expr) => {{
                let (a, b) = ($i).split(1.0 / 3.0);
                let (b, c) = b.split(1.0 / 2.0);

                [a, b, c]
                    .into_iter()
                    .map(|x| PathElement::new(($s)(*x), self.color))
                    .collect()
            }};
        }
        let segments: arrayvec::ArrayVec<[PathElement; 3]> = match self.segment {
            Segment::Line(s) => segment_case!(s, Segment::Line),
            Segment::Quadratic(s) => segment_case!(s, Segment::Quadratic),
            Segment::Cubic(s) => segment_case!(s, Segment::Cubic),
            Segment::Arc(s) => segment_case!(s, Segment::Arc),
        };

        segments
            .into_inner()
            .expect("We should have precisely the right capacity")
    }

    /// Computes the distance from p to this path element
    /// Returns the distance from the point to this path element,
    /// and the distance along this element to the closest point.
    pub fn distance(&self, p: Point) -> (AugmentedDistance, f32) {
        use lyon_geom::{LineSegment, QuadraticBezierSegment};
        match self.segment {
            Segment::Line(LineSegment { from: s, to: e }) => {
                let aq = p - s;
                let ab = e - s;
                let f = aq.dot(ab) / ab.dot(ab);
                let eq = if f >= 0.5 { p - e } else { p - s };

                let dist_to_endpoint = eq.length();
                let endpoint_sd = AugmentedDistance::new(
                    aq.cross(ab).signum() * dist_to_endpoint,
                    // ab.normalize().cross(eq.normalize()),
                    ab.normalize().dot(eq.normalize()).abs(),
                );

                if 0.0 < f && f < 1.0 {
                    let ortho = Vector::new(ab.y, -ab.x).normalize();
                    let ortho_dist = ortho.dot(aq);
                    if ortho_dist.abs() < endpoint_sd.distance.abs() {
                        (AugmentedDistance::new(ortho_dist, 0.0), f)
                    } else {
                        (endpoint_sd, f)
                    }
                } else {
                    (endpoint_sd, f)
                }
            }

            Segment::Quadratic(QuadraticBezierSegment {
                from: p0,
                ctrl: p1,
                to: p2,
            }) => {
                use lyon_geom::utils::cubic_polynomial_roots;
                let qa = p0 - p;
                let ab = p1 - p0;
                let br = (p0 - p1) + (p2 - p1);
                let a = br.dot(br);
                let b = 3.0 * ab.dot(br);
                let c = 2.0 * ab.dot(ab) + qa.dot(br);
                let d = qa.dot(ab);
                let solutions = cubic_polynomial_roots(a, b, c, d);

                let mut min_dist = ab.cross(qa).signum() * qa.length();

                let mut f = -qa.dot(ab) / ab.dot(ab);
                {
                    let ec = p2 - p1;
                    let ep = p2 - p;
                    let dist = ec.cross(ep).signum() * ep.length();
                    if dist.abs() < min_dist.abs() {
                        min_dist = dist;
                        f = (p - p1).dot(ec) / ec.dot(ec);
                    }
                }
                for t in solutions {
                    if t <= 0.0 || 1.0 <= t {
                        continue;
                    }
                    let endpoint = p0 + (ab * 2.0 * t) + (br * t * t);
                    let delta = endpoint - p;
                    let dist = (p2 - p0).cross(delta).signum() * delta.length();

                    if dist.abs() < min_dist.abs() {
                        min_dist = dist;
                        f = t;
                    }
                }

                if 0.0 <= f && f <= 1.0 {
                    (AugmentedDistance::new(min_dist, 0.0), f)
                // (AugmentedDistance::new(200f32, 0.0), f)
                } else if f < 0.5 {
                    (
                        AugmentedDistance::new(min_dist, ab.normalize().dot(qa.normalize()).abs()),
                        f,
                    )
                } else {
                    (
                        AugmentedDistance::new(
                            min_dist,
                            (p2 - p1).normalize().dot((p2 - p).normalize()).abs(),
                        ),
                        f,
                    )
                }
            }

            _ => unimplemented!(),
        }
    }

    pub(crate) fn to_psuedodistance(
        &self,
        dist: AugmentedDistance,
        p: Vector,
        near: f32,
    ) -> AugmentedDistance {
        if near <= 0.0 {
            let dir = self.direction(0.0).normalize();
            let aq = p - self.sample(0.0).to_vector();
            let ts = aq.dot(dir);
            if ts < 0.0 {
                let ds = aq.cross(dir);
                if ds.abs() <= dist.distance.abs() {
                    return AugmentedDistance::new(ds, 0.0);
                }
            }

            dist
        } else if near >= 1.0 {
            let dir = self.direction(1.0).normalize();
            let aq = p - self.sample(1.0).to_vector();
            let ts = aq.dot(dir);
            if ts > 0.0 {
                let ds = aq.cross(dir);
                if ds.abs() <= dist.distance.abs() {
                    return AugmentedDistance::new(ds, 0.0);
                }
            }

            dist
        } else {
            dist
        }
    }
}

/// This is a path collector which produces our custom contour type.
pub struct PathCollector {
    /// The start point of the last contour
    contour_start: Point,
    /// The current pen location
    pen: Point,
    /// in-flight path elements
    elements: Vec<PathElement>,
    /// Completed contours
    contours: Vec<Contour>,
}

impl PathCollector {
    pub fn new() -> Self {
        Self {
            contour_start: Point::new(0.0, 0.0),
            pen: Point::new(0.0, 0.0),
            elements: Vec::new(),
            contours: Vec::new(),
        }
    }
}

impl PathBuilder for PathCollector {
    fn quadratic_bezier_to(&mut self, ctrl: Point, to: Point) {
        self.elements.push(PathElement::new(
            Segment::Quadratic(lyon_geom::QuadraticBezierSegment {
                from: self.pen,
                to,
                ctrl,
            }),
            ColorFlags::W,
        ));
        self.pen = to;
    }

    fn cubic_bezier_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
        self.elements.push(PathElement::new(
            Segment::Cubic(lyon_geom::CubicBezierSegment {
                from: self.pen,
                to,
                ctrl1,
                ctrl2,
            }),
            ColorFlags::W,
        ));

        self.pen = to;
    }

    fn arc(&mut self, _center: Point, _radii: Vector, _sweep_angle: Angle, _x_rotation: Angle) {
        unimplemented!()
    }
}

impl FlatPathBuilder for PathCollector {
    type PathType = Vec<Contour>;

    fn move_to(&mut self, to: Point) {
        self.pen = to;
        self.contour_start = to;
    }

    fn line_to(&mut self, to: Point) {
        self.elements.push(PathElement::new(
            Segment::Line(lyon_geom::LineSegment { from: self.pen, to }),
            ColorFlags::W,
        ));
        self.pen = to;
    }

    fn close(&mut self) {
        if (self.pen - self.contour_start).length() > 1E-14 {
            self.elements.push(PathElement::new(
                Segment::Line(lyon_geom::LineSegment {
                    from: self.pen,
                    to: self.contour_start,
                }),
                ColorFlags::W,
            ));
        }

        self.pen = self.contour_start;
        let elements = std::mem::replace(&mut self.elements, Vec::new());

        self.contours.push(Contour { elements });
    }

    fn build(self) -> Self::PathType {
        let mut contours = self.contours;
        if self.elements.len() > 0 {
            let final_contour = Contour {
                elements: self.elements,
            };

            contours.push(final_contour);
        }

        contours
    }

    fn build_and_reset(&mut self) -> Self::PathType {
        let elements = std::mem::replace(&mut self.elements, Vec::new());
        if elements.len() > 0 {
            let final_contour = Contour { elements };

            self.contours.push(final_contour);
        }

        let tr = std::mem::replace(&mut self.contours, Vec::new());

        self.contour_start = Point::new(0.0, 0.0);
        self.pen = Point::new(0.0, 0.0);

        tr
    }

    fn current_position(&self) -> Point {
        self.pen
    }
}
