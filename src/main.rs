#[macro_use]
extern crate glium;

use font_kit::font::Font;
use glium::{glutin, Surface};
use lyon_path::math::{Angle, Point, Vector};

fn get_font() -> Font {
    use font_kit::family_name::FamilyName;
    use font_kit::properties::{Properties, Style};
    use font_kit::source::SystemSource;
    let source = SystemSource::new();

    source
        .select_best_match(
            &[FamilyName::Monospace],
            Properties::new().style(Style::Normal),
        )
        .unwrap()
        .load()
        .unwrap()
}

#[derive(Copy, Clone, PartialEq, Debug)]
struct SignedDistance {
    /// The actual distance
    distance: f32,
    /// The cosine of the angle between the tangent vector of the path segment
    /// at the point of closest approach and the vector from the point of
    /// closest approach to the point to which distance was measured. This is used to
    dot: f32,
}

impl SignedDistance {
    fn new(distance: f32, dot: f32) -> Self {
        Self { distance, dot }
    }
}

impl std::cmp::PartialOrd for SignedDistance {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        use std::cmp::Ordering;

        match self.distance.abs().partial_cmp(&other.distance.abs()) {
            Some(Ordering::Less) => Some(Ordering::Less),
            Some(Ordering::Greater) => Some(Ordering::Greater),
            Some(Ordering::Equal) => self.dot.partial_cmp(&other.dot),
            None => None,
        }
    }
}

#[derive(Clone, Debug, Copy)]
enum PathElement {
    Linear {
        s: Point,
        e: Point,
        color: ColorFlags,
    },
    Quadratic {
        s: Point,
        e: Point,
        c: Point,
        color: ColorFlags,
    },
    Cubic {
        s: Point,
        e: Point,
        c1: Point,
        c2: Point,
        color: ColorFlags,
    },
}

fn median(a: f32, b: f32, c: f32) -> f32 {
    let min = |a: f32, b: f32| a.min(b);
    let max = |a: f32, b: f32| a.max(b);
    max(min(a, b), min(max(a, b), c))
}

fn solve_quadratic(ts: &mut [f32; 2], a: f32, b: f32, c: f32) -> usize {
    if a.abs() < 1E-14 {
        if b.abs() < 1E-14 {
            0
        } else {
            ts[0] = -c / b;
            1
        }
    } else {
        let dscr = (b * b) - (4.0 * a * c);
        if dscr > 0.0 {
            let dscr = dscr.sqrt();
            ts[0] = (-b + dscr) / (2.0 * a);
            ts[1] = (-b - dscr) / (2.0 * a);
            2
        } else if dscr > -1E-14 {
            ts[0] = -b / (2.0 * a);
            1
        } else {
            0
        }
    }
}

fn solve_cubic_normed(ts: &mut [f32; 3], a: f32, b: f32, c: f32) -> usize {
    let a2 = a * a;
    let q = (a2 - 3.0 * b) / 9.0;
    let r = (a * (2.0 * a2 - 9.0 * b) + 27.0 * c) / 54.0;
    let r2 = r * r;
    let q3 = q * q * q;
    if r2 < q3 {
        let t = r / q3.sqrt();
        let t = if t < -1.0 {
            -1.0
        } else if t > 1.0 {
            1.0
        } else {
            t
        };
        let t = t.acos();
        let a = a / 3.0;
        let q = -2.0 * q.sqrt();
        ts[0] = q * (t / 3.0).cos() - a;
        ts[1] = q * ((t + 2.0 * std::f32::consts::PI) / 3.0).cos() - a;
        ts[2] = q * ((t - 2.0 * std::f32::consts::PI) / 3.0).cos() - a;
        return 3;
    } else {
        let aa = -(r.abs() + (r2 - q3).sqrt()).powf(1.0 / 3.0);
        let aa = if r < 0.0 { -aa } else { aa };
        let bb = if aa.abs() < 1E-14 { 0.0 } else { q / aa };
        let a = a / 3.0;
        ts[0] = (aa + bb) - a;
        ts[1] = -0.5 * (aa + bb) - a;
        ts[2] = 0.5 * (3.0f32).sqrt() * (aa - bb);
        if ts[2].abs() < 1E-14 {
            return 2;
        } else {
            return 1;
        }
    }
}

fn solve_cubic(ts: &mut [f32; 3], a: f32, b: f32, c: f32, d: f32) -> usize {
    if a.abs() < 1E-14 {
        let mut tsr = [0.0, 0.0];
        let tr = solve_quadratic(&mut tsr, b, c, d);
        ts[0] = tsr[0];
        ts[1] = tsr[1];
        tr
    } else {
        solve_cubic_normed(ts, b / a, c / a, d / a)
    }
}

impl PathElement {
    fn sample(&self, f: f32) -> Point {
        let nf = 1.0f32 - f;
        match *self {
            PathElement::Linear { s, e, color: _ } => s * nf + (e * f).to_vector(),
            PathElement::Quadratic { s, e, c, color: _ } => {
                s * (nf * nf) + (c * (nf * 2.0f32 * f)).to_vector() + (e * (f * f)).to_vector()
            }
            PathElement::Cubic {
                s,
                e,
                c1,
                c2,
                color: _,
            } => {
                s * (nf * nf * nf)
                    + (c1 * (nf * 3.0f32 * nf)).to_vector()
                    + (c2 * (nf * 3.0f32 * f * f)).to_vector()
                    + (e * (f * f * f)).to_vector()
            }
        }
    }

    fn direction(&self, f: f32) -> Vector {
        match *self {
            PathElement::Linear { s, e, color: _ } => e - s,
            PathElement::Quadratic { s, e, c, color: _ } => Vector::lerp(&(c - s), e - c, f),
            PathElement::Cubic {
                s,
                e,
                c1,
                c2,
                color: _,
            } => {
                let d0 = s - c1;
                let d1 = c2 - c1;
                let d2 = e - c2;

                if f < 1E14 {
                    d0
                } else if f > 1.0 - 1E14 {
                    d2
                } else {
                    Vector::lerp(&Vector::lerp(&d0, d1, f), Vector::lerp(&d1, d2, f), f)
                }
            }
        }
    }

    /// Split a path element into 3rds
    fn split_in_thirds(&self) -> [PathElement; 3] {
        let a = self.sample(1.0 / 3.0);
        let b = self.sample(2.0 / 3.0);
        match *self {
            PathElement::Linear { s, e, color: col } => [
                PathElement::Linear {
                    s: s,
                    e: a,
                    color: col,
                },
                PathElement::Linear {
                    s: a,
                    e: b,
                    color: col,
                },
                PathElement::Linear {
                    s: b,
                    e: e,
                    color: col,
                },
            ],
            PathElement::Quadratic {
                s,
                e,
                c,
                color: col,
            } => {
                // I don't trust this
                let ca = s.lerp(c, 1.0 / 3.0);
                let cb = (s.lerp(c, 5.0 / 9.0)).lerp(c.lerp(e, 4.0 / 9.0), 0.5);
                let cc = c.lerp(e, 2.0 / 3.0);

                [
                    PathElement::Quadratic {
                        s: s,
                        e: a,
                        c: ca,
                        color: col,
                    },
                    PathElement::Quadratic {
                        s: a,
                        e: b,
                        c: cb,
                        color: col,
                    },
                    PathElement::Quadratic {
                        s: b,
                        e: e,
                        c: cc,
                        color: col,
                    },
                ]
            }
            PathElement::Cubic {
                s,
                e,
                c1,
                c2,
                color: col,
            } => {
                // I trust this even less

                // These variable names are read as <start><end><n / 3> lerp
                let sc11 = s.lerp(c1, 1.0 / 3.0);
                let sc12 = s.lerp(c1, 2.0 / 3.0);
                let c1c21 = c1.lerp(c2, 1.0 / 3.0);
                let c1c22 = c1.lerp(c2, 2.0 / 3.0);
                let c2e1 = c2.lerp(e, 1.0 / 3.0);
                let c2e2 = c2.lerp(e, 2.0 / 3.0);

                // These are <segment (a/b/c)<cpoint>
                let a1 = if (c1 - s).length() < 1E14 { s } else { sc11 };
                let a2 = sc11.lerp(c1c21, 1.0 / 3.0);

                let b1 = (sc11.lerp(c1c21, 1.0 / 3.0)).lerp(c1c21.lerp(c2e1, 1.0 / 3.0), 2.0 / 3.0);
                let b2 = (sc12.lerp(c1c22, 2.0 / 3.0)).lerp(c1c22.lerp(c2e2, 2.0 / 3.0), 1.0 / 3.0);

                let c1 = c1c22.lerp(c2e2, 2.0 / 3.0);
                let c2 = if (e - c2).length() < 1E14 { e } else { c2e2 };

                [
                    PathElement::Cubic {
                        s: s,
                        e: a,
                        c1: a1,
                        c2: a2,
                        color: col,
                    },
                    PathElement::Cubic {
                        s: a,
                        e: b,
                        c1: b1,
                        c2: b2,
                        color: col,
                    },
                    PathElement::Cubic {
                        s: b,
                        e: e,
                        c1: c1,
                        c2: c2,
                        color: col,
                    },
                ]
            }
        }
    }

    /// Computes the distance from p to this path element
    /// Returns the distance from the point to this path element,
    /// and the distance along this element to the closest point.
    fn distance(&self, p: Point) -> (SignedDistance, f32) {
        match *self {
            PathElement::Linear { s, e, .. } => {
                let aq = p - s;
                let ab = e - s;
                let f = aq.dot(ab) / ab.dot(ab);
                let eq = if f >= 0.5 { p - e } else { p - s };

                let dist_to_endpoint = eq.length();
                let endpoint_sd = SignedDistance::new(
                    aq.cross(ab).signum() * dist_to_endpoint,
                    // ab.normalize().cross(eq.normalize()),
                    ab.normalize().dot(eq.normalize()).abs(),
                );

                if 0.0 < f && f < 1.0 {
                    let ortho = Vector::new(ab.y, -ab.x).normalize();
                    let ortho_dist = ortho.dot(aq);
                    if ortho_dist.abs() < endpoint_sd.distance.abs() {
                        (SignedDistance::new(ortho_dist, 0.0), f)
                    } else {
                        (endpoint_sd, f)
                    }
                } else {
                    (endpoint_sd, f)
                }
            }

            PathElement::Quadratic {
                s: p0,
                e: p2,
                c: p1,
                ..
            } => {
                let qa = p0 - p;
                let ab = p1 - p0;
                let br = (p0 - p1) + (p2 - p1);
                let a = br.dot(br);
                let b = 3.0 * ab.dot(br);
                let c = 2.0 * ab.dot(ab) + qa.dot(br);
                let d = qa.dot(ab);
                let mut ts = [0.0f32, 0.0, 0.0];
                let solutions = solve_cubic(&mut ts, a, b, c, d);

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
                for i in 0..solutions {
                    if ts[i] <= 0.0 || ts[i] >= 1.0 {
                        continue;
                    }
                    let endpoint = p0 + (ab * 2.0 * ts[i]) + (br * ts[i] * ts[i]);
                    let delta = endpoint - p;
                    let dist = (p2 - p0).cross(delta).signum() * delta.length();

                    if dist.abs() < min_dist.abs() {
                        min_dist = dist;
                        f = ts[i];
                    }
                }

                if 0.0 <= f && f <= 1.0 {
                    (SignedDistance::new(min_dist, 0.0), f)
                // (SignedDistance::new(200f32, 0.0), f)
                } else if f < 0.5 {
                    (
                        SignedDistance::new(min_dist, ab.normalize().dot(qa.normalize()).abs()),
                        f,
                    )
                } else {
                    (
                        SignedDistance::new(
                            min_dist,
                            (p2 - p1).normalize().dot((p2 - p).normalize()).abs(),
                        ),
                        f,
                    )
                }
            }

            PathElement::Cubic { .. } => unimplemented!(),
        }
    }

    fn to_psuedodistance(&self, dist: SignedDistance, p: Vector, near: f32) -> SignedDistance {
        if near <= 0.0 {
            let dir = self.direction(0.0).normalize();
            let aq = p - self.sample(0.0).to_vector();
            let ts = aq.dot(dir);
            if ts < 0.0 {
                let ds = aq.cross(dir);
                if ds.abs() <= dist.distance.abs() {
                    return SignedDistance::new(ds, 0.0);
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
                    return SignedDistance::new(ds, 0.0);
                }
            }

            dist
        } else {
            dist
        }
    }

    fn set_color(&mut self, col: ColorFlags) {
        match *self {
            PathElement::Linear { s, e, .. } => *self = PathElement::Linear { color: col, s, e },
            PathElement::Quadratic { s, e, c, .. } => {
                *self = PathElement::Quadratic {
                    color: col,
                    s,
                    e,
                    c,
                }
            }
            PathElement::Cubic { s, e, c1, c2, .. } => {
                *self = PathElement::Cubic {
                    color: col,
                    s,
                    e,
                    c1,
                    c2,
                }
            }
        }
    }

    fn color(&self) -> ColorFlags {
        match *self {
            PathElement::Linear { color, .. } => color,
            PathElement::Quadratic { color, .. } => color,
            PathElement::Cubic { color, .. } => color,
        }
    }
}

bitflags::bitflags! {
    struct ColorFlags: u32 {
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
    fn switch(self, seed: &mut u64) -> Self {
        match self {
            ColorFlags::W | ColorFlags::K => {
                const START: [ColorFlags; 3] = [ColorFlags::C, ColorFlags::M, ColorFlags::Y];
                let tr = START[(*seed % 3) as usize];
                *seed /= 3;
                tr
            }
            _ => {
                let v = self.bits();
                let v = (v << (1 + (*seed & 1))) & 0b111;
                let v = match v.count_ones() {
                    0 => unreachable!(), /* There should never be 0 bits set */
                    1 => v | 0b001, /* We just shifted a bit off the left side, add one on the right */
                    2 => v,         /* We already have 2 bits, nothing to do */
                    _ => unreachable!(), /* There should never be 3+ bits set */
                };
                *seed >>= 1;

                Self::from_bits_truncate(v)
            }
        }
    }

    fn switch_banned(self, seed: &mut u64, banned: ColorFlags) -> Self {
        let combined = self & banned;
        match combined {
            ColorFlags::R | ColorFlags::G | ColorFlags::B => combined ^ ColorFlags::W,
            _ => self.switch(seed),
        }
    }

    fn float_color(self) -> [f32; 3] {
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

#[derive(Clone, Debug)]
struct Contour {
    elements: Vec<PathElement>,
}

impl Contour {
    fn winding(&self) -> f32 {
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

fn get_glyph(font: &Font, chr: char) -> Vec<Contour> {
    use lyon_path::builder::{FlatPathBuilder, PathBuilder};

    struct PathCollector {
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
        fn new() -> Self {
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
            self.elements.push(PathElement::Quadratic {
                s: self.pen,
                e: to,
                c: ctrl,
                color: ColorFlags::W,
            });

            self.pen = to;
        }

        fn cubic_bezier_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
            self.elements.push(PathElement::Cubic {
                s: self.pen,
                e: to,
                c1: ctrl1,
                c2: ctrl2,
                color: ColorFlags::W,
            });

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
            self.elements.push(PathElement::Linear {
                s: self.pen,
                e: to,
                color: ColorFlags::W,
            });
            self.pen = to;
        }

        fn close(&mut self) {
            if (self.pen - self.contour_start).length() > 1E-14 {
                self.elements.push(PathElement::Linear {
                    s: self.pen,
                    e: self.contour_start,
                    color: ColorFlags::W,
                });
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

    use font_kit::hinting::HintingOptions;
    let glyph_id = font.glyph_for_char(chr).unwrap();

    let mut builder = PathCollector::new();
    font.outline(glyph_id, HintingOptions::None, &mut builder)
        .unwrap();

    builder.build()
}

fn recolor_contours(contours: Vec<Contour>, threshold: Angle, mut seed: u64) -> Vec<Contour> {
    let (threshold, _) = threshold.sin_cos();

    // Determine if a point is a corner, assuming i and o are incoming and
    // outgoing normalized direction vectors
    let is_corner = |i: Vector, o: Vector| {
        let d = i.dot(o); /* |i| |o| cos(t) */
        let c = i.cross(o).abs(); /* |i| |o| sin(t) */

        // if this corner turns more than 90 degrees (detected by dot product/cos)
        // or if it turns more than the threshold angle (detected by cross product/sin)
        (d <= 0.0) || (c > threshold)
    };
    contours
        .into_iter()
        .map(|mut c| {
            let mut corners = Vec::new();
            let n = c.elements.len();
            // Find all the corners
            if n != 0 {
                let mut prev_dir = c.elements[n - 1].direction(1.0).normalize();
                for (i, e) in c.elements.iter().enumerate() {
                    let c_dir = e.direction(0.0).normalize();
                    if is_corner(prev_dir, c_dir) {
                        corners.push(i)
                    }
                    prev_dir = e.direction(1.0).normalize();
                }
            }

            match corners.len() {
                0 => {
                    // The whole contour is smooth, and we initialized all colors to white.
                    // No work to do
                    c
                }
                1 => {
                    // "Teardrop" case: there is only one sharp corner so we
                    // just pick 3 colors up front and cycle through them
                    println!("Handling teardrop with {:?} elements", n);
                    let mut colors = [
                        (ColorFlags::W).switch(&mut seed),
                        ColorFlags::W,
                        ColorFlags::W,
                    ];
                    colors[1] = colors[0].switch(&mut seed);
                    colors[2] = colors[1].switch(&mut seed);
                    let corner = corners[0];
                    match n {
                        0 => {
                            unreachable!();
                        }
                        1 => {
                            // Only a single edge segment, but it's a teardrop.
                            // We split it in 3 to make the colors happen
                            let mut split = c.elements[0].split_in_thirds();
                            split[0].set_color(colors[0]);
                            split[1].set_color(colors[1]);
                            split[2].set_color(colors[2]);

                            c.elements.clear();
                            c.elements.extend_from_slice(&split);
                        }
                        2 => {
                            // 2 segments. We split it into 6, and assign colors by hand
                            let mut split0 = c.elements[0].split_in_thirds();
                            let mut split1 = c.elements[1].split_in_thirds();
                            split0[0].set_color(colors[0]);
                            split0[1].set_color(colors[0]);
                            split0[2].set_color(colors[1]);
                            split1[0].set_color(colors[1]);
                            split1[1].set_color(colors[2]);
                            split1[2].set_color(colors[2]);

                            c.elements.clear();
                            c.elements.extend_from_slice(&split0);
                            c.elements.extend_from_slice(&split1);
                        }
                        _ => {
                            // We have more than 3 edges to rotate colors through
                            for (i, e) in c.elements.iter_mut().enumerate() {
                                // ported from this cursed C++ code:
                                // contour->edges[(corner+i)%m]->color = (colors+1)[int(3+2.875*i/(m-1)-1.4375+.5)-3];
                                let i = (n + i - corner) % n; // Emulate the ( corner + i) % m
                                let idx_fractional =
                                    3.5f32 + 2.875f32 * (i as f32) / ((n - 1) as f32) - 1.4375f32;
                                let idx = idx_fractional.floor() as usize - 2;
                                e.set_color(colors[idx]);
                            }
                        }
                    }
                    c
                }
                _ => {
                    // We have 2 or more corners
                    // Cycle through colors, switching whenever we hit another corner
                    let n_corners = corners.len();
                    let mut spline = 0;
                    let start = corners[0];
                    let mut color = ColorFlags::W;
                    color = color.switch(&mut seed);
                    let initial_color = color;
                    for i in 0..n {
                        let i = (start + i) % n;
                        if spline + 1 < n_corners && corners[spline + 1] == i {
                            spline = spline + 1;
                            color = color.switch_banned(&mut seed, initial_color);
                        }
                        c.elements[i].set_color(color);
                    }
                    c
                }
            }
        })
        .collect()
}

fn rescale_contours(contours: Vec<Contour>, bounds: lyon_path::math::Rect) -> Vec<Contour> {
    let reproject = |p: Point| {
        return Point::new(
            (p.x - bounds.origin.x) / bounds.size.width,
            (p.y - bounds.origin.y) / bounds.size.height,
        );
    };
    contours
        .into_iter()
        .map(|contour| {
            let elements: Vec<_> = contour
                .elements
                .into_iter()
                .map(|elem| match elem {
                    PathElement::Linear { s, e, color } => PathElement::Linear {
                        s: reproject(s),
                        e: reproject(e),
                        color,
                    },
                    PathElement::Quadratic { s, e, c, color } => PathElement::Quadratic {
                        s: reproject(s),
                        e: reproject(e),
                        c: reproject(c),
                        color,
                    },
                    PathElement::Cubic {
                        s,
                        e,
                        c1,
                        c2,
                        color,
                    } => PathElement::Cubic {
                        s: reproject(s),
                        e: reproject(e),
                        c1: reproject(c1),
                        c2: reproject(c2),
                        color,
                    },
                })
                .collect();

            Contour { elements }
        })
        .collect()
}

#[derive(Copy, Clone)]
struct Vertex2D {
    position: [f32; 2],
    color: [f32; 3],
}

glium::implement_vertex!(Vertex2D, position, color);

enum ContourColorMode {
    TraceContour,
    EdgeColors,
}

fn contours_vbo(
    contours: &[Contour],
    facade: &impl glium::backend::Facade,
    color_mode: ContourColorMode,
) -> glium::VertexBuffer<Vertex2D> {
    const VERTS_PER_SEG: usize = 4;
    let mut verts = Vec::new();
    verts.reserve(contours.iter().map(|c| c.elements.len()).sum::<usize>() * VERTS_PER_SEG);

    for contour in contours {
        let c_color: [f32; 3] = {
            use rand::Rng;
            let mut rng = rand::thread_rng();
            let r = rng.gen_range(0.0, 1.0);
            let g = rng.gen_range(0.0, 1.0);
            let b = rng.gen_range(0.0, 1.0);
            [r, g, b]
        };
        let n_elems = contour.elements.len();
        for (j, elem) in contour.elements.iter().enumerate() {
            let cf = (j as f32) / (n_elems as f32);
            for i in 0..VERTS_PER_SEG {
                let f = (i as f32) / (VERTS_PER_SEG as f32);
                let p = elem.sample(f);
                let cf = cf + (f / (n_elems as f32));
                verts.push(Vertex2D {
                    position: [p.x, p.y],
                    color: match color_mode {
                        ContourColorMode::TraceContour => {
                            [c_color[0] * cf, c_color[1] * cf, c_color[2] * cf]
                        }
                        ContourColorMode::EdgeColors => elem.color().float_color(),
                    },
                });
            }
        }
    }

    glium::VertexBuffer::immutable(facade, &verts).unwrap()
}

#[derive(Copy, Clone)]
struct EdgePoint<'a> {
    dist: SignedDistance,
    edge: Option<&'a PathElement>,
    nearest_approach: f32,
}

impl<'a> EdgePoint<'a> {
    fn new() -> Self {
        Self {
            dist: SignedDistance::new(1e24, 0.0),
            edge: None,
            nearest_approach: 0.0,
        }
    }

    fn to_pseudodistance(&mut self, p: Vector) {
        match self.edge {
            Some(edge) => self.dist = edge.to_psuedodistance(self.dist, p, self.nearest_approach),
            None => {}
        }
    }
}

fn compute_msdf(contours: &[Contour], range: f32) -> Vec<Vec<(f32, f32, f32)>> {
    #[derive(Copy, Clone, PartialEq)]
    struct MultiDistance {
        r: f32,
        g: f32,
        b: f32,
        med: f32,
    };
    impl MultiDistance {
        fn new(v: f32) -> Self {
            Self {
                r: v,
                g: v,
                b: v,
                med: v,
            }
        }
    }
    const DIM: usize = 64;
    let scale: f32 = 1.0 / (DIM as f32);
    let windings: Vec<i32> = contours.iter().map(|c| c.winding() as i32).collect();
    println!("Windings: {:?}", windings);

    (0..DIM)
        .map(|y| {
            let py = (y as f32 + 0.5) * scale;
            (0..DIM)
                .map(|x| {
                    // We assume there is at least 1 contour
                    // If there isn't make everything magenta
                    if contours.len() == 0 {
                        return (1.0f32, 0.0, 1.0);
                    }

                    let px = (x as f32 + 0.5) * scale;
                    let p = Vector::new(px, py);

                    let mut neg_dist = 1e24f32;
                    let mut pos_dist = -1e24f32;
                    let mut d = 1e24f32;
                    let mut winding = 0;
                    let mut contour_distances = Vec::new();
                    contour_distances.reserve(contours.len());

                    let mut sr = EdgePoint::new();
                    let mut sg = EdgePoint::new();
                    let mut sb = EdgePoint::new();

                    for (i, contour) in contours.iter().enumerate() {
                        let mut contour_min_r = EdgePoint::new();
                        let mut contour_min_g = EdgePoint::new();
                        let mut contour_min_b = EdgePoint::new();

                        for elem in &contour.elements {
                            let (d, na) = elem.distance(p.to_point());

                            if elem.color().contains(ColorFlags::R) && d < contour_min_r.dist {
                                contour_min_r.dist = d;
                                contour_min_r.edge = Some(&elem);
                                contour_min_r.nearest_approach = na;
                            }
                            if elem.color().contains(ColorFlags::G) && d < contour_min_g.dist {
                                contour_min_g.dist = d;
                                contour_min_g.edge = Some(&elem);
                                contour_min_g.nearest_approach = na;
                            }
                            if elem.color().contains(ColorFlags::B) && d < contour_min_b.dist {
                                contour_min_b.dist = d;
                                contour_min_b.edge = Some(&elem);
                                contour_min_b.nearest_approach = na;
                            }
                        }

                        if contour_min_r.dist < sr.dist {
                            sr = contour_min_r;
                        }
                        if contour_min_g.dist < sg.dist {
                            sg = contour_min_g;
                        }
                        if contour_min_b.dist < sb.dist {
                            sb = contour_min_b;
                        }

                        let med_min_dist = median(
                            contour_min_r.dist.distance,
                            contour_min_g.dist.distance,
                            contour_min_b.dist.distance,
                        )
                        .abs();
                        if med_min_dist < d {
                            d = med_min_dist;
                            winding = -windings[i];
                        }

                        contour_min_r.to_pseudodistance(p);
                        contour_min_g.to_pseudodistance(p);
                        contour_min_b.to_pseudodistance(p);

                        let med_min_dist = median(
                            contour_min_r.dist.distance,
                            contour_min_g.dist.distance,
                            contour_min_b.dist.distance,
                        );

                        let mut msd = MultiDistance::new(med_min_dist);
                        msd.r = contour_min_r.dist.distance;
                        msd.g = contour_min_g.dist.distance;
                        msd.b = contour_min_b.dist.distance;
                        msd.med = med_min_dist;
                        contour_distances.push(msd);
                        if windings[i] > 0
                            && med_min_dist >= 0.0
                            && med_min_dist.abs() < pos_dist.abs()
                        {
                            pos_dist = med_min_dist;
                        }
                        if windings[i] < 0
                            && med_min_dist <= 0.0
                            && med_min_dist.abs() < neg_dist.abs()
                        {
                            neg_dist = med_min_dist;
                        }
                    }

                    assert!(contour_distances.len() == windings.len());

                    sr.to_pseudodistance(p);
                    sg.to_pseudodistance(p);
                    sb.to_pseudodistance(p);

                    let mut mmsd = MultiDistance::new(-1e24);
                    if pos_dist >= 0.0 && pos_dist.abs() <= neg_dist.abs() {
                        mmsd.med = -1e24;
                        winding = 1;
                        for (csd, cw) in contour_distances.iter().zip(windings.iter()) {
                            if *cw > 0 && csd.med > mmsd.med && csd.med.abs() < neg_dist.abs() {
                                mmsd = *csd;
                            }
                        }
                    } else if neg_dist <= 0.0 && neg_dist.abs() <= pos_dist.abs() {
                        mmsd.med = 1e24;
                        winding = -1;
                        for (csd, cw) in contour_distances.iter().zip(windings.iter()) {
                            if *cw < 0 && csd.med < mmsd.med && csd.med.abs() < pos_dist.abs() {
                                mmsd = *csd;
                            }
                        }
                    }
                    for (csd, w) in contour_distances.iter().zip(windings.iter()) {
                        if *w != winding && csd.med.abs() < mmsd.med.abs() {
                            mmsd = *csd;
                        }
                    }

                    if median(sr.dist.distance, sg.dist.distance, sb.dist.distance) == mmsd.med {
                        mmsd.r = sr.dist.distance;
                        mmsd.g = sg.dist.distance;
                        mmsd.b = sb.dist.distance;
                    }

                    (mmsd.r, mmsd.g, mmsd.b)
                })
                .collect()
        })
        .collect()
}

fn compute_sdf(contours: &[Contour], range: f32) -> Vec<Vec<(f32, f32, f32)>> {
    const DIM: usize = 16;
    let scale: f32 = 1.0 / (DIM as f32);
    let windings: Vec<i32> = contours.iter().map(|c| c.winding() as i32).collect();

    (0..DIM)
        .map(|y| {
            let py = (y as f32 + 0.5) * scale;
            (0..DIM)
                .map(|x| {
                    if contours.len() == 0 {
                        return (1.0f32, 0.0, 1.0);
                    }

                    let px = (x as f32 + 0.5) * scale;
                    let p = Vector::new(px, py);

                    let mut neg_dist = 1e24f32;
                    let mut pos_dist = -1e24f32;
                    let mut winding = 0;
                    let mut contour_distances = Vec::new();
                    contour_distances.reserve(contours.len());

                    for (i, contour) in contours.iter().enumerate() {
                        let mut contour_min = EdgePoint::new();

                        for elem in contour.elements.iter() {
                            let (d, na) = elem.distance(p.to_point());

                            if d < contour_min.dist {
                                contour_min.dist = d;
                                contour_min.edge = Some(&elem);
                                contour_min.nearest_approach = na;
                            }
                        }

                        // contour_min.to_pseudodistance(p);
                        let cmdd = contour_min.dist.distance;

                        contour_distances.push(cmdd);

                        if windings[i] > 0 && cmdd >= 0.0 && cmdd.abs() < pos_dist.abs() {
                            pos_dist = cmdd;
                        }
                        if windings[i] < 0 && cmdd <= 0.0 && cmdd.abs() < neg_dist.abs() {
                            neg_dist = cmdd;
                        }
                    }

                    assert!(contour_distances.len() == windings.len());

                    let mut md = -1e24;
                    if pos_dist >= 0.0 && pos_dist.abs() <= neg_dist.abs() {
                        md = pos_dist;
                        winding = 1;
                        for (d, w) in contour_distances.iter().zip(windings.iter()) {
                            if *w > 0 && *d > md && d.abs() < neg_dist.abs() {
                                md = *d;
                            }
                        }
                    } else if neg_dist <= 0.0 && neg_dist.abs() <= pos_dist.abs() {
                        md = neg_dist;
                        winding = -1;
                        for (d, w) in contour_distances.iter().zip(windings.iter()) {
                            if *w < 0 && *d < md && d.abs() < pos_dist.abs() {
                                md = *d;
                            }
                        }
                    }
                    for (c, w) in contour_distances.iter().zip(windings.iter()) {
                        if *w != winding && c.abs() < md.abs() {
                            md = *c;
                        }
                    }

                    (md, md, md)
                })
                .collect()
        })
        .collect()
}

struct RenderData {
    raw_contours_vbo: glium::VertexBuffer<Vertex2D>,
    contours_vbo: glium::VertexBuffer<Vertex2D>,
    dist_tex: glium::texture::texture2d::Texture2d,
}

fn contour_vbos_for_chr(
    font: &Font,
    chr: char,
    display: &impl glium::backend::Facade,
) -> RenderData {
    let metrics = font.metrics();
    let contours = get_glyph(&font, chr);
    let contours = rescale_contours(
        contours,
        euclid::TypedRect::new(
            Point::new(0.0f32, metrics.descent),
            euclid::TypedSize2D::new(
                metrics.units_per_em as f32,
                metrics.ascent - metrics.descent,
            ),
        ),
    );
    let raw_contours_vbo = contours_vbo(&contours, display, ContourColorMode::TraceContour);
    println!("{:?}", contours);
    let contours = recolor_contours(contours, Angle::degrees(3.0), 0);
    println!("{:?}", font.metrics());
    let msdf = compute_msdf(&contours, 0.5);
    let msdf_min = msdf
        .iter()
        .flat_map(|x| x)
        .map(|x| x.0.min(x.1.min(x.2)))
        .fold(1e24, |acc, x| if x < acc { x } else { acc });
    let msdf_max = msdf
        .iter()
        .flat_map(|x| x)
        .map(|x| x.0.max(x.1.max(x.2)))
        .fold(-1e24, |acc, x| if x > acc { x } else { acc });
    let msdf_mag = 2.0 * msdf_max.abs().max(msdf_min.abs());
    let msdf: Vec<Vec<(f32, f32, f32)>> = msdf
        .into_iter()
        .map(|ys| {
            ys.into_iter()
                .map(|x| {
                    let remap = |v: f32| {
                        (v / msdf_mag) + 0.5
                        // if v < 0.0 {
                        //     (v - msdf_min) / (2.0 * -msdf_min)
                        // } else {
                        //     0.5 + v / (2.0 * msdf_max)
                        // }
                    };
                    (
                        remap(x.0), // (x.0 - msdf_min) / (msdf_max - msdf_min),
                        remap(x.1), // (x.1 - msdf_min) / (msdf_max - msdf_min),
                        remap(x.2), // (x.2 - msdf_min) / (msdf_max - msdf_min),
                    )
                })
                .collect()
        })
        .collect();
    let msdf = glium::texture::texture2d::Texture2d::new(display, msdf).unwrap();

    let contours_vbo = contours_vbo(&contours, display, ContourColorMode::EdgeColors);

    RenderData {
        dist_tex: msdf,
        raw_contours_vbo,
        contours_vbo,
    }
}

fn main() {
    let mut events_loop = glutin::EventsLoop::new();
    let window = glutin::WindowBuilder::new();
    let context = glutin::ContextBuilder::new();
    let context = context.with_gl_profile(glutin::GlProfile::Core);
    let context = context.with_gl_debug_flag(true);
    let display =
        glium::Display::new(window, context, &events_loop).expect("Error creating GL display");

    let font = get_font();

    let line_shader = program!(&display,
        410 => {
            vertex: r#"
#version 410
in vec2 position;
in vec3 color;
out vec3 cross_color;

void main() {
    gl_Position = vec4((position - vec2(0.5, 0.5)) * 2.0, 0.0, 1.0);
    cross_color = color;
}
"#,
            fragment: r#"
#version 410

in vec3 cross_color;
out vec4 color;

void main() {
    color = vec4(cross_color, 1.0);
}

"#,
        },
    )
    .unwrap();

    let bg_shader = program!(&display,
        410 => {
            vertex: r#"
#version 410
in vec2 position;
in vec3 color;
out vec3 cross_color;
out vec2 cross_uv;

void main() {
    gl_Position = vec4((position - vec2(0.5, 0.5)) * 2.0, 0.0, 1.0);
    cross_color = color;
    cross_uv = position;
}"#,
            fragment: r#"
#version 410

uniform sampler2D tex;

in vec2 cross_uv;
in vec3 cross_color;
out vec4 color;

void main() {
    // color = vec4(texture(tex, cross_uv).rgb, 1.0);
    // float v = color.b;
    // if (v > 0.5) {
    //     color = vec4(vec2(v), 0.0, 1.0);
    // } else {
    //     color = vec4(0.0, vec2(v), 1.0);
    // }
    vec3 x = texture(tex, cross_uv).rgb;
    float v = max(min(x.r, x.g), min(max(x.r, x.g), x.b));
    #define RADIUS 0.01
    // float c = smoothstep(0.5 - RADIUS, 0.5 + RADIUS, texture(tex, cross_uv).r);
    float c = smoothstep(0.5 - RADIUS, 0.5 + RADIUS, v);
    color = vec4(c, c, c, 1.0);
}"#,
        },
    )
    .unwrap();

    let tex_vbo = glium::VertexBuffer::immutable(
        &display,
        &[
            Vertex2D {
                position: [0.0, 0.0],
                color: [1.0, 1.0, 1.0],
            },
            Vertex2D {
                position: [1.0, 0.0],
                color: [1.0, 1.0, 1.0],
            },
            Vertex2D {
                position: [0.0, 1.0],
                color: [1.0, 1.0, 1.0],
            },
            Vertex2D {
                position: [1.0, 1.0],
                color: [1.0, 1.0, 1.0],
            },
        ],
    )
    .unwrap();

    const default_char: char = '2';
    let mut render_data = contour_vbos_for_chr(&font, default_char, &display);

    let mut o_down = false;
    let mut draw_outlines = false;

    let mut closed = false;
    while !closed {
        let params = glium::DrawParameters {
            ..Default::default()
        };

        let mut target = display.draw();
        target.clear_color(0.5, 0.5, 0.5, 1.0);

        let uniforms = uniform!(
            tex: render_data
                .dist_tex.sampled()
                .wrap_function(glium::uniforms::SamplerWrapFunction::Clamp),
        );

        target
            .draw(
                &tex_vbo,
                glium::index::NoIndices(glium::index::PrimitiveType::TriangleStrip),
                &bg_shader,
                &uniforms,
                &params,
            )
            .unwrap();

        // let uniforms = uniform!();
        // target
        //     .draw(
        //         &render_data.raw_contours_vbo,
        //         glium::index::NoIndices(glium::index::PrimitiveType::LineStrip),
        //         &line_shader,
        //         &uniforms,
        //         &params,
        //     )
        //     .unwrap();

        if draw_outlines {
            target
                .draw(
                    &render_data.contours_vbo,
                    glium::index::NoIndices(glium::index::PrimitiveType::LineStrip),
                    &line_shader,
                    &uniforms,
                    &params,
                )
                .unwrap();
        }

        target.finish().unwrap();

        events_loop.poll_events(|ev| match ev {
            glutin::Event::WindowEvent { event, .. } => match event {
                glutin::WindowEvent::CloseRequested => closed = true,
                glutin::WindowEvent::KeyboardInput { input, .. } => {
                    let key = match input.virtual_keycode {
                        Some(glutin::VirtualKeyCode::Key0) => '0',
                        Some(glutin::VirtualKeyCode::Key1) => '1',
                        Some(glutin::VirtualKeyCode::Key2) => '2',
                        Some(glutin::VirtualKeyCode::Key3) => '3',
                        Some(glutin::VirtualKeyCode::Key4) => '4',
                        Some(glutin::VirtualKeyCode::Key5) => '5',
                        Some(glutin::VirtualKeyCode::Key6) => '6',
                        Some(glutin::VirtualKeyCode::Key7) => '7',
                        Some(glutin::VirtualKeyCode::Key8) => '8',
                        Some(glutin::VirtualKeyCode::Key9) => '9',
                        Some(glutin::VirtualKeyCode::A) => 'a',
                        Some(glutin::VirtualKeyCode::B) => 'b',
                        Some(glutin::VirtualKeyCode::C) => 'c',
                        Some(glutin::VirtualKeyCode::D) => 'd',
                        Some(glutin::VirtualKeyCode::W) => 'w',
                        Some(glutin::VirtualKeyCode::X) => 'x',
                        Some(glutin::VirtualKeyCode::Y) => 'y',
                        Some(glutin::VirtualKeyCode::Z) => 'z',
                        Some(glutin::VirtualKeyCode::O) => {
                            if !o_down {
                                draw_outlines = !draw_outlines;
                            }
                            o_down = !o_down;
                            default_char
                        }
                        _ => default_char,
                    };
                    render_data = contour_vbos_for_chr(&font, key, &display);
                }
                _ => {}
            },
            _ => {}
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_line_dist_x() {
        let seg = PathElement::Linear {
            s: Point::new(0.0, 0.0),
            e: Point::new(2.0, 0.0),
            color: ColorFlags::W,
        };
        let (dst, f) = seg.distance(Point::new(1.0, 1.0));
        assert!((dst.distance + 1.0).abs() < 0.0001);
        assert!((f - 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(1.0, -1.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f - 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(-1.0, 0.0));
        assert!((dst.distance + 1.0).abs() < 0.0001);
        assert!((f + 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(3.0, 0.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f - 1.5).abs() < 0.0001);
    }

    #[test]
    fn test_line_dist_y() {
        let seg = PathElement::Linear {
            s: Point::new(0.0, 0.0),
            e: Point::new(0.0, 2.0),
            color: ColorFlags::W,
        };
        let (dst, f) = seg.distance(Point::new(1.0, 1.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f - 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(-1.0, 1.0));
        assert!((dst.distance + 1.0).abs() < 0.0001);
        assert!((f - 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(0.0, -1.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f + 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(0.0, 3.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f - 1.5).abs() < 0.0001);
    }

    #[test]
    fn test_quadratic() {
        let mut solns = [0.0, 0.0f32];
        assert!(solve_quadratic(&mut solns, 1.0, -2.0, 1.0) == 2);
        assert!((solns[0] - 1.0).abs() < 0.001);
        assert!((solns[1] - 1.0).abs() < 0.001);

        assert!(solve_quadratic(&mut solns, 0.0, 1.0, 1.0) == 1);
        assert!((solns[0] - 1.0).abs() < 0.001);

        assert!(solve_quadratic(&mut solns, 1.0, 0.0, 1.0) == 1);
        assert!((solns[0] - 0.0).abs() < 0.001);
    }
}
