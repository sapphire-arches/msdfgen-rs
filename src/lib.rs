//! msdfgen-rs is a library for generating "MSDFs"
//! It is heavily inspired by
//! [Viktor Chlumský's `msdfgen` utility](https://github.com/Chlumsky/msdfgen/).
//! We also provide a generator for regular SDFs.

#[warn(missing_docs)]
mod math;
pub mod path;
mod utils;

use self::math::median;
use self::path::ColorFlags;
use self::utils::{EdgePoint};
use lyon_path::math::{Angle, Point, Vector};
use lyon_path::Segment;

pub use self::path::{Contour, PathCollector};

/// Computes an MSDF from a list of contours. The returned vectors are a `dim` by `dim`
/// matrix of signed distance values.
pub fn compute_msdf(contours: &[Contour], dim: usize) -> Vec<Vec<(f32, f32, f32)>> {
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
    let scale: f32 = 1.0 / (dim as f32);
    let windings: Vec<i32> = contours.iter().map(|c| c.winding() as i32).collect();
    println!("Windings: {:?}", windings);

    (0..dim)
        .map(|y| {
            let py = (y as f32 + 0.5) * scale;
            (0..dim)
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

                            if elem.color.contains(ColorFlags::R) && d < contour_min_r.dist {
                                contour_min_r.dist = d;
                                contour_min_r.edge = Some(&elem);
                                contour_min_r.nearest_approach = na;
                            }
                            if elem.color.contains(ColorFlags::G) && d < contour_min_g.dist {
                                contour_min_g.dist = d;
                                contour_min_g.edge = Some(&elem);
                                contour_min_g.nearest_approach = na;
                            }
                            if elem.color.contains(ColorFlags::B) && d < contour_min_b.dist {
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

/// Computes a SDF from a list of contours. The returned vectors are a `dim` by `dim`
/// matrix of signed distances.
pub fn compute_sdf(contours: &[Contour], dim: usize) -> Vec<Vec<f32>> {
    let scale: f32 = 1.0 / (dim as f32);
    let windings: Vec<i32> = contours.iter().map(|c| c.winding() as i32).collect();

    (0..dim)
        .map(|y| {
            let py = (y as f32 + 0.5) * scale;
            (0..dim)
                .map(|x| {
                    if contours.len() == 0 {
                        return 1.0f32;
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

                    md
                })
                .collect()
        })
        .collect()
}

/// Recolor the contours prior to MSDF computation.
/// This function uses a simple technique,
/// again based on Viktor's implementation.
/// It is left as a separate step so you can implement more complex techniques as desired.
pub fn recolor_contours(contours: Vec<Contour>, threshold: Angle, mut seed: u64) -> Vec<Contour> {
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
                            split[0].color = colors[0];
                            split[1].color = colors[1];
                            split[2].color = colors[2];

                            c.elements.clear();
                            c.elements.extend_from_slice(&split);
                        }
                        2 => {
                            // 2 segments. We split it into 6, and assign colors by hand
                            let mut split0 = c.elements[0].split_in_thirds();
                            let mut split1 = c.elements[1].split_in_thirds();
                            split0[0].color = colors[0];
                            split0[1].color = colors[0];
                            split0[2].color = colors[1];
                            split1[0].color = colors[1];
                            split1[1].color = colors[2];
                            split1[2].color = colors[2];

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
                                e.color = colors[idx];
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
                        c.elements[i].color = color;
                    }
                    c
                }
            }
        })
        .collect()
}
