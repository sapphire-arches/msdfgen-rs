//! Utilities for the SDF computations

use lyon_path::math::{Angle, Point, Vector};
use lyon_path::Segment;
use crate::path::PathElement;

/// Represents a distance to an edge segment
/// TODO: rename this to EdgeDistance
#[derive(Copy, Clone)]
pub(crate) struct EdgePoint<'a> {
    pub(crate) dist: SignedDistance,
    pub(crate) edge: Option<&'a PathElement>,
    pub(crate) nearest_approach: f32,
}

impl<'a> EdgePoint<'a> {
    /// Create a new edge point, initialized to infinite distance
    pub(crate) fn new() -> Self {
        Self {
            dist: SignedDistance::new(-1e24, 0.0),
            edge: None,
            nearest_approach: 0.0,
        }
    }

    pub(crate) fn to_pseudodistance(&mut self, p: Vector) {
        match self.edge {
            Some(edge) => self.dist = edge.to_psuedodistance(self.dist, p, self.nearest_approach),
            None => {}
        }
    }
}

/// A signed distance, augmented with the cosine of the angle
/// between the tangent of the edge and the vector from the
/// point of nearest approach to the measured point.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct SignedDistance {
    /// The actual distance
    pub(crate) distance: f32,
    /// The cosine of the angle between the tangent vector of the path segment
    /// at the point of closest approach and the vector from the point of
    /// closest approach to the point to which distance was measured. This is used to
    dot: f32,
}

impl SignedDistance {
    pub(crate) fn new(distance: f32, dot: f32) -> Self {
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
