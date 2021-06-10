// This module contains a few helpers that should not be considered as part of the public API,
// but are exposed for use by other lyon crates.
// Changing them doesn't necessarily imply semver breaking bumps.

use lyon_geom::Scalar;

pub use crate::geom::{CubicBezierSegment, QuadraticBezierSegment};
pub use crate::math::Point;
pub use crate::traits::PathBuilder;
pub use crate::EndpointId;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DebugValidator {
    #[cfg(debug_assertions)]
    in_subpath: bool,
}

impl DebugValidator {
    #[inline(always)]
    pub fn new() -> Self {
        DebugValidator {
            #[cfg(debug_assertions)]
            in_subpath: false,
        }
    }

    #[inline(always)]
    pub fn begin(&mut self) {
        #[cfg(debug_assertions)]
        {
            assert!(!self.in_subpath);
            self.in_subpath = true;
        }
    }

    #[inline(always)]
    pub fn end(&mut self) {
        #[cfg(debug_assertions)]
        {
            assert!(self.in_subpath);
            self.in_subpath = false;
        }
    }

    #[inline(always)]
    pub fn edge(&self) {
        #[cfg(debug_assertions)]
        {
            assert!(self.in_subpath);
        }
    }

    #[inline(always)]
    pub fn build(&self) {
        #[cfg(debug_assertions)]
        {
            assert!(!self.in_subpath);
        }
    }
}

pub fn flatten_quadratic_bezier<T: Scalar>(
    tolerance: T,
    from: Point<T>,
    ctrl: Point<T>,
    to: Point<T>,
    builder: &mut impl PathBuilder<T>,
) -> EndpointId {
    let curve = QuadraticBezierSegment { from, ctrl, to };
    let mut id = EndpointId::INVALID;
    curve.for_each_flattened(tolerance, &mut |point| {
        id = builder.line_to(point);
    });

    id
}

pub fn flatten_cubic_bezier<T: Scalar>(
    tolerance: T,
    from: Point<T>,
    ctrl1: Point<T>,
    ctrl2: Point<T>,
    to: Point<T>,
    builder: &mut impl PathBuilder<T>,
) -> EndpointId {
    let curve = CubicBezierSegment {
        from,
        ctrl1,
        ctrl2,
        to,
    };
    let mut id = EndpointId::INVALID;
    curve.for_each_flattened(tolerance, &mut |point| {
        id = builder.line_to(point);
    });

    id
}
