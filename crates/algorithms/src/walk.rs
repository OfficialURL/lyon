//! Move at a defined speed along a path.
//!
//! # Path walking
//!
//! ## Overview
//!
//! In principle, walking a path is similar to iterating over it,
//! but instead of going from receiving path segments (of varying
//! sizes), the path walker makes it possible to advance by a certain
//! distance along the path.
//!
//! ## Example
//!
//! ```
//! use lyon_algorithms::walk::{RegularPattern, walk_along_path};
//! use lyon_algorithms::path::PathSlice;
//! use lyon_algorithms::path::iterator::*;
//! use lyon_algorithms::path::math::Point;
//!
//! fn dots_along_path(path: PathSlice, dots: &mut Vec<Point>) {
//!     let mut pattern = RegularPattern {
//!         callback: &mut |position, _tangent, _distance| {
//!             dots.push(position);
//!             true // Return true to continue walking the path.
//!         },
//!         // Invoke the callback above at a regular interval of 3 units.
//!         interval: 3.0,
//!     };
//!
//!     let tolerance = 0.01; // The path flattening tolerance.
//!     let start_offset = 0.0; // Start walking at the beginning of the path.
//!     walk_along_path(
//!         path.iter().flattened(tolerance),
//!         start_offset,
//!         &mut pattern
//!     );
//! }
//!
//! ```
//!

use lyon_path::geom::Scalar;

use crate::geom::{CubicBezierSegment, QuadraticBezierSegment};
use crate::math::*;
use crate::path::builder::*;
use crate::path::{EndpointId, PathEvent};

/// Walks along the path staring at offset `start` and applies a `Pattern`.
pub fn walk_along_path<T: Scalar, Iter>(path: Iter, start: T, pattern: &mut dyn Pattern<T>)
where
    Iter: Iterator<Item = PathEvent<T>>,
{
    let mut walker = PathWalker::new(start, pattern);
    for evt in path {
        walker.path_event(evt);
        if walker.done {
            return;
        }
    }
}

/// Types implementing the `Pattern` can be used to walk along a path
/// at constant speed.
///
/// At each step, the pattern receives the position, tangent and already
/// traversed distance along the path and returns the distance until the
/// next step.
///
/// See the `RegularPattern` and `RepeatedPattern` implementations.
/// This trait is also implemented for all functions/closures with signature
/// `FnMut(Point<T>, Vector<T>, T) -> Option<T>`.
pub trait Pattern<T: Scalar> {
    /// This method is invoked at each step along the path.
    ///
    /// If this method returns None, path walking stops. Otherwise the returned
    /// value is the distance along the path to the next element in the pattern.
    fn next(&mut self, position: Point<T>, tangent: Vector<T>, distance: T) -> Option<T>;

    /// Invoked at the start each sub-path.
    ///
    /// Takes the leftover requested distance from the previous sub-path path,
    /// if any.
    ///
    /// If this method returns None, path walking stops. Otherwise the returned
    /// value is the distance along the path to the next element in the pattern.
    fn begin(&mut self, distance: T) -> Option<T> {
        Some(distance)
    }
}

/// A helper struct to walk along a flattened path using a builder API.
pub struct PathWalker<'l, T: Scalar> {
    prev: Point<T>,
    advancement: T,
    leftover: T,
    next_distance: T,
    first: Point<T>,
    need_moveto: bool,
    done: bool,

    pattern: &'l mut dyn Pattern<T>,
}

impl<'l, T: Scalar> PathWalker<'l, T> {
    pub fn new(start: T, pattern: &'l mut dyn Pattern<T>) -> Self {
        let start = T::max(start, T::ZERO);
        Self {
            prev: point(T::ZERO, T::ZERO),
            first: point(T::ZERO, T::ZERO),
            advancement: T::ZERO,
            leftover: T::ZERO,
            next_distance: start,
            need_moveto: true,
            done: false,
            pattern,
        }
    }
}

impl<'l, T: Scalar> PathBuilder<T> for PathWalker<'l, T> {
    fn begin(&mut self, to: Point<T>) -> EndpointId {
        self.need_moveto = false;
        self.first = to;
        self.prev = to;

        if let Some(distance) = self.pattern.begin(self.next_distance) {
            self.next_distance = distance;
        } else {
            self.done = true;
        }

        EndpointId::INVALID
    }

    fn line_to(&mut self, to: Point<T>) -> EndpointId {
        debug_assert!(!self.need_moveto);

        let v = to - self.prev;
        let d = v.length();

        if d < T::EPSILON {
            return EndpointId::INVALID;
        }

        let tangent = v / d;

        let mut distance = self.leftover + d;
        while distance >= self.next_distance {
            let position = self.prev + tangent * (self.next_distance - self.leftover);
            self.prev = position;
            self.leftover = T::ZERO;
            self.advancement += self.next_distance;
            distance -= self.next_distance;

            if let Some(distance) = self.pattern.next(position, tangent, self.advancement) {
                self.next_distance = distance;
            } else {
                self.done = true;
                return EndpointId::INVALID;
            }
        }

        self.prev = to;
        self.leftover = distance;

        EndpointId::INVALID
    }

    fn end(&mut self, close: bool) {
        if close {
            let first = self.first;
            self.line_to(first);
            self.need_moveto = true;
        }
    }

    fn quadratic_bezier_to(&mut self, ctrl: Point<T>, to: Point<T>) -> EndpointId {
        let curve = QuadraticBezierSegment {
            from: self.prev,
            ctrl,
            to,
        };
        curve.for_each_flattened(T::value(0.01), &mut |p| {
            self.line_to(p);
        });

        EndpointId::INVALID
    }

    fn cubic_bezier_to(&mut self, ctrl1: Point<T>, ctrl2: Point<T>, to: Point<T>) -> EndpointId {
        let curve = CubicBezierSegment {
            from: self.prev,
            ctrl1,
            ctrl2,
            to,
        };
        curve.for_each_flattened(T::value(0.01), &mut |p| {
            self.line_to(p);
        });

        EndpointId::INVALID
    }
}

/// A simple pattern that invokes a callback at regular intervals.
///
/// If the callback returns false, path walking stops.
pub struct RegularPattern<T: Scalar, Cb> {
    /// The function to call at each step.
    pub callback: Cb,
    /// A constant interval between each step.
    pub interval: T,
}

impl<T: Scalar, Cb> Pattern<T> for RegularPattern<T,Cb>
where
    Cb: FnMut(Point<T>, Vector<T>, T) -> bool,
{
    #[inline]
    fn next(&mut self, position: Point<T>, tangent: Vector<T>, distance: T) -> Option<T> {
        if !(self.callback)(position, tangent, distance) {
            return None;
        }
        Some(self.interval)
    }
}

/// A pattern that invokes a callback at a repeated sequence of
/// constant intervals.
///
/// If the callback returns false, path walking stops.
pub struct RepeatedPattern<'l, T: Scalar, Cb> {
    /// The function to call at each step.
    pub callback: Cb,
    /// The repeated interval sequence.
    pub intervals: &'l [T],
    /// The index of the next interval in the sequence.
    pub index: usize,
}

impl<'l, T: Scalar, Cb> Pattern<T> for RepeatedPattern<'l, T, Cb>
where
    Cb: FnMut(Point<T>, Vector<T>, T) -> bool,
{
    #[inline]
    fn next(&mut self, position: Point<T>, tangent: Vector<T>, distance: T) -> Option<T> {
        if !(self.callback)(position, tangent, distance) {
            return None;
        }
        let idx = self.index % self.intervals.len();
        self.index += 1;
        Some(self.intervals[idx])
    }
}

impl<T: Scalar, Cb> Pattern<T> for Cb
where
    Cb: FnMut(Point<T>, Vector<T>, T) -> Option<T>,
{
    #[inline]
    fn next(&mut self, position: Point<T>, tangent: Vector<T>, distance: T) -> Option<T> {
        (self)(position, tangent, distance)
    }
}

#[test]
fn walk_square() {
    let expected = [
        (point(0.0, 0.0), vector(1.0, 0.0), 0.0),
        (point(2.0, 0.0), vector(1.0, 0.0), 2.0),
        (point(4.0, 0.0), vector(1.0, 0.0), 4.0),
        (point(6.0, 0.0), vector(1.0, 0.0), 6.0),
        (point(6.0, 2.0), vector(0.0, 1.0), 8.0),
        (point(6.0, 4.0), vector(0.0, 1.0), 10.0),
        (point(6.0, 6.0), vector(0.0, 1.0), 12.0),
        (point(4.0, 6.0), vector(-1.0, 0.0), 14.0),
        (point(2.0, 6.0), vector(-1.0, 0.0), 16.0),
        (point(0.0, 6.0), vector(-1.0, 0.0), 18.0),
        (point(0.0, 4.0), vector(0.0, -1.0), 20.0),
        (point(0.0, 2.0), vector(0.0, -1.0), 22.0),
        (point(0.0, 0.0), vector(0.0, -1.0), 24.0),
    ];

    let mut i = 0;
    let mut pattern = RegularPattern {
        interval: 2.0,
        callback: |pos, n, d| {
            println!("p:{:?} n:{:?} d:{:?}", pos, n, d);
            assert_eq!(pos, expected[i].0);
            assert_eq!(n, expected[i].1);
            assert_eq!(d, expected[i].2);
            i += 1;
            true
        },
    };

    let mut walker = PathWalker::new(0.0, &mut pattern);

    walker.begin(point(0.0, 0.0));
    walker.line_to(point(6.0, 0.0));
    walker.line_to(point(6.0, 6.0));
    walker.line_to(point(0.0, 6.0));
    walker.close();
}

#[test]
fn walk_with_leftover() {
    let expected = [
        (point(1.0, 0.0), vector(1.0, 0.0), 1.0),
        (point(4.0, 0.0), vector(1.0, 0.0), 4.0),
        (point(5.0, 2.0), vector(0.0, 1.0), 7.0),
        (point(5.0, 5.0), vector(0.0, 1.0), 10.0),
        (point(2.0, 5.0), vector(-1.0, 0.0), 13.0),
        (point(0.0, 4.0), vector(0.0, -1.0), 16.0),
        (point(0.0, 1.0), vector(0.0, -1.0), 19.0),
    ];

    let mut i = 0;
    let mut pattern = RegularPattern {
        interval: 3.0,
        callback: |pos, n, d| {
            println!("p:{:?} n:{:?} d:{:?}", pos, n, d);
            assert_eq!(pos, expected[i].0);
            assert_eq!(n, expected[i].1);
            assert_eq!(d, expected[i].2);
            i += 1;
            true
        },
    };

    let mut walker = PathWalker::new(1.0, &mut pattern);

    walker.begin(point(0.0, 0.0));
    walker.line_to(point(5.0, 0.0));
    walker.line_to(point(5.0, 5.0));
    walker.line_to(point(0.0, 5.0));
    walker.close();
}

#[test]
fn walk_starting_after() {
    // With a starting distance that is greater than the path, the
    // callback should never be called.
    let cb = &mut |_, _, _| -> Option<f32> { panic!() };
    let mut walker = PathWalker::new(10.0, cb);

    walker.begin(point(0.0, 0.0));
    walker.line_to(point(5.0, 0.0));
    walker.end(false);
}

#[test]
fn walk_abort_early() {
    let mut callback_counter = 0;
    let mut pattern = RegularPattern {
        interval: 3.0,
        callback: |_pos, _n, _d| {
            callback_counter += 1;
            false
        },
    };

    let mut walker = PathWalker::new(1.0, &mut pattern);

    walker.begin(point(0.0, 0.0));
    walker.line_to(point(100.0, 0.0));

    assert_eq!(callback_counter, 1);
}
