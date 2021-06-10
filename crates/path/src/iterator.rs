//! Tools to iterate over paths.
//!
//! # Lyon path iterators
//!
//! ## Overview
//!
//! This module provides a collection of traits to extend the `Iterator` trait when
//! iterating over paths.
//!
//! ## Examples
//!
//! ```
//! use lyon_path::iterator::*;
//! use lyon_path::math::{point, vector};
//! use lyon_path::geom::BezierSegment;
//! use lyon_path::{Path, PathEvent};
//!
//! fn main() {
//!     // Start with a path.
//!     let mut builder = Path::builder();
//!     builder.begin(point(0.0, 0.0));
//!     builder.line_to(point(10.0, 0.0));
//!     builder.cubic_bezier_to(point(10.0, 10.0), point(0.0, 10.0), point(0.0, 5.0));
//!     builder.end(true);
//!     let path = builder.build();
//!
//!     // A simple std::iter::Iterator<PathEvent>,
//!     let simple_iter = path.iter();
//!
//!     // Make it an iterator over simpler primitives flattened events,
//!     // which do not contain any curve. To do so we approximate each curve
//!     // linear segments according to a tolerance threshold which controls
//!     // the tradeoff between fidelity of the approximation and amount of
//!     // generated events. Let's use a tolerance threshold of 0.01.
//!     // The beauty of this approach is that the flattening happens lazily
//!     // while iterating without allocating memory for the path.
//!     let flattened_iter = path.iter().flattened(0.01);
//!
//!     for evt in flattened_iter {
//!         match evt {
//!             PathEvent::Begin { at } => { println!(" - move to {:?}", at); }
//!             PathEvent::Line { from, to } => { println!(" - line {:?} -> {:?}", from, to); }
//!             PathEvent::End { last, first, close } => {
//!                 if close {
//!                     println!(" - close {:?} -> {:?}", last, first);
//!                 } else {
//!                     println!(" - end");
//!                 }
//!             }
//!             _ => { panic!() }
//!         }
//!     }
//!
//!     // Sometimes, working with segments directly without dealing with Begin/End events
//!     // can be more convenient:
//!     for segment in path.iter().bezier_segments() {
//!         match segment {
//!             BezierSegment::Linear(segment) => { println!("{:?}", segment); }
//!             BezierSegment::Quadratic(segment) => { println!("{:?}", segment); }
//!             BezierSegment::Cubic(segment) => { println!("{:?}", segment); }
//!         }
//!     }
//! }
//! ```
//!
//! Chaining the provided iterators allow performing some path manipulations lazily
//! without allocating actual path objects to hold the result of the transformations.
//!
//! ```
//! extern crate lyon_path;
//! use lyon_path::iterator::*;
//! use lyon_path::math::{point, Angle, Rotation};
//! use lyon_path::Path;
//!
//! fn main() {
//!     // In practice it is more common to iterate over Path objects than vectors
//!     // of SVG commands (the former can be constructed from the latter).
//!     let mut builder = Path::builder();
//!     builder.begin(point(1.0, 1.0));
//!     builder.line_to(point(2.0, 1.0));
//!     builder.quadratic_bezier_to(point(2.0, 2.0), point(1.0, 2.0));
//!     builder.cubic_bezier_to(point(0.0, 2.0), point(0.0, 0.0), point(1.0, 0.0));
//!     builder.end(true);
//!     let path = builder.build();
//!
//!     let transform = Rotation::new(Angle::radians(1.0));
//!
//!     for evt in path.iter().transformed(&transform).bezier_segments() {
//!         // ...
//!     }
//! }
//! ```

use lyon_geom::Scalar;

use crate::geom::traits::Transformation;
use crate::geom::{
    cubic_bezier, quadratic_bezier, BezierSegment, CubicBezierSegment, LineSegment,
    QuadraticBezierSegment,
};
use crate::math::*;
use crate::PathEvent;

/// An extension trait for `PathEvent` iterators.
pub trait PathIterator<T: Scalar>: Iterator<Item = PathEvent<T>> + Sized {
    /// Returns an iterator that turns curves into line segments.
    fn flattened(self, tolerance: T) -> Flattened<T, Self> {
        Flattened::new(tolerance, self)
    }

    /// Returns an iterator applying a 2D transform to all of its events.
    fn transformed<'l, U: Transformation<T>>(self, mat: &'l U) -> Transformed<'l, Self, U> {
        Transformed::new(mat, self)
    }

    /// Returns an iterator of segments.
    fn bezier_segments(self) -> BezierSegments<Self> {
        BezierSegments { iter: self }
    }
}

impl<T: Scalar, Iter> PathIterator<T> for Iter where Iter: Iterator<Item = PathEvent<T>> {}

/// An iterator that consumes `Event` iterator and yields flattend path events (with no curves).
pub struct Flattened<T: Scalar, Iter> {
    it: Iter,
    current_position: Point<T>,
    current_curve: TmpFlatteningIter<T>,
    tolerance: T,
}

enum TmpFlatteningIter<T: Scalar> {
    Quadratic(quadratic_bezier::Flattened<T>),
    Cubic(cubic_bezier::Flattened<T>),
    None,
}

impl<T: Scalar, Iter: Iterator<Item = PathEvent<T>>> Flattened<T, Iter> {
    /// Create the iterator.
    pub fn new(tolerance: T, it: Iter) -> Self {
        Flattened {
            it,
            current_position: point(T::ZERO, T::ZERO),
            current_curve: TmpFlatteningIter::None,
            tolerance,
        }
    }
}

impl<T: Scalar, Iter> Iterator for Flattened<T, Iter>
where
    Iter: Iterator<Item = PathEvent<T>>,
{
    type Item = PathEvent<T>;
    fn next(&mut self) -> Option<PathEvent<T>> {
        match self.current_curve {
            TmpFlatteningIter::Quadratic(ref mut it) => {
                if let Some(to) = it.next() {
                    let from = self.current_position;
                    self.current_position = to;
                    return Some(PathEvent::Line { from, to });
                }
            }
            TmpFlatteningIter::Cubic(ref mut it) => {
                if let Some(to) = it.next() {
                    let from = self.current_position;
                    self.current_position = to;
                    return Some(PathEvent::Line { from, to });
                }
            }
            _ => {}
        }
        self.current_curve = TmpFlatteningIter::None;
        match self.it.next() {
            Some(PathEvent::Begin { at }) => Some(PathEvent::Begin { at }),
            Some(PathEvent::Line { from, to }) => Some(PathEvent::Line { from, to }),
            Some(PathEvent::End { last, first, close }) => {
                Some(PathEvent::End { last, first, close })
            }
            Some(PathEvent::Quadratic { from, ctrl, to }) => {
                self.current_position = from;
                self.current_curve = TmpFlatteningIter::Quadratic(
                    QuadraticBezierSegment { from, ctrl, to }.flattened(self.tolerance),
                );
                self.next()
            }
            Some(PathEvent::Cubic {
                from,
                ctrl1,
                ctrl2,
                to,
            }) => {
                self.current_position = from;
                self.current_curve = TmpFlatteningIter::Cubic(
                    CubicBezierSegment {
                        from,
                        ctrl1,
                        ctrl2,
                        to,
                    }
                    .flattened(self.tolerance),
                );
                self.next()
            }
            None => None,
        }
    }
}

/// Applies a 2D transform to a path iterator and yields the resulting path iterator.
pub struct Transformed<'l, I, T> {
    it: I,
    transform: &'l T,
}

impl<'l, I, T: Scalar, U: Transformation<T>> Transformed<'l, I, U>
where
    I: Iterator<Item = PathEvent<T>>,
{
    /// Creates a new transformed path iterator from a path iterator.
    #[inline]
    pub fn new(transform: &'l U, it: I) -> Transformed<'l, I, U> {
        Transformed { it, transform }
    }
}

impl<'l, I, T: Scalar, U> Iterator for Transformed<'l, I, U>
where
    I: Iterator<Item = PathEvent<T>>,
    U: Transformation<T>,
{
    type Item = PathEvent<T>;
    fn next(&mut self) -> Option<PathEvent<T>> {
        match self.it.next() {
            None => None,
            Some(ref evt) => Some(evt.transformed(self.transform)),
        }
    }
}

/// An iterator that consumes an iterator of `Point`s and produces `Event`s.
///
/// # Example
///
/// ```
/// # extern crate lyon_path;
/// # use lyon_path::iterator::FromPolyline;
/// # use lyon_path::math::point;
/// # fn main() {
/// let points = [
///     point(1.0, 1.0),
///     point(2.0, 1.0),
///     point(1.0, 2.0)
/// ];
/// let iter = FromPolyline::closed(points.iter().cloned());
/// # }
/// ```
pub struct FromPolyline<T: Scalar, Iter> {
    iter: Iter,
    current: Point<T>,
    first: Point<T>,
    is_first: bool,
    done: bool,
    close: bool,
}

impl<T: Scalar, Iter: Iterator<Item = Point<T>>> FromPolyline<T, Iter> {
    pub fn new(close: bool, iter: Iter) -> Self {
        FromPolyline {
            iter,
            current: point(T::ZERO, T::ZERO),
            first: point(T::ZERO, T::ZERO),
            is_first: true,
            done: false,
            close,
        }
    }

    pub fn closed(iter: Iter) -> Self {
        FromPolyline::new(true, iter)
    }

    pub fn open(iter: Iter) -> Self {
        FromPolyline::new(false, iter)
    }
}

impl<T: Scalar, Iter> Iterator for FromPolyline<T, Iter>
where
    Iter: Iterator<Item = Point<T>>,
{
    type Item = PathEvent<T>;

    fn next(&mut self) -> Option<PathEvent<T>> {
        if self.done {
            return None;
        }

        if let Some(next) = self.iter.next() {
            debug_assert!(next.x.is_finite());
            debug_assert!(next.y.is_finite());
            let from = self.current;
            self.current = next;
            return if self.is_first {
                self.is_first = false;
                self.first = next;
                Some(PathEvent::Begin { at: next })
            } else {
                Some(PathEvent::Line { from, to: next })
            };
        }

        self.done = true;

        Some(PathEvent::End {
            last: self.current,
            first: self.first,
            close: self.close,
        })
    }
}

/// Turns an iterator of `Event` into an iterator of `BezierSegment<f32>`.
pub struct BezierSegments<Iter> {
    iter: Iter,
}

impl<T: Scalar, Iter> Iterator for BezierSegments<Iter>
where
    Iter: Iterator<Item = PathEvent<T>>,
{
    type Item = BezierSegment<T>;
    fn next(&mut self) -> Option<BezierSegment<T>> {
        match self.iter.next() {
            Some(PathEvent::Line { from, to }) => {
                Some(BezierSegment::Linear(LineSegment { from, to }))
            }
            Some(PathEvent::End {
                last,
                first,
                close: true,
            }) => Some(BezierSegment::Linear(LineSegment {
                from: last,
                to: first,
            })),
            Some(PathEvent::End { close: false, .. }) => self.next(),
            Some(PathEvent::Quadratic { from, ctrl, to }) => {
                Some(BezierSegment::Quadratic(QuadraticBezierSegment {
                    from,
                    ctrl,
                    to,
                }))
            }
            Some(PathEvent::Cubic {
                from,
                ctrl1,
                ctrl2,
                to,
            }) => Some(BezierSegment::Cubic(CubicBezierSegment {
                from,
                ctrl1,
                ctrl2,
                to,
            })),
            Some(PathEvent::Begin { .. }) => self.next(),
            None => None,
        }
    }
}

#[test]
fn test_from_polyline_open() {
    let points = &[
        point(1.0, 1.0),
        point(3.0, 1.0),
        point(4.0, 5.0),
        point(5.0, 2.0),
    ];

    let mut evts = FromPolyline::open(points.iter().cloned());

    assert_eq!(
        evts.next(),
        Some(PathEvent::Begin {
            at: point(1.0, 1.0)
        })
    );
    assert_eq!(
        evts.next(),
        Some(PathEvent::Line {
            from: point(1.0, 1.0),
            to: point(3.0, 1.0)
        })
    );
    assert_eq!(
        evts.next(),
        Some(PathEvent::Line {
            from: point(3.0, 1.0),
            to: point(4.0, 5.0)
        })
    );
    assert_eq!(
        evts.next(),
        Some(PathEvent::Line {
            from: point(4.0, 5.0),
            to: point(5.0, 2.0)
        })
    );
    assert_eq!(
        evts.next(),
        Some(PathEvent::End {
            last: point(5.0, 2.0),
            first: point(1.0, 1.0),
            close: false
        })
    );
    assert_eq!(evts.next(), None);
}

#[test]
fn test_from_polyline_closed() {
    let points = &[
        point(1.0, 1.0),
        point(3.0, 1.0),
        point(4.0, 5.0),
        point(5.0, 2.0),
    ];

    let mut evts = FromPolyline::closed(points.iter().cloned());

    assert_eq!(
        evts.next(),
        Some(PathEvent::Begin {
            at: point(1.0, 1.0)
        })
    );
    assert_eq!(
        evts.next(),
        Some(PathEvent::Line {
            from: point(1.0, 1.0),
            to: point(3.0, 1.0)
        })
    );
    assert_eq!(
        evts.next(),
        Some(PathEvent::Line {
            from: point(3.0, 1.0),
            to: point(4.0, 5.0)
        })
    );
    assert_eq!(
        evts.next(),
        Some(PathEvent::Line {
            from: point(4.0, 5.0),
            to: point(5.0, 2.0)
        })
    );
    assert_eq!(
        evts.next(),
        Some(PathEvent::End {
            last: point(5.0, 2.0),
            first: point(1.0, 1.0),
            close: true
        })
    );
    assert_eq!(evts.next(), None);
}
