//! Bounding rectangle computation for paths.

use lyon_path::geom::Scalar;

use crate::geom::{CubicBezierSegment, QuadraticBezierSegment};
use crate::math::{point, Point, Rect};
use crate::path::PathEvent;

/// Computes a conservative axis-aligned rectangle that contains the path.
///
/// This bounding rectangle approximation is faster but less precise than
/// [`building_rect`](fn.bounding_rect.html).
pub fn fast_bounding_rect<T: Scalar, Iter, Evt>(path: Iter) -> Rect<T>
where
    Iter: Iterator<Item = Evt>,
    Evt: FastBoundingRect<T>,
{
    let mut min = point(T::max_value(), T::max_value());
    let mut max = point(T::min_value(), T::min_value());
    for e in path {
        e.min_max(&mut min, &mut max);
    }

    // Return an empty rectangle by default if there was no event in the path.
    if min == point(T::max_value(), T::max_value()) {
        return Rect::zero();
    }

    Rect {
        origin: min,
        size: (max - min).to_size(),
    }
}

#[doc(hidden)]
pub trait FastBoundingRect<T: Scalar> {
    fn min_max(&self, min: &mut Point<T>, max: &mut Point<T>);
}

impl<T: Scalar> FastBoundingRect<T> for PathEvent<T> {
    fn min_max(&self, min: &mut Point<T>, max: &mut Point<T>) {
        match self {
            PathEvent::Begin { at } => {
                *min = Point::min(*min, *at);
                *max = Point::max(*max, *at);
            }
            PathEvent::Line { to, .. } => {
                *min = Point::min(*min, *to);
                *max = Point::max(*max, *to);
            }
            PathEvent::Quadratic { ctrl, to, .. } => {
                *min = Point::min(*min, Point::min(*ctrl, *to));
                *max = Point::max(*max, Point::max(*ctrl, *to));
            }
            PathEvent::Cubic {
                ctrl1, ctrl2, to, ..
            } => {
                *min = Point::min(*min, Point::min(*ctrl1, Point::min(*ctrl2, *to)));
                *max = Point::max(*max, Point::max(*ctrl1, Point::max(*ctrl2, *to)));
            }
            PathEvent::End { .. } => {}
        }
    }
}

/// Computes the smallest axis-aligned rectangle that contains the path.
pub fn bounding_rect<T: Scalar, Iter, Evt>(path: Iter) -> Rect<T>
where
    Iter: Iterator<Item = Evt>,
    Evt: TightBoundingRect<T>,
{
    let mut min = point(T::max_value(), T::max_value());
    let mut max = point(T::max_value(), T::max_value());

    for evt in path {
        evt.min_max(&mut min, &mut max);
    }

    // Return an empty rectangle by default if there was no event in the path.
    if min == point(T::max_value(), T::max_value()) {
        return Rect::zero();
    }

    Rect {
        origin: min,
        size: (max - min).to_size(),
    }
}

#[doc(hidden)]
pub trait TightBoundingRect<T: Scalar> {
    fn min_max(&self, min: &mut Point<T>, max: &mut Point<T>);
}

impl<T: Scalar> TightBoundingRect<T> for PathEvent<T> {
    fn min_max(&self, min: &mut Point<T>, max: &mut Point<T>) {
        match self {
            PathEvent::Begin { at } => {
                *min = Point::min(*min, *at);
                *max = Point::max(*max, *at);
            }
            PathEvent::Line { to, .. } => {
                *min = Point::min(*min, *to);
                *max = Point::max(*max, *to);
            }
            PathEvent::Quadratic { from, ctrl, to } => {
                let r = QuadraticBezierSegment {
                    from: *from,
                    ctrl: *ctrl,
                    to: *to,
                }
                .bounding_rect();
                *min = Point::min(*min, r.min());
                *max = Point::max(*max, r.max());
            }
            PathEvent::Cubic {
                from,
                ctrl1,
                ctrl2,
                to,
            } => {
                let r = CubicBezierSegment {
                    from: *from,
                    ctrl1: *ctrl1,
                    ctrl2: *ctrl2,
                    to: *to,
                }
                .bounding_rect();
                *min = Point::min(*min, r.min());
                *max = Point::max(*max, r.max());
            }
            PathEvent::End { .. } => {}
        }
    }
}

#[test]
fn simple_bounding_rect() {
    use crate::math::rect;
    use crate::path::Path;

    let mut builder = Path::builder();
    builder.begin(point(-10.0, -3.0));
    builder.line_to(point(0.0, -12.0));
    builder.quadratic_bezier_to(point(3.0, 4.0), point(5.0, 3.0));
    builder.end(true);
    let path = builder.build();

    assert_eq!(
        fast_bounding_rect(path.iter()),
        rect(-10.0, -12.0, 15.0, 16.0)
    );

    let mut builder = Path::builder();
    builder.begin(point(0.0, 0.0));
    builder.cubic_bezier_to(point(-1.0, 2.0), point(3.0, -4.0), point(1.0, -1.0));
    builder.end(false);
    let path = builder.build();

    assert_eq!(fast_bounding_rect(path.iter()), rect(-1.0, -4.0, 4.0, 6.0));
}
