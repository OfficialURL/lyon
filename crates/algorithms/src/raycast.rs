//! Find the first collision between a ray and a path.

use lyon_path::geom::Scalar;

use crate::geom::{CubicBezierSegment, Line, LineSegment, QuadraticBezierSegment};
use crate::math::{point, vector, Point, Vector};
use crate::path::PathEvent;

pub struct Ray<T: Scalar> {
    pub origin: Point<T>,
    pub direction: Vector<T>,
}

// Position and normal at the point of contact between a ray and a shape.
pub struct Hit<T: Scalar> {
    pub position: Point<T>,
    pub normal: Vector<T>,
}

// TODO: early out in the bézier/arc cases using bounding rect or circle
// to speed things up.

/// Find the closest collision between a ray and the path.
pub fn raycast_path<T: Scalar, Iter>(ray: &Ray<T>, path: Iter, tolerance: T) -> Option<Hit<T>>
where
    Iter: Iterator<Item = PathEvent<T>>,
{
    let ray_len = ray.direction.square_length();
    if ray_len == T::ZERO || ray_len.is_nan() {
        return None;
    }

    let mut state = RayCastInner {
        ray: Line {
            point: ray.origin,
            vector: ray.direction,
        },
        min_dot: T::max_value(),
        result: point(T::ZERO, T::ZERO),
        normal: vector(T::ZERO, T::ZERO),
    };

    for evt in path {
        match evt {
            PathEvent::Begin { .. } => {}
            PathEvent::Line { from, to } => {
                test_segment(&mut state, &LineSegment { from, to });
            }
            PathEvent::End { last, first, .. } => {
                test_segment(
                    &mut state,
                    &LineSegment {
                        from: last,
                        to: first,
                    },
                );
            }
            PathEvent::Quadratic { from, ctrl, to } => {
                let mut prev = from;
                QuadraticBezierSegment { from, ctrl, to }.for_each_flattened(tolerance, &mut |p| {
                    test_segment(&mut state, &LineSegment { from: prev, to: p });
                    prev = p;
                });
            }
            PathEvent::Cubic {
                from,
                ctrl1,
                ctrl2,
                to,
            } => {
                let mut prev = from;
                CubicBezierSegment {
                    from,
                    ctrl1,
                    ctrl2,
                    to,
                }
                .for_each_flattened(tolerance, &mut |p| {
                    test_segment(&mut state, &LineSegment { from: prev, to: p });
                    prev = p;
                });
            }
        }
    }

    if state.min_dot == T::max_value() {
        return None;
    }

    if state.normal.dot(ray.direction) > T::ZERO {
        state.normal = -state.normal;
    }

    Some(Hit {
        position: state.result,
        normal: state.normal.normalize(),
    })
}

struct RayCastInner<T: Scalar> {
    ray: Line<T>,
    min_dot: T,
    result: Point<T>,
    normal: Vector<T>,
}

fn test_segment<T: Scalar>(state: &mut RayCastInner<T>, segment: &LineSegment<T>) {
    if let Some(pos) = segment.line_intersection(&state.ray) {
        let dot = (pos - state.ray.point).dot(state.ray.vector);
        if dot >= T::ZERO && dot < state.min_dot {
            state.min_dot = dot;
            state.result = pos;
            let v = segment.to_vector();
            state.normal = vector(-v.y, v.x);
        }
    }
}

#[test]
fn test_raycast() {
    use crate::geom::euclid::approxeq::ApproxEq;
    use crate::path::Path;

    let mut builder = Path::builder();
    builder.begin(point(0.0, 0.0));
    builder.line_to(point(1.0, 0.0));
    builder.line_to(point(1.0, 1.0));
    builder.line_to(point(0.0, 1.0));
    builder.end(true);
    let path = builder.build();

    assert!(raycast_path(
        &Ray {
            origin: point(-1.0, 2.0),
            direction: vector(1.0, 0.0)
        },
        path.iter(),
        0.1
    )
    .is_none());

    let hit = raycast_path(
        &Ray {
            origin: point(-1.0, 0.5),
            direction: vector(1.0, 0.0),
        },
        path.iter(),
        0.1,
    )
    .unwrap();
    assert!(hit.position.approx_eq(&point(0.0, 0.5)));
    assert!(hit.normal.approx_eq(&vector(-1.0, 0.0)));

    let hit = raycast_path(
        &Ray {
            origin: point(-1.0, 0.0),
            direction: vector(1.0, 0.0),
        },
        path.iter(),
        0.1,
    )
    .unwrap();
    assert!(hit.position.approx_eq(&point(0.0, 0.0)));

    let hit = raycast_path(
        &Ray {
            origin: point(0.5, 0.5),
            direction: vector(1.0, 0.0),
        },
        path.iter(),
        0.1,
    )
    .unwrap();
    assert!(hit.position.approx_eq(&point(1.0, 0.5)));
    assert!(hit.normal.approx_eq(&vector(-1.0, 0.0)));

    let hit = raycast_path(
        &Ray {
            origin: point(0.0, -1.0),
            direction: vector(1.0, 1.0),
        },
        path.iter(),
        0.1,
    )
    .unwrap();
    assert!(hit.position.approx_eq(&point(1.0, 0.0)));
}
