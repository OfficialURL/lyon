//! Various math tools that are mostly usefull for the tessellators.

use lyon_path::geom::Scalar;

use crate::math::*;

/// Compute a normal vector at a point P such that ```x ---e1----> P ---e2---> x```
///
/// The resulting vector is not normalized. The length is such that extruding the shape
/// would yield parallel segments exactly 1 unit away from their original. (useful
/// for generating strokes and vertex-aa).
/// The normal points towards the left side of e1.
///
/// v1 and v2 are expected to be normalized.
pub fn compute_normal<T: Scalar>(v1: Vector<T>, v2: Vector<T>) -> Vector<T> {
    //debug_assert!((v1.length() - 1.0).abs() < 0.001, "v1 should be normalized ({})", v1.length());
    //debug_assert!((v2.length() - 1.0).abs() < 0.001, "v2 should be normalized ({})", v2.length());

    let epsilon = T::EPSILON*T::TEN;

    let n1 = vector(-v1.y, v1.x);

    let v12 = v1 + v2;

    if v12.square_length() < epsilon {
        return n1;
    }

    let tangent = v12.normalize();
    let n = vector(-tangent.y, tangent.x);

    let inv_len = n.dot(n1);

    if inv_len.abs() < epsilon {
        return n1;
    }

    n / inv_len
}

#[test]
fn test_compute_normal() {
    fn assert_almost_eq<T: Scalar>(a: Vector<T>, b: Vector<T>) {
        if (a - b).square_length() > T::EPSILON {
            panic!("assert almost equal: {:?} != {:?}", a, b);
        }
    }

    fn test_compute_normal<T: Scalar>() {
        assert_almost_eq(
            compute_normal(vector(T::ONE, T::ZERO), vector(T::ZERO, T::ONE)),
            vector(-T::ONE, T::ONE),
        );
        assert_almost_eq(
            compute_normal(vector(T::ONE, T::ZERO), vector(T::ONE, T::ZERO)),
            vector(T::ZERO, T::ONE),
        );
    }

    test_compute_normal::<f32>();
    test_compute_normal::<f64>();
}
