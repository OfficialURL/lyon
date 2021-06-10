//! Fit paths into rectangles.

use lyon_path::geom::Scalar;

use crate::aabb::bounding_rect;
use crate::math::*;
use crate::path::iterator::*;
use crate::path::Path;

/// The strategy to use when fitting (stretching, overflow, etc.)
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FitStyle {
    /// Stretch vertically and horizontally to fit the destination rectangle exactly.
    Stretch,
    /// Uniformly scale without overflow.
    Min,
    /// Uniformly scale with overflow.
    Max,
    /// Uniformly scale to fit horizontally.
    Horizontal,
    /// Uniformly scale to fit vertically.
    Vertical,
}

/// Computes a transform that fits a rectangle into another one.
pub fn fit_rectangle<T: Scalar>(
    src_rect: &Rect<T>,
    dst_rect: &Rect<T>,
    style: FitStyle,
) -> Transform<T> {
    let scale = vector(
        dst_rect.size.width / src_rect.size.width,
        dst_rect.size.height / src_rect.size.height,
    );

    let scale = match style {
        FitStyle::Stretch => scale,
        FitStyle::Min => {
            let s = T::min(scale.x, scale.y);
            vector(s, s)
        }
        FitStyle::Max => {
            let s = T::max(scale.x, scale.y);
            vector(s, s)
        }
        FitStyle::Horizontal => vector(scale.x, scale.x),
        FitStyle::Vertical => vector(scale.y, scale.y),
    };

    let src_center = src_rect.origin.lerp(src_rect.max(), T::HALF);
    let dst_center = dst_rect.origin.lerp(dst_rect.max(), T::HALF);

    Transform::translation(-src_center.x, -src_center.y)
        .then_scale(scale.x, scale.y)
        .then_translate(dst_center.to_vector())
}

/// Fits a path into a rectangle.
pub fn fit_path<T: Scalar>(path: &Path<T>, output_rect: &Rect<T>, style: FitStyle) -> Path<T> {
    let aabb = bounding_rect(path.iter());
    let transform = fit_rectangle(&aabb, output_rect, style);

    let mut builder = Path::builder();
    for evt in path.iter().transformed(&transform) {
        builder.path_event(evt)
    }

    builder.build()
}

#[test]
fn simple_fit() {
    use crate::geom::euclid::approxeq::ApproxEq;

    fn approx_eq<T: Scalar>(a: &Rect<T>, b: &Rect<T>) -> bool
    where
        T: ApproxEq<T>,
    {
        let result = a.origin.approx_eq(&b.origin) && a.max().approx_eq(&b.max());
        if !result {
            println!("{:?} == {:?}", a, b);
        }
        result
    }

    let t = fit_rectangle(
        &rect(0.0, 0.0, 1.0, 1.0),
        &rect(0.0, 0.0, 2.0, 2.0),
        FitStyle::Stretch,
    );

    assert!(approx_eq(
        &t.outer_transformed_rect(&rect(0.0, 0.0, 1.0, 1.0)),
        &rect(0.0, 0.0, 2.0, 2.0)
    ));

    let t = fit_rectangle(
        &rect(1.0, 2.0, 4.0, 4.0),
        &rect(0.0, 0.0, 2.0, 8.0),
        FitStyle::Stretch,
    );

    assert!(approx_eq(
        &t.outer_transformed_rect(&rect(1.0, 2.0, 4.0, 4.0)),
        &rect(0.0, 0.0, 2.0, 8.0)
    ));

    let t = fit_rectangle(
        &rect(1.0, 2.0, 2.0, 4.0),
        &rect(0.0, 0.0, 2.0, 2.0),
        FitStyle::Horizontal,
    );

    assert!(approx_eq(
        &t.outer_transformed_rect(&rect(1.0, 2.0, 2.0, 4.0)),
        &rect(0.0, -1.0, 2.0, 4.0)
    ));

    let t = fit_rectangle(
        &rect(1.0, 2.0, 2.0, 2.0),
        &rect(0.0, 0.0, 4.0, 2.0),
        FitStyle::Horizontal,
    );

    assert!(approx_eq(
        &t.outer_transformed_rect(&rect(1.0, 2.0, 2.0, 2.0)),
        &rect(0.0, -1.0, 4.0, 4.0)
    ));
}
