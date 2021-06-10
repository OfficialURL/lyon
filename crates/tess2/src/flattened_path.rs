use tessellation::geom::Scalar;

use crate::math::*;
use crate::path::builder::*;
use crate::path::private::{flatten_cubic_bezier, flatten_quadratic_bezier};
use crate::path::EndpointId;

use std::ops::Range;

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
struct SubPathInfo {
    range: Range<usize>,
    is_closed: bool,
}

/// A path data structure for pre-flattened paths and polygons.
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Default)]
pub struct FlattenedPath<T: Scalar> {
    points: Vec<Point<T>>,
    sub_paths: Vec<SubPathInfo>,
}

impl<T: Scalar> FlattenedPath<T> {
    /// Creates an empty path.
    pub fn new() -> Self {
        FlattenedPath {
            points: Vec::new(),
            sub_paths: Vec::new(),
        }
    }

    /// Creates a builder for flattened paths.
    pub fn builder(tolerance: T) -> Builder<T> {
        Builder::new(tolerance)
    }

    /// Returns whether the path is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Returns a slice of all the points in the path.
    pub fn points(&self) -> &[Point<T>] {
        &self.points
    }

    /// Returns an iterator over the sub-paths.
    pub fn sub_paths(&self) -> SubPaths<T> {
        SubPaths {
            points: &self.points,
            sub_paths: &self.sub_paths,
        }
    }

    /// Returns the nth sub-paths.
    pub fn sub_path(&self, index: usize) -> SubPath<T> {
        SubPath {
            points: &self.points[self.sub_paths[index].range.clone()],
            is_closed: self.sub_paths[index].is_closed,
        }
    }

    /// The number of sub-paths.
    pub fn num_sub_paths(&self) -> usize {
        self.sub_paths.len()
    }
}

/// An iterator of the sub paths of a flattened path.
pub struct SubPaths<'l, T: Scalar> {
    points: &'l [Point<T>],
    sub_paths: &'l [SubPathInfo],
}

impl<'l, T: Scalar> SubPaths<'l, T> {
    pub fn all_points(&self) -> &[Point<T>] {
        &self.points[self.sub_paths[0].range.clone()]
    }

    pub fn sub_path(&self, index: usize) -> SubPath<'l, T> {
        SubPath {
            points: &self.points[self.sub_paths[index].range.clone()],
            is_closed: self.sub_paths[index].is_closed,
        }
    }

    pub fn num_sub_paths(&self) -> usize {
        self.sub_paths.len()
    }
}

impl<'l, T: Scalar> Iterator for SubPaths<'l, T> {
    type Item = SubPath<'l, T>;
    fn next(&mut self) -> Option<SubPath<'l, T>> {
        if self.sub_paths.is_empty() {
            return None;
        }

        let sp = self.sub_paths[0].clone();
        self.sub_paths = &self.sub_paths[1..];

        Some(SubPath {
            points: &self.points[sp.range],
            is_closed: sp.is_closed,
        })
    }
}

/// An iterator over the points of a sub-path.
pub struct SubPath<'l, T: Scalar> {
    points: &'l [Point<T>],
    is_closed: bool,
}

impl<'l, T: Scalar> SubPath<'l, T> {
    /// Returns a slice of the points of this sub-path.
    pub fn points(&self) -> &'l [Point<T>] {
        self.points
    }

    /// Returns whether this sub-path is closed.
    pub fn is_closed(&self) -> bool {
        self.is_closed
    }
}

/// A builder for flattened paths.
#[derive(Default)]
pub struct Builder<T: Scalar> {
    points: Vec<Point<T>>,
    sub_paths: Vec<SubPathInfo>,
    sp_start: usize,
    tolerance: T,
}

impl<T: Scalar> Builder<T> {
    pub fn new(tolerance: T) -> Self {
        Builder {
            points: Vec::new(),
            sub_paths: Vec::new(),
            sp_start: 0,
            tolerance,
        }
    }

    pub fn build(self) -> FlattenedPath<T> {
        FlattenedPath {
            points: self.points,
            sub_paths: self.sub_paths,
        }
    }

    pub fn with_svg(self, tolerance: T) -> WithSvg<T, Flattened<T, Self>> {
        WithSvg::new(Flattened::new(self, tolerance))
    }

    pub fn current_position(&self) -> Point<T> {
        *self.points.last().unwrap()
    }
}

impl<T: Scalar> Build for Builder<T> {
    type PathType = FlattenedPath<T>;

    fn build(self) -> FlattenedPath<T> {
        FlattenedPath {
            points: self.points,
            sub_paths: self.sub_paths,
        }
    }
}

impl<T: Scalar> PathBuilder<T> for Builder<T> {
    fn begin(&mut self, to: Point<T>) -> EndpointId {
        nan_check(to);
        let sp_end = self.points.len();
        if self.sp_start != sp_end {
            self.sub_paths.push(SubPathInfo {
                range: self.sp_start..sp_end,
                is_closed: false,
            });
        }
        self.sp_start = sp_end;
        self.points.push(to);

        EndpointId(sp_end as u32)
    }

    fn line_to(&mut self, to: Point<T>) -> EndpointId {
        nan_check(to);
        let id = EndpointId(self.points.len() as u32);
        self.points.push(to);

        id
    }

    fn end(&mut self, close: bool) {
        let sp_end = self.points.len();
        if self.sp_start != sp_end {
            self.sub_paths.push(SubPathInfo {
                range: self.sp_start..sp_end,
                is_closed: close,
            });
        }
        self.sp_start = sp_end;
    }

    fn quadratic_bezier_to(&mut self, ctrl: Point<T>, to: Point<T>) -> EndpointId {
        flatten_quadratic_bezier(self.tolerance, self.current_position(), ctrl, to, self)
    }

    fn cubic_bezier_to(&mut self, ctrl1: Point<T>, ctrl2: Point<T>, to: Point<T>) -> EndpointId {
        flatten_cubic_bezier(
            self.tolerance,
            self.current_position(),
            ctrl1,
            ctrl2,
            to,
            self,
        )
    }
}

#[inline]
fn nan_check<T:Scalar>(p: Point<T>) {
    debug_assert!(!p.x.is_nan());
    debug_assert!(!p.y.is_nan());
}
