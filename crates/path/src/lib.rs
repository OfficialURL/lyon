#![doc(html_logo_url = "https://nical.github.io/lyon-doc/lyon-logo.svg")]
#![deny(bare_trait_objects)]
#![deny(unconditional_recursion)]

//! Data structures and traits to work with paths (vector graphics).
//!
//! To build and consume paths, see the [builder](builder/index.html) and
//! [iterator](iterator/index.html) modules.
//!
//! This crate is reexported in [lyon](https://docs.rs/lyon/).
//!
//! # Examples
//!
//! ```
//! # extern crate lyon_path;
//! # fn main() {
//! use lyon_path::Path;
//! use lyon_path::math::{point};
//! use lyon_path::builder::*;
//!
//! // Create a builder object to build the path.
//! let mut builder = Path::builder();
//!
//! // Build a simple path.
//! let mut builder = Path::builder();
//! builder.begin(point(0.0, 0.0));
//! builder.line_to(point(1.0, 2.0));
//! builder.line_to(point(2.0, 0.0));
//! builder.line_to(point(1.0, 1.0));
//! builder.close();
//!
//! // Generate the actual path object.
//! let path = builder.build();
//!
//! for event in &path {
//!     println!("{:?}", event);
//! }
//! # }
//! ```
//!

use geom::Scalar;
pub use lyon_geom as geom;

#[cfg(feature = "serialization")]
#[macro_use]
pub extern crate serde;

pub mod builder;
pub mod commands;
mod events;
pub mod iterator;
pub mod path;
pub mod path_buffer;
pub mod polygon;

#[doc(hidden)]
pub mod private;

#[doc(inline)]
pub use crate::commands::{PathCommands, PathCommandsSlice};
pub use crate::events::*;
pub use crate::geom::ArcFlags;
#[doc(inline)]
pub use crate::path::{Path, PathSlice};
#[doc(inline)]
pub use crate::path_buffer::{PathBuffer, PathBufferSlice};
#[doc(inline)]
pub use crate::polygon::{IdPolygon, Polygon};

use math::Point;
use std::fmt;
use std::u32;

pub mod traits {
    //! `lyon_path` traits reexported here for convenience.

    pub use crate::builder::Build;
    pub use crate::builder::PathBuilder;
    pub use crate::builder::SvgPathBuilder;
    pub use crate::iterator::PathIterator;
}

pub mod math {
    //! Floating point version of the lyon_geom types used everywhere. Most other lyon crates
    //! reexport them.

    use lyon_geom::Scalar;

    use crate::geom::euclid;

    /// Alias for ```euclid::default::Point2D<T>```.
    pub type Point<T> = euclid::default::Point2D<T>;

    /// Alias for ```euclid::default::Point2D<T>```.
    pub type Vector<T> = euclid::default::Vector2D<T>;

    /// Alias for ```euclid::default::Size2D<T>```.
    pub type Size<T> = euclid::default::Size2D<T>;

    /// Alias for ```euclid::default::Rect<T>```
    pub type Rect<T> = euclid::default::Rect<T>;

    /// Alias for ```euclid::default::Transform2D<T>```
    pub type Transform<T> = euclid::default::Transform2D<T>;

    /// Alias for ```euclid::default::Rotation2D<T>```
    pub type Rotation<T> = euclid::default::Rotation2D<T>;

    /// Alias for ```euclid::default::Translation2D<T>```
    pub type Translation<T> = euclid::Translation2D<T, euclid::UnknownUnit, euclid::UnknownUnit>;

    /// Alias for ```euclid::default::Scale<T>```
    pub type Scale<T> = euclid::default::Scale<T>;

    /// An angle in radians (T).
    pub type Angle<T> = euclid::Angle<T>;

    /// Shorthand for `Rect::new(Point::new(x, y), Size::new(w, h))`.
    #[inline]
    pub fn rect<T: Scalar>(x: T, y: T, w: T, h: T) -> Rect<T> {
        Rect {
            origin: point(x, y),
            size: size(w, h),
        }
    }

    /// Shorthand for `Vector::new(x, y)`.
    #[inline]
    pub fn vector<T: Scalar>(x: T, y: T) -> Vector<T> {
        Vector::new(x, y)
    }

    /// Shorthand for `Point::new(x, y)`.
    #[inline]
    pub fn point<T: Scalar>(x: T, y: T) -> Point<T> {
        Point::new(x, y)
    }

    /// Shorthand for `Size::new(x, y)`.
    #[inline]
    pub fn size<T: Scalar>(w: T, h: T) -> Size<T> {
        Size::new(w, h)
    }
}

/// The fill rule defines how to determine what is inside and what is outside of the shape.
///
/// See the SVG specification.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
pub enum FillRule {
    EvenOdd,
    NonZero,
}

impl FillRule {
    #[inline]
    pub fn is_in(&self, winding_number: i16) -> bool {
        match *self {
            FillRule::EvenOdd => winding_number % 2 != 0,
            FillRule::NonZero => winding_number != 0,
        }
    }

    #[inline]
    pub fn is_out(&self, winding_number: i16) -> bool {
        !self.is_in(winding_number)
    }
}

/// The two possible orientations for the edges of a shape to be built in.
///
/// Positive winding corresponds to the positive orientation in trigonometry.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
pub enum Winding {
    Positive,
    Negative,
}

/// ID of a control point in a path.
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
pub struct ControlPointId(pub u32);

impl ControlPointId {
    pub const INVALID: Self = ControlPointId(u32::MAX);
    pub fn offset(self) -> usize {
        self.0 as usize
    }
    pub fn to_usize(self) -> usize {
        self.0 as usize
    }
    pub fn from_usize(val: usize) -> Self {
        ControlPointId(val as u32)
    }
}

impl fmt::Debug for ControlPointId {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "#{}", self.0)
    }
}

/// ID of an endpoint point in a path.
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
pub struct EndpointId(pub u32);
impl EndpointId {
    pub const INVALID: Self = EndpointId(u32::MAX);
    pub fn offset(self) -> usize {
        self.0 as usize
    }
    pub fn to_usize(self) -> usize {
        self.0 as usize
    }
    pub fn from_usize(val: usize) -> Self {
        EndpointId(val as u32)
    }
}

impl fmt::Debug for EndpointId {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "#{}", self.0)
    }
}

/// Refers to an event in a path.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
pub struct EventId(#[doc(hidden)] pub u32);

impl EventId {
    pub const INVALID: Self = EventId(std::u32::MAX);
    pub fn to_usize(&self) -> usize {
        self.0 as usize
    }
}

/// Interface for types types (typically endpoints and control points) that have
/// a 2D position.
pub trait Position<T: Scalar> {
    fn position(&self) -> Point<T>;
}

impl<T: Scalar, U> Position<T> for crate::geom::euclid::Point2D<T, U> {
    fn position(&self) -> Point<T> {
        self.to_untyped()
    }
}

impl<'l, T: Scalar, U: Position<T>> Position<T> for &'l U {
    fn position(&self) -> Point<T> {
        (*self).position()
    }
}

impl<T: Scalar> Position<T> for (T, T) {
    fn position(&self) -> Point<T> {
        Point::new(self.0, self.1)
    }
}

impl<T: Scalar> Position<T> for [T; 2] {
    fn position(&self) -> Point<T> {
        Point::new(self[0], self[1])
    }
}

/// Interface for objects storing endpoints and control points positions.
///
/// This interface can be implemented by path objects themselves or via external
/// data structures.
pub trait PositionStore<T: Scalar> {
    fn get_endpoint(&self, id: EndpointId) -> Point<T>;
    fn get_control_point(&self, id: ControlPointId) -> Point<T>;
}

impl<'l, T: Scalar> PositionStore<T> for (&'l [Point<T>], &'l [Point<T>]) {
    fn get_endpoint(&self, id: EndpointId) -> Point<T> {
        self.0[id.to_usize()]
    }
    fn get_control_point(&self, id: ControlPointId) -> Point<T> {
        self.1[id.to_usize()]
    }
}

/// Interface for objects storing custom attributes associated with endpoints.
///
/// This interface can be implemented by path objects themselves or via external
/// data structures.
pub trait AttributeStore<T: Scalar> {
    /// Returns the endpoint's custom attributes as a slice of 32 bits floats.
    ///
    /// The size of the slice must be equal to the result of `num_attributes()`.
    fn get(&self, id: EndpointId) -> &[T];

    /// Returns the number of float attributes per endpoint.
    ///
    /// All endpoints must have the same number of attributes.
    fn num_attributes(&self) -> usize;
}

impl<T: Scalar> AttributeStore<T> for () {
    fn num_attributes(&self) -> usize {
        0
    }
    fn get(&self, _: EndpointId) -> &[T] {
        &[]
    }
}

/// A view over a contiguous storage of custom attributes.
pub struct AttributeSlice<'l, T: Scalar> {
    data: &'l [T],
    stride: usize,
}

impl<'l, T: Scalar> AttributeSlice<'l, T> {
    pub fn new(data: &'l [T], num_attributes: usize) -> Self {
        AttributeSlice {
            data,
            stride: num_attributes,
        }
    }
}

impl<'l, T: Scalar> AttributeStore<T> for AttributeSlice<'l, T> {
    fn get(&self, id: EndpointId) -> &[T] {
        let start = id.to_usize() * self.stride;
        let end = start + self.stride;
        &self.data[start..end]
    }

    fn num_attributes(&self) -> usize {
        self.stride
    }
}
