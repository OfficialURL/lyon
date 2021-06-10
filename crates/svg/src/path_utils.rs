use lyon_path::geom::euclid::approxeq::ApproxEq;
use lyon_path::geom::Scalar;

use crate::parser::{PathParser, PathSegment};

use crate::path::builder::*;
use crate::path::geom::Arc;
use crate::path::math::{point, vector, Angle, Point, Vector};
use crate::path::ArcFlags;

#[derive(Clone, Debug, PartialEq)]
pub struct ParseError;

/// Builds path object using an SvgPathBuilder and a list of commands.
/// Once the path is built you can tessellate it.
///
/// The [SvgPathBuilder](trait.SvgPathBuilder.html) Adds to [PathBuilder](trait.PathBuilder.html)
/// the rest of the [SVG path](https://svgwg.org/specs/paths/) commands.
///
/// # Examples
///
/// ```
/// # extern crate lyon_svg as svg;
/// # extern crate lyon_path;
/// # use lyon_path::Path;
/// # use svg::path_utils::build_path;
/// # fn main() {
/// // Create a simple path.
/// let commands = &"M 0 0 L 10 0 L 10 10 L 0 10 z";
/// let svg_builder = Path::builder().with_svg();
/// let path = build_path(svg_builder, commands);
/// # }
/// ```
pub fn build_path<T: Scalar, Builder>(
    mut builder: Builder,
    src: &str,
) -> Result<Builder::PathType, ParseError>
where
    T: ApproxEq<T>,
    Builder: SvgPathBuilder<T> + Build,
{
    for item in PathParser::from(src) {
        if let Ok(segment) = item {
            svg_event(&segment, &mut builder)
        }
    }

    Ok(builder.build())
}

fn svg_event<T: Scalar, Builder>(token: &PathSegment, builder: &mut Builder)
where
    T: ApproxEq<T>,
    Builder: SvgPathBuilder<T>,
{
    match *token {
        PathSegment::MoveTo { abs: true, x, y } => {
            builder.move_to(point(T::value(x), T::value(y)));
        }
        PathSegment::MoveTo { abs: false, x, y } => {
            builder.relative_move_to(vector(T::value(x), T::value(y)));
        }
        PathSegment::LineTo { abs: true, x, y } => {
            builder.line_to(point(T::value(x), T::value(y)));
        }
        PathSegment::LineTo { abs: false, x, y } => {
            builder.relative_line_to(vector(T::value(x), T::value(y)));
        }
        PathSegment::HorizontalLineTo { abs: true, x } => {
            builder.horizontal_line_to(T::value(x));
        }
        PathSegment::HorizontalLineTo { abs: false, x } => {
            builder.relative_horizontal_line_to(T::value(x));
        }
        PathSegment::VerticalLineTo { abs: true, y } => {
            builder.vertical_line_to(T::value(y));
        }
        PathSegment::VerticalLineTo { abs: false, y } => {
            builder.relative_vertical_line_to(T::value(y));
        }
        PathSegment::CurveTo {
            abs: true,
            x1,
            y1,
            x2,
            y2,
            x,
            y,
        } => {
            builder.cubic_bezier_to(
                point(T::value(x1), T::value(y1)),
                point(T::value(x2), T::value(y2)),
                point(T::value(x), T::value(y)),
            );
        }
        PathSegment::CurveTo {
            abs: false,
            x1,
            y1,
            x2,
            y2,
            x,
            y,
        } => {
            builder.relative_cubic_bezier_to(
                vector(T::value(x1), T::value(y1)),
                vector(T::value(x2), T::value(y2)),
                vector(T::value(x), T::value(y)),
            );
        }
        PathSegment::SmoothCurveTo {
            abs: true,
            x2,
            y2,
            x,
            y,
        } => {
            builder.smooth_cubic_bezier_to(
                point(T::value(x2), T::value(y2)),
                point(T::value(x), T::value(y)),
            );
        }
        PathSegment::SmoothCurveTo {
            abs: false,
            x2,
            y2,
            x,
            y,
        } => {
            builder.smooth_relative_cubic_bezier_to(
                vector(T::value(x2), T::value(y2)),
                vector(T::value(x), T::value(y)),
            );
        }
        PathSegment::Quadratic {
            abs: true,
            x1,
            y1,
            x,
            y,
        } => {
            builder.quadratic_bezier_to(
                point(T::value(x1), T::value(y1)),
                point(T::value(x), T::value(y)),
            );
        }
        PathSegment::Quadratic {
            abs: false,
            x1,
            y1,
            x,
            y,
        } => {
            builder.relative_quadratic_bezier_to(
                vector(T::value(x1), T::value(y1)),
                vector(T::value(x), T::value(y)),
            );
        }
        PathSegment::SmoothQuadratic { abs: true, x, y } => {
            builder.smooth_quadratic_bezier_to(point(T::value(x), T::value(y)));
        }
        PathSegment::SmoothQuadratic { abs: false, x, y } => {
            builder.smooth_relative_quadratic_bezier_to(vector(T::value(x), T::value(y)));
        }
        PathSegment::EllipticalArc {
            abs: true,
            rx,
            ry,
            x_axis_rotation,
            large_arc,
            sweep,
            x,
            y,
        } => {
            builder.arc_to(
                vector(T::value(rx), T::value(ry)),
                Angle::degrees(T::value(x_axis_rotation)),
                ArcFlags {
                    large_arc: large_arc,
                    sweep: sweep,
                },
                point(T::value(x), T::value(y)),
            );
        }
        PathSegment::EllipticalArc {
            abs: false,
            rx,
            ry,
            x_axis_rotation,
            large_arc,
            sweep,
            x,
            y,
        } => {
            builder.relative_arc_to(
                vector(T::value(rx), T::value(ry)),
                Angle::degrees(T::value(x_axis_rotation)),
                ArcFlags {
                    large_arc: large_arc,
                    sweep: sweep,
                },
                vector(T::value(x), T::value(y)),
            );
        }
        PathSegment::ClosePath { .. } => {
            builder.close();
        }
    }
}

/// An `SvgPathBuilder` that builds a `String` representation of the path
/// using the SVG syntax.
///
/// No effort is put into making the serializer fast or make the
/// output compact. Intended primarily for debugging purposes.
pub struct PathSerializer<T: Scalar> {
    path: String,
    current: Point<T>,
}

impl<T: Scalar> PathSerializer<T> {
    pub fn new() -> Self {
        PathSerializer {
            path: String::new(),
            current: point(T::ZERO, T::ZERO),
        }
    }

    pub fn arc(
        &mut self,
        center: Point<T>,
        radii: Vector<T>,
        sweep_angle: Angle<T>,
        x_rotation: Angle<T>,
    ) {
        let start_angle = (self.current - center).angle_from_x_axis() - x_rotation;
        let svg = Arc {
            center,
            radii,
            start_angle,
            sweep_angle,
            x_rotation,
        }
        .to_svg_arc();

        self.path += &format!(
            "A {} {} {} {} {} {} {}",
            radii.x,
            radii.y,
            svg.x_rotation.get(),
            svg.flags.large_arc,
            svg.flags.sweep,
            svg.to.x,
            svg.to.y
        );
    }
}

impl<T: Scalar> Build for PathSerializer<T> {
    type PathType = String;

    fn build(self) -> String {
        self.path
    }
}

impl<T: Scalar> SvgPathBuilder<T> for PathSerializer<T> {
    fn move_to(&mut self, to: Point<T>) {
        self.path += &format!("M {} {} ", to.x, to.y);
        self.current = to;
    }

    fn close(&mut self) {
        self.path.push_str("Z");
    }

    fn line_to(&mut self, to: Point<T>) {
        self.path += &format!("L {} {} ", to.x, to.y);
        self.current = to;
    }

    fn quadratic_bezier_to(&mut self, ctrl: Point<T>, to: Point<T>) {
        self.path += &format!("Q {} {} {} {}", ctrl.x, ctrl.y, to.x, to.y);
    }

    fn cubic_bezier_to(&mut self, ctrl1: Point<T>, ctrl2: Point<T>, to: Point<T>) {
        self.path += &format!(
            "C {} {} {} {} {} {}",
            ctrl1.x, ctrl1.y, ctrl2.x, ctrl2.y, to.x, to.y
        );
    }

    fn relative_move_to(&mut self, to: Vector<T>) {
        self.path += &format!("m {} {} ", to.x, to.y);
    }

    fn relative_line_to(&mut self, to: Vector<T>) {
        self.path += &format!("l {} {} ", to.x, to.y);
    }

    fn relative_quadratic_bezier_to(&mut self, ctrl: Vector<T>, to: Vector<T>) {
        self.path += &format!("q {} {} {} {}", ctrl.x, ctrl.y, to.x, to.y);
    }

    fn relative_cubic_bezier_to(&mut self, ctrl1: Vector<T>, ctrl2: Vector<T>, to: Vector<T>) {
        self.path += &format!(
            "c {} {} {} {} {} {}",
            ctrl1.x, ctrl1.y, ctrl2.x, ctrl2.y, to.x, to.y
        );
    }

    fn smooth_cubic_bezier_to(&mut self, ctrl2: Point<T>, to: Point<T>) {
        self.path += &format!("S {} {} {} {}", ctrl2.x, ctrl2.y, to.x, to.y);
    }

    fn smooth_relative_cubic_bezier_to(&mut self, ctrl2: Vector<T>, to: Vector<T>) {
        self.path += &format!("s {} {} {} {}", ctrl2.x, ctrl2.y, to.x, to.y);
    }

    fn smooth_quadratic_bezier_to(&mut self, to: Point<T>) {
        self.path += &format!("T {} {} ", to.x, to.y);
    }

    fn smooth_relative_quadratic_bezier_to(&mut self, to: Vector<T>) {
        self.path += &format!("t {} {} ", to.x, to.y);
    }

    fn horizontal_line_to(&mut self, x: T) {
        self.path += &format!("H {} ", x);
    }

    fn relative_horizontal_line_to(&mut self, dx: T) {
        self.path += &format!("h {} ", dx);
    }

    fn vertical_line_to(&mut self, y: T) {
        self.path += &format!("V {} ", y);
    }

    fn relative_vertical_line_to(&mut self, dy: T) {
        self.path += &format!("v {} ", dy);
    }

    fn arc_to(&mut self, radii: Vector<T>, x_rotation: Angle<T>, flags: ArcFlags, to: Point<T>) {
        self.path += &format!(
            "A {} {} {} {} {} {} {} ",
            radii.x,
            radii.y,
            x_rotation.get() * T::value(180.0) / T::PI(),
            if flags.large_arc { 1u32 } else { 0 },
            if flags.sweep { 1u32 } else { 0 },
            to.x,
            to.y
        );
    }

    fn relative_arc_to(
        &mut self,
        radii: Vector<T>,
        x_rotation: Angle<T>,
        flags: ArcFlags,
        to: Vector<T>,
    ) {
        self.path += &format!(
            "a {} {} {} {} {} {} {} ",
            radii.x,
            radii.y,
            x_rotation.get() * T::value(180.0) / T::PI(),
            if flags.large_arc { 1u32 } else { 0 },
            if flags.sweep { 1u32 } else { 0 },
            to.x,
            to.y
        );
    }
}
