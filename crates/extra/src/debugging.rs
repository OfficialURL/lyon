use path::builder::*;
use path::geom::Scalar;
use path::math::Point;
use path::PathEvent;
use path::{Path, PathSlice};
use svg;

pub type Polygons<T> = Vec<Vec<Point<T>>>;

pub fn path_to_polygons<T: Scalar>(path: PathSlice<T>) -> Polygons<T> {
    let mut polygons = Vec::new();
    let mut poly = Vec::new();
    for evt in path {
        match evt {
            PathEvent::Begin { at } => {
                if poly.len() > 0 {
                    polygons.push(poly);
                }
                poly = vec![at];
            }
            PathEvent::Line { to, .. } => {
                poly.push(to);
            }
            PathEvent::End { .. } => {
                if !poly.is_empty() {
                    polygons.push(poly);
                }
                poly = Vec::new();
            }
            _ => {
                println!(
                    " -- path_to_polygons: warning! Unsupported event type {:?}",
                    evt
                );
            }
        }
    }
    return polygons;
}

pub fn polygons_to_path<T: Scalar>(polygons: &Polygons<T>) -> Path<T> {
    let mut builder = Path::builder().flattened(T::value(0.05));
    for poly in polygons.iter() {
        builder.begin(poly[0]);
        for i in 1..poly.len() {
            builder.line_to(poly[i]);
        }
        builder.close();
    }
    return builder.build();
}

pub fn find_reduced_test_case<
    T: Scalar,
    F: Fn(Path<T>) -> bool + panic::UnwindSafe + panic::RefUnwindSafe,
>(
    path: PathSlice<T>,
    cb: &F,
) -> Path<T> {
    let mut polygons = path_to_polygons(path);

    println!(" -- removing sub-paths...");

    polygons = find_reduced_test_case_sp(polygons, cb);

    println!(" -- removing vertices...");

    for p in 0..polygons.len() {
        let mut v = 0;
        loop {
            if v >= polygons[p].len() || polygons[p].len() <= 3 {
                break;
            }

            let mut cloned = polygons.clone();
            cloned[p].remove(v);

            let path = polygons_to_path(&cloned);

            let failed = panic::catch_unwind(|| cb(path)).unwrap_or(true);

            if failed {
                polygons = cloned;
                continue;
            }

            v += 1;
        }
    }

    let mut svg_path = svg::path_utils::PathSerializer::new();
    println!(" ----------- reduced test case: -----------\n\n");
    println!("#[test]");
    println!("fn reduced_test_case() {{");
    println!("    let mut builder = Path::builder();\n");
    for p in 0..polygons.len() {
        let pos = polygons[p][0];
        println!("    builder.begin(point({:.}, {:.}));", pos.x, pos.y);
        svg_path.move_to(pos);
        for v in 1..polygons[p].len() {
            let pos = polygons[p][v];
            println!("    builder.line_to(point({:.}, {:.}));", pos.x, pos.y);
            svg_path.line_to(pos);
        }
        println!("    builder.close();\n");
        svg_path.close();
    }
    println!("    test_path(builder.build().as_slice());\n");
    println!("    // SVG path syntax:");
    println!("    // \"{}\"", svg_path.build());
    println!("}}\n\n");

    return polygons_to_path(&polygons);
}

use std::panic;

fn find_reduced_test_case_sp<T: Scalar, F>(mut polygons: Polygons<T>, cb: &F) -> Polygons<T>
where
    F: Fn(Path<T>) -> bool + panic::UnwindSafe + panic::RefUnwindSafe,
{
    let mut i = 0;
    loop {
        if i >= polygons.len() {
            return polygons;
        }

        let mut cloned = polygons.clone();
        cloned.remove(i);
        let path = polygons_to_path(&cloned);

        let failed = panic::catch_unwind(|| cb(path)).unwrap_or(true);

        if failed {
            polygons = cloned;
            continue;
        }

        i += 1;
    }
}
