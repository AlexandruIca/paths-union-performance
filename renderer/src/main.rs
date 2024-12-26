use std::fs::File;
use std::io::{self, BufRead, BufReader};

use image::ColorType;
use kurbo::{flatten, BezPath, PathEl, Point};

#[derive(Debug, Clone)]
struct Path {
    lines: Vec<Point>,
    fill: [u8; 3],
}

// Returns true if `p` is to the right of the line `l` from the viewer's perspective.
// Note that SVG's coordinate space has (0, 0) at the top-left corner, with Y going down.
fn point_is_to_the_right_of_line(p: &Point, l: (&Point, &Point)) -> bool {
    let (x0, y0, x1, y1) = (l.0.x, l.0.y, l.1.x, l.1.y);
    let (x, y) = (p.x, p.y);
    let cross = (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0);
    cross > 0.0
}

fn render(
    width: usize,
    height: usize,
    paths: &Vec<Vec<PathEdges>>,
    indices: &Vec<Path>,
) -> Vec<u8> {
    const NUM_CHANNELS: usize = 3;
    let mut result = vec![255_u8; width * height * NUM_CHANNELS];

    for y in 0..height {
        for x in 0..width {
            let p = Point::new(x as f64 + 0.5, y as f64 + 0.5);

            for path in paths[y].iter() {
                // Non-zero winding only. Support for even-odd can be added trivially, but we don't need it for our example.
                let mut winding = 0_i32;

                for l in path.edges.iter() {
                    let l0 = indices[path.fill_index].lines[l.line_index - 1];
                    let l1 = indices[path.fill_index].lines[l.line_index];

                    if (l0.y - l1.y).abs() < 1e-6 {
                        // Ignore horizontal lines, because we deal with scanlines going horizontally.
                        continue;
                    }

                    let in_bounds = p.y > l0.y.min(l1.y) && p.y <= l0.y.max(l1.y);
                    let (l0, l1, dir) = if l0.y > l1.y {
                        (l0, l1, 1_i32)
                    } else {
                        (l1, l0, -1_i32)
                    };
                    let to_the_right = point_is_to_the_right_of_line(&p, (&l0, &l1));

                    winding += ((in_bounds && to_the_right) as i32) * dir;
                }

                if winding != 0 {
                    let color = indices[path.fill_index].fill;

                    for i in 0..NUM_CHANNELS {
                        result[(y * width + x) * NUM_CHANNELS + i] = color[i];
                    }
                }
            }
        }
    }

    result
}

fn to_u8(channel: f64) -> u8 {
    (channel * 255.99).clamp(0.0, 255.0) as u8
}

fn main() -> io::Result<()> {
    let file = File::open("flattened.txt")?;
    let reader = BufReader::new(file);
    let mut lines = reader.lines();
    let mut segments = Vec::<Point>::new();
    let mut paths = Vec::<Path>::new();

    let header_line = lines
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::UnexpectedEof, "Missing header!"))??;
    let mut values = header_line.split_whitespace();
    let (width, height, count): (f64, f64, usize) = (
        values.next().unwrap_or("0").parse().unwrap_or(0.0),
        values.next().unwrap_or("0").parse().unwrap_or(0.0),
        values.next().unwrap_or("0").parse().unwrap_or(0),
    );

    for i in 0..count {
        let fill_line = lines.next().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::UnexpectedEof,
                format!("Missing fill color for path #{}!", i),
            )
        })??;
        let path_line = lines.next().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::UnexpectedEof,
                format!("Missing path string for path #{}!", i),
            )
        })??;

        let fill: Vec<f64> = fill_line
            .split_whitespace()
            .map(|s| {
                s.parse().expect(&format!(
                    "Failed to parse color channel for path #{}: {}",
                    i, s
                ))
            })
            .collect();

        let fill_color = (
            fill.get(0).copied().unwrap_or(0.0),
            fill.get(1).copied().unwrap_or(0.0),
            fill.get(2).copied().unwrap_or(0.0),
        );

        // Color processing is far from ideal, but we don't care for this example.
        let color = [
            to_u8(fill_color.0),
            to_u8(fill_color.1),
            to_u8(fill_color.2),
        ];

        let path = BezPath::from_svg(&path_line);

        if path.is_err() {
            eprintln!(
                "Failed to parse SVG path string #{}: {}",
                i,
                path.err().unwrap()
            );
            continue;
        }

        let path = path.unwrap();
        let mut start_point = Point::new(0.0, 0.0);
        let mut last_point = Point::new(0.0, 0.0);
        segments.clear();

        flatten(path, 0.1, |element| match element {
            PathEl::MoveTo(p) => {
                if last_point != start_point {
                    segments.push(start_point);
                }
                segments.push(p);
                start_point = p;
                last_point = p;
            }
            PathEl::LineTo(p) => {
                segments.push(p);
                last_point = p;
            }
            PathEl::ClosePath => {
                if last_point != start_point {
                    segments.push(start_point);
                    last_point = start_point;
                }
            }
            _ => unreachable!(),
        });

        paths.push(Path {
            lines: segments.clone(),
            fill: color,
        });
    }

    let sorted = sort_paths(&paths, height as usize);
    let buf = render(width as usize, height as usize, &sorted, &paths);
    image::save_buffer(
        "Output.png",
        buf.as_slice(),
        width as u32,
        height as u32,
        ColorType::Rgb8,
    )
    .expect("Failed to save output!");

    Ok(())
}

#[derive(Debug, Clone)]
struct Edge {
    line_index: usize,
}

#[derive(Debug, Clone)]
struct PathEdges {
    edges: Vec<Edge>,
    fill_index: usize,
}

// This is a very naive sort for paths. Basically filtering only the paths that are needed for each line.
// A grid approach would be better but this is enough for this test.
// Certainly a few orders of magnitude faster than not doing it at all.
fn sort_paths(paths: &Vec<Path>, height: usize) -> Vec<Vec<PathEdges>> {
    let mut result = vec![Vec::<PathEdges>::new(); height];

    for i in 0..paths.len() {
        for j in 1..paths[i].lines.len() {
            let l = [paths[i].lines[j - 1], paths[i].lines[j]];

            let start_y = l[0].y.min(l[1].y).floor() as usize;
            let end_y = (l[0].y.max(l[1].y).ceil() as usize + 1).min(height);

            for y in start_y..end_y {
                if result[y].is_empty() || (result[y].last().unwrap().fill_index != i) {
                    result[y].push(PathEdges {
                        edges: Vec::<Edge>::new(),
                        fill_index: i,
                    });
                }

                result[y]
                    .last_mut()
                    .unwrap()
                    .edges
                    .push(Edge { line_index: j });
            }
        }
    }

    result
}
