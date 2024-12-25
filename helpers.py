import math
import re

import xml.etree.ElementTree as ET
from xml.dom import minidom
from dataclasses import dataclass
from typing import List, Tuple, Any

import pathops
from svg.path import parse_path
from svg.path.path import Move, Close, Line, CubicBezier, QuadraticBezier, Arc


"""
Row-major 3x3 affine transform matrix.
"""


@dataclass
class Transform:
    matrix: List[List[float]]

    def apply_to(self, point):
        m = self.matrix
        return (
            m[0][0] * point[0] + m[0][1] * point[1] + m[0][2],
            m[1][0] * point[0] + m[1][1] * point[1] + m[1][2]
        )

    @staticmethod
    def from_abcdef(a: float, b: float, c: float, d: float, e: float, f: float) -> 'Transform':
        return Transform([
            [a, c, e],
            [b, d, f],
            [0.0, 0.0, 1.0],
        ])

    @staticmethod
    def from_translation(x: float, y: float) -> 'Transform':
        return Transform([
            [1.0, 0.0, x],
            [0.0, 1.0, y],
            [0.0, 0.0, 1.0]
        ])

    @staticmethod
    def from_scaling(sx: float, sy: float) -> 'Transform':
        return Transform([
            [sx, 0.0, 0.0],
            [0.0, sy, 0.0],
            [0.0, 0.0, 1.0]
        ])

    @staticmethod
    def from_rotation(angle_radians: float, pivot: Tuple[float, float]) -> 'Transform':
        cos_theta = math.cos(angle_radians)
        sin_theta = math.sin(angle_radians)

        cx, cy = pivot

        return Transform([
            [cos_theta, -sin_theta, cx * (1.0 - cos_theta) + cy * sin_theta],
            [sin_theta, cos_theta, cy * (1.0 - cos_theta) - cx * sin_theta],
            [0.0, 0.0, 1.0]
        ])

    @staticmethod
    def from_skewing(ax_radians: float, ay_radians: float) -> 'Transform':
        tangent_x, tangent_y = math.tan(ax_radians), math.tan(ay_radians)

        return AffineTransform([
            [1.0, tangent_x, 0.0],
            [tangent_y, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])

    def concat(self, other: 'Transform') -> 'Transform':
        m1, m2 = self.matrix, other.matrix

        result = [[0.0 for _ in range(3)] for _ in range(3)]

        for i in range(3):
            for j in range(3):
                for k in range(3):
                    result[i][j] += m1[i][k] * m2[k][j]

        return Transform(result)


IDENTITY = Transform([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

"""
Parses an SVG transform. Examples:

 - 'scale(1.1, 1.1)'
 - 'translate(10 10)'
 - 'rotate(40, 100, 100)'
 - 'matrix(1 0 0 0 1 0)'

It can also parse multiple transforms. They will be multiplied from right to left.
"""


def parse_svg_transform(input: str) -> Transform:
    pattern = r'(\w+)\(([^)]+)\)'
    matches = re.findall(pattern, input)
    result = IDENTITY

    for func, args in reversed(matches):
        arg = filter(None, re.split(r'[, ]+', args))
        arg = [float(f) for f in arg]

        match func:
            case 'translate':
                tx, ty = arg if len(arg) == 2 else (arg[0], 0)
                result = result.concat(Transform.from_translation(tx, ty))
            case 'scale':
                sx, sy = arg if len(arg) == 2 else (arg[0], arg[0])
                result = result.concat(Transform.from_scaling(sx, sy))
            case 'rotate':
                angle = math.radians(arg[0])
                cx, cy = 0.0, 0.0

                if len(arg) == 2:
                    cx = arg[1]
                elif len(arg) > 2:
                    cx = arg[1]
                    cy = arg[2]

                result = result.concat(
                    Transform.from_rotation(angle, (cx, cy)))
            case 'skewX':
                angle = math.radians(arg[0])
                result = result.concat(Transform.from_skewing(angle, 0.0))
            case 'skewY':
                angle = math.radians(arg[0])
                result = result.concat(Transform.from_skewing(0.0, angle))
            case 'matrix':
                if len(arg) == 6:
                    a, b, c, d, e, f = arg
                    result = result.concat(Transform([
                        [a, c, e],
                        [b, d, f],
                        [0.0, 0.0, 1.0]
                    ]))

    return result


"""
This function converts an elliptical arc (in SVG notation) to a series of cubic
Bézier curves.

For explanation of implementation details see:
- https://pomax.github.io/bezierinfo/#circles_cubic
- https://minus-ze.ro/posts/flattening-bezier-curves-and-arcs/
- https://github.com/colinmeinke/svg-arc-to-cubic-bezier/blob/master/src/index.js
"""


def convert_arc_to_cubics(start, end, rx, ry, rotation, large_arc, sweep):
    def unit_vector_angle(u, v):
        sign = -1 if (u[0] * v[1] - u[1] * v[0]) < 0 else 1
        dot = u[0] * v[0] + u[1] * v[1]
        dot_clamped = min(max(dot, -1.0), 1.0)

        return sign * math.acos(dot_clamped)

    def parameterize(p1, p2, large_arc, sweep, rx, ry, sin_theta, cos_theta, x1p, y1p):
        rx_sq = rx * rx
        ry_sq = ry * ry
        x1p_sq = x1p * x1p
        y1p_sq = y1p * y1p

        radicant = max(rx_sq * ry_sq - rx_sq * y1p_sq - ry_sq * x1p_sq, 0)
        radicant /= rx_sq * y1p_sq + ry_sq * x1p_sq
        radicant = math.sqrt(radicant) * (-1 if large_arc == sweep else 1)

        cxp = radicant * (rx / ry) * y1p
        cyp = radicant * (-ry / rx) * x1p

        cx = cos_theta * cxp - sin_theta * cyp + (p1[0] + p2[0]) / 2
        cy = sin_theta * cxp + cos_theta * cyp + (p1[1] + p2[1]) / 2

        v1x = (x1p - cxp) / rx
        v1y = (y1p - cyp) / ry
        v2x = (-x1p - cxp) / rx
        v2y = (-y1p - cyp) / ry

        theta1 = unit_vector_angle((1, 0), (v1x, v1y))
        delta_theta = unit_vector_angle((v1x, v1y), (v2x, v2y))

        if sweep == 0 and delta_theta > 0:
            delta_theta -= 2 * math.pi
        if sweep == 1 and delta_theta < 0:
            delta_theta += 2 * math.pi

        return {'center': (cx, cy), 'theta': theta1, 'delta_theta': delta_theta}

    def approximate_unit_arc(theta, delta_theta):
        alpha = (4 / 3) * math.tan(delta_theta / 4)

        x1 = math.cos(theta)
        y1 = math.sin(theta)
        x2 = math.cos(theta + delta_theta)
        y2 = math.sin(theta + delta_theta)

        return [(x1, y1), (x1 - y1 * alpha, y1 + x1 * alpha), (x2 + y2 * alpha, y2 - x2 * alpha), (x2, y2)]

    def apply_arc_point_transform(p, center_p, rx, ry, sin_theta, cos_theta):
        x = p[0]
        y = p[1]

        x *= rx
        y *= ry

        xp = cos_theta * x - sin_theta * y
        yp = sin_theta * x + cos_theta * y

        return (xp + center_p['center'][0], yp + center_p['center'][1])

    sin_theta = math.sin(math.radians(rotation))
    cos_theta = math.cos(math.radians(rotation))

    x1p = (cos_theta * (start[0] - end[0])) / 2 + \
        (sin_theta * (start[1] - end[1])) / 2
    y1p = (-sin_theta * (start[0] - end[0])) / 2 + \
        (cos_theta * (start[1] - end[1])) / 2

    if x1p == 0 and y1p == 0:
        return []
    if rx == 0 or ry == 0:
        return []

    rx, ry = abs(rx), abs(ry)
    l = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry)

    if l > 1:
        rx *= math.sqrt(l)
        ry *= math.sqrt(l)

    center_p = parameterize(start, end, large_arc, sweep,
                            rx, ry, sin_theta, cos_theta, x1p, y1p)
    result = []
    theta = center_p['theta']
    delta_theta = center_p['delta_theta']
    num_segments = max(math.ceil(abs(delta_theta) / (math.pi / 2)), 1)
    delta_theta /= num_segments

    for i in range(num_segments):
        curve = approximate_unit_arc(theta, delta_theta)

        result.append([
            apply_arc_point_transform(
                curve[0], center_p, rx, ry, sin_theta, cos_theta),
            apply_arc_point_transform(
                curve[1], center_p, rx, ry, sin_theta, cos_theta),
            apply_arc_point_transform(
                curve[2], center_p, rx, ry, sin_theta, cos_theta),
            apply_arc_point_transform(
                curve[3], center_p, rx, ry, sin_theta, cos_theta),
        ])

        theta += delta_theta

    return result


def svg_to_skia_path(svg_path: str, ctm: Transform) -> Any:
    path = parse_path(svg_path)
    skia_path = pathops.Path()
    skia_path.fillType = pathops.FillType.WINDING

    def p(x, y):
        return ctm.apply_to((x, y))

    for segment in path:
        if isinstance(segment, Move):
            skia_path.moveTo(*p(segment.start.real, segment.start.imag))
        elif isinstance(segment, Line):
            skia_path.lineTo(*p(segment.end.real, segment.end.imag))
        elif isinstance(segment, QuadraticBezier):
            skia_path.quadTo(
                *p(segment.control.real, segment.control.imag),
                *p(segment.end.real, segment.end.imag)
            )
        elif isinstance(segment, CubicBezier):
            skia_path.cubicTo(
                *p(segment.control1.real, segment.control1.imag),
                *p(segment.control2.real, segment.control2.imag),
                *p(segment.end.real, segment.end.imag)
            )
        elif isinstance(segment, Arc):
            start_point = segment.start.real, segment.start.imag
            end_point = segment.end.real, segment.end.imag
            rx, ry = segment.radius.real, segment.radius.imag
            x_axis_rotation = math.degrees(segment.rotation)
            large_arc = int(segment.arc)
            sweep = int(segment.sweep)

            # Convert arcs to a series of cubics because otherwise Skia will encode
            # conics, and apparently it doesn't support conics in union.
            cubics = convert_arc_to_cubics(
                start_point, end_point, rx, ry, x_axis_rotation, large_arc, sweep)
            for cubic in cubics:
                skia_path.cubicTo(
                    *p(cubic[1][0], cubic[1][1]),
                    *p(cubic[2][0], cubic[2][1]),
                    *p(cubic[3][0], cubic[3][1])
                )
        elif isinstance(segment, Close):
            skia_path.close()

    return skia_path


def skia_to_svg_path(skia_path: Any) -> str:
    if skia_path is None:
        return ''

    svg_path = ''
    for verb, points in skia_path:
        match verb:
            case pathops.PathVerb.MOVE:
                svg_path += f'M{points[0][0]},{points[0][1]} '
            case pathops.PathVerb.LINE:
                svg_path += f'L{points[0][0]},{points[0][1]} '
            case pathops.PathVerb.QUAD:
                svg_path += f'Q{points[0][0]},{points[0][1]} {points[1][0]},{points[1][1]} '
            case pathops.PathVerb.CUBIC:
                svg_path += f'C{points[0][0]},{points[0][1]} {points[1][0]},{points[1][1]} {points[2][0]},{points[2][1]} '
            case pathops.PathVerb.CLOSE:
                svg_path += f'Z '
    return svg_path.strip()


@dataclass
class Dimensions:
    width: str
    height: str
    view_box: str


def read_paths_from_svg(svg_file_path: str) -> Tuple[Dimensions, List[Any]]:
    doc = minidom.parse(svg_file_path)
    root = doc.getElementsByTagName('svg')[0]
    view_box = root.attributes['viewBox'].value
    width = root.attributes['width'].value
    height = root.attributes['height'].value
    dimensions = Dimensions(width, height, view_box)
    paths = doc.getElementsByTagName('path')

    paths_data = []
    for path in paths:
        transform = IDENTITY

        if 'transform' in path.attributes:
            transform = parse_svg_transform(path.attributes['transform'].value)

        d = path.attributes['d'].value
        path_data = svg_to_skia_path(d, transform)
        paths_data.append(path_data)

    doc.unlink()
    return (dimensions, paths_data)


def write_output(result: Any, dimensions: Dimensions, stroke_width: float):
    with open('Contour.svg', 'w') as f:
        root = ET.Element('svg')
        root.set('xmlns', 'http://www.w3.org/2000/svg')
        root.set('xmlns:xlink', 'http://www.w3.org/1999/xlink')
        root.set('width', dimensions.width)
        root.set('height', dimensions.height)
        root.set('viewBox', dimensions.view_box)

        path = ET.SubElement(root, 'path')
        path.set('fill', 'transparent')
        path.set('stroke', 'magenta')
        path.set('stroke-width', f'{stroke_width}')
        path.set('stroke-linecap', 'round')
        path.set('stroke-linejoin', 'round')
        path.set('d', skia_to_svg_path(result))

        svg = ET.ElementTree(root)
        svg.write('Contour.svg')
