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


def parse_css_color(color: str) -> Tuple[float, float, float, float]:
    if not color:
        return (0.0, 0.0, 0.0, 0.0)

    color_str = color.strip().lower()

    if not color_str:
        return (0.0, 0.0, 0.0, 0.0)

    if color_str.startswith('#'):
        if all(c in '0123456789abcdef' for c in color_str[1:]):
            if len(color_str) == 7:
                r = int(color_str[1:3], 16) / 255.0
                g = int(color_str[3:5], 16) / 255.0
                b = int(color_str[5:7], 16) / 255.0
                return (r, g, b, 1.0)
            elif len(color_str) == 9:
                r = int(color_str[1:3], 16) / 255.0
                g = int(color_str[3:5], 16) / 255.0
                b = int(color_str[5:7], 16) / 255.0
                a = int(color_str[7:9], 16) / 255.0
                return (r, g, b, a)

    if color_str.startswith('rgb'):
        try:
            values = color_str[color_str.index(
                '(') + 1:color_str.rindex(')')].split('', '')
            values = [v.strip() for v in values]

            if len(values) == 3:
                r = int(values[0]) / 255.0
                g = int(values[1]) / 255.0
                b = int(values[2]) / 255.0
                return (r, g, b, 1.0)
            elif len(values) == 4:
                r = int(values[0]) / 255.0
                g = int(values[1]) / 255.0
                b = int(values[2]) / 255.0
                a = float(values[3])
                return (r, g, b, a)
            else:
                raise ValueError('Invalid color')
        except (ValueError, IndexError):
            print(f'Invalid rgb/rgba format: {color_str}')

    return (0.0, 0.0, 0.0, 0.0)


def read_paths_from_svg(svg_file_path: str) -> Tuple[Dimensions, List[Any], List[Tuple[float, float, float]]]:
    doc = minidom.parse(svg_file_path)
    root = doc.getElementsByTagName('svg')[0]
    view_box = root.attributes['viewBox'].value
    width = root.attributes['width'].value
    height = root.attributes['height'].value
    dimensions = Dimensions(width, height, view_box)
    paths = doc.getElementsByTagName('path')

    colors_data = []
    paths_data = []

    for path in paths:
        transform = IDENTITY
        color = (0.0, 0.0, 0.0)

        if 'transform' in path.attributes:
            transform = parse_svg_transform(path.attributes['transform'].value)

        if 'fill' in path.attributes:
            fill = parse_css_color(path.attributes['fill'].value)
            color = (fill[0] * fill[3], fill[1] * fill[3], fill[2] * fill[3])

        d = path.attributes['d'].value
        path_data = svg_to_skia_path(d, transform)
        paths_data.append(path_data)
        colors_data.append(color)

    doc.unlink()
    return (dimensions, paths_data, colors_data)


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


overlap_order = [164, 340, 581, 941, 746, 341, 1116, 1850, 1258, 1296, 1980, 395, 401, 393, 394, 402, 400, 403, 1695, 392, 398, 397, 406, 407, 404, 405, 396, 399, 110, 287, 73, 245, 527, 1814, 874, 1063, 487, 642, 1929, 1022, 829, 1197, 1173, 262, 85, 1249, 1679, 655, 502, 1946, 846, 1039, 1669, 374, 373, 372, 349, 350, 252, 351, 378, 348, 379, 356, 380, 377, 355, 352, 354, 357, 375, 382, 358, 385, 362, 361, 381, 360, 368, 359, 389, 364, 386, 388, 387, 366, 367, 369, 390, 365, 370, 391, 384, 363, 371, 383, 344, 310, 133, 353, 645, 84, 261, 1936, 31, 202, 343, 1290, 1331, 279, 342, 347, 1029, 120, 297, 836, 1659, 1372, 217, 44, 275, 98, 707, 550, 88, 1434, 265, 59, 654, 501, 232, 1945, 730, 1433, 443, 676, 519, 1367, 1733, 170, 5, 334, 1404, 1320, 158, 1364, 897, 1282, 1960, 1086, 151, 328, 537, 1038, 1668, 845, 1824, 925, 1284, 979, 1624, 784, 1436, 1677, 1789, 458, 1055, 866, 166, 612, 734, 668, 1, 142, 515, 319, 1903, 236, 1957, 63, 1417, 179, 12, 208, 130, 307, 505, 658, 36, 241, 884, 1681, 1073, 141, 68, 627, 1917, 318, 345, 473, 1949, 259, 732, 93, 270, 157, 103, 280, 799, 47, 929, 1639, 220, 994, 15, 182, 300, 574, 587, 1213, 1239, 123, 413, 859, 575, 1052, 10, 1856, 240, 56, 3, 229, 1844, 67, 156, 168, 175, 25, 194, 333, 1042, 597, 723, 849, 1172, 1672, 568, 303, 86, 814, 927, 126, 263, 1009, 122, 316, 583, 89, 139, 266, 299, 58, 117, 124, 145, 947, 301, 409, 935, 1690, 106, 230, 294, 322, 1110, 752, 1345, 231, 1852, 71, 283, 189, 216, 244, 631, 672, 184, 16, 52, 225, 421, 477, 559, 1207, 20, 57, 1830, 196, 547, 915, 1104, 1710, 27, 449, 1384, 1921, 43, 46, 148, 172, 652, 1739, 136, 219, 313, 325, 482, 499, 558, 636, 1324, 143, 249, 274, 302, 1263, 1300, 1943, 7, 125, 715, 1410, 90, 320, 510, 663, 1366, 1772, 1926, 100, 330, 615, 1283, 72, 76, 83, 97, 153, 155, 635, 748, 1165, 1279, 1961, 185, 267, 332, 520, 677, 1137, 1336, 1361, 60, 75, 152, 178, 192, 235, 277, 329, 424, 461, 673, 943, 1316, 17, 62, 146, 218, 260, 540, 1400, 1430, 1435, 23, 209, 248, 592, 863, 1186, 1952, 37, 45, 150, 233, 237, 251, 296, 697, 1095, 64, 323, 411, 675, 1905, 6, 78, 119, 247, 327, 418, 436, 956, 1013, 1176, 1713, 1748, 1914, 1925, 26, 135, 149, 171, 250, 624, 906, 1266, 1725, 1861, 107, 113, 195, 312, 470, 481, 585, 761, 818, 894, 1094, 1348, 1601, 1826, 1854, 77, 108, 116, 154, 284, 289, 291, 503, 700, 727, 823, 985, 1056, 1303, 1441, 49, 112, 114, 213, 238, 285, 290, 331, 573, 726, 790, 843, 1036, 1341, 1630, 1947, 35, 53, 65, 138, 173, 207, 226, 292, 656, 1083, 1666, 40, 96, 115, 293, 315, 326, 659, 696, 854, 1047, 1315, 1379, 1387, 1843, 8, 105, 212, 222, 272, 506, 543, 867, 905, 1278, 1294, 1360, 95, 254, 713, 764, 864, 887, 1018, 1399, 1642, 1950, 22, 32, 87, 91, 127, 271, 273, 276, 282, 304, 997, 21, 51, 94, 134, 144, 253, 264, 311, 457, 471, 523, 534, 541, 556, 562, 639, 698, 802, 13, 39, 79, 109, 187, 203, 214, 224, 306, 308, 611, 626, 959, 1076, 1228, 1604, 41, 50, 54, 99, 324, 472, 539, 680, 757, 811, 1339, 1438, 1821, 1833, 2, 74, 111, 131, 132, 167, 191, 206, 211, 223, 234, 268, 286, 288, 295, 431, 438, 466, 485, 920, 1910, 1915, 1916, 1962, 29, 33, 80, 82, 104, 118, 129, 137, 147, 193, 201, 227, 281, 309, 620, 776, 945, 1017, 1616, 1689, 9, 48, 180, 198, 205, 255, 426, 750, 952, 971, 1006, 1040, 1079, 1109, 1376, 1720, 18, 19, 24, 61, 66, 165, 174, 188, 221, 239, 243, 246, 305, 314, 625, 710, 720, 731, 822, 1305, 1310, 1389, 1670, 1727, 0, 14, 30, 34, 69, 128, 199, 200, 242, 256, 257, 415, 565, 847, 903, 922, 1043, 1317, 1355, 1394, 1416, 1715, 55, 81, 176, 177, 181, 186, 210, 215, 460, 542, 553, 560, 572, 850, 890, 1273, 1350, 1902, 1968, 11, 38, 121, 162, 183, 228, 269, 298, 338, 459, 507, 514, 570, 717, 724, 798, 921, 1070, 1092, 1098, 1268, 1288, 1685, 1750, 1777, 1971, 42, 70, 101, 204, 476, 490, 500, 613, 640, 660, 728, 778, 886, 1075, 1858, 1931, 1944, 1951, 1964, 1965, 434, 486, 563, 569, 589, 641, 653, 667, 674, 699, 718, 722, 735, 826, 888, 1340, 1371, 1673, 1709, 1723, 1904, 1920, 1959, 1974, 278, 450, 478, 517, 567, 588, 604, 632, 644, 737, 813, 881, 1007, 1021, 1077, 1220, 1246, 1269, 1299, 1440, 1776, 1840, 1956, 1969, 92, 414, 427, 489, 536, 566, 571, 670, 721, 870, 973, 1008, 1101, 1219, 1244, 1292, 1313, 1344, 1358, 1377, 1390, 1397, 1439, 1647, 1716, 1823, 1864, 1918, 1922, 448, 474, 492, 504, 524, 552, 628, 630, 633, 666, 771, 966, 993, 1002, 1059, 1078, 1306, 1327, 1328, 1329, 1351, 1383, 1423, 1611, 1638, 1740, 1779, 1788, 1842, 1857, 1935, 1973, 102, 346, 437, 512, 531, 608, 617, 621, 643, 709, 900, 912, 1026, 1044, 1293, 1370, 1378, 1421, 1432, 1606, 1818, 1859, 321, 467, 491, 529, 532, 533, 590, 607, 682, 712, 766, 801, 807, 812, 827, 907, 926, 996, 1089, 1096, 1181, 1190, 1210, 1236, 1276, 1287, 1420, 1641, 1656, 1674, 1726, 1811, 1898, 1907, 1934, 1963, 444, 454, 463, 513, 516, 522, 530, 555, 561, 598, 602, 701, 754, 760, 833, 851, 858, 889, 949, 961, 1262, 1286, 1289, 1291, 1614, 1618, 1651, 1682, 1688, 1721, 1816, 1820, 1841, 1911, 1948, 416, 453, 455, 479, 493, 525, 544, 609, 646, 665, 708, 729, 733, 739, 767, 817, 828, 861, 865, 910, 914, 916, 917, 923, 995, 1051, 1105, 1106, 1108, 1321, 1330, 1332, 1338, 1356, 1369, 1375, 1409, 1422, 1600, 1640, 1667, 1686, 1749, 1778, 1782, 1832, 1838, 1893, 1897, 1899, 1923, 1954, 1955, 1972, 429, 432, 464, 494, 497, 511, 521, 526, 551, 619, 647, 657, 664, 669, 705, 753, 774, 791, 815, 819, 835, 919, 962, 969, 1010, 1014, 1015,
                 1161, 1413, 1425, 1607, 1658, 1781, 1796, 1819, 1889, 1937, 1958, 446, 452, 465, 546, 564, 596, 622, 629, 634, 777, 800, 844, 871, 883, 913, 918, 930, 948, 955, 986, 1028, 1037, 1066, 1067, 1088, 1099, 1103, 1140, 1271, 1312, 1325, 1353, 1405, 1412, 1414, 1734, 1775, 1801, 1803, 1809, 1815, 1873, 1938, 408, 410, 422, 442, 447, 468, 488, 495, 508, 528, 535, 549, 554, 584, 593, 595, 601, 618, 638, 648, 661, 702, 703, 711, 719, 738, 808, 820, 831, 832, 834, 932, 984, 1012, 1025, 1054, 1061, 1177, 1274, 1314, 1363, 1373, 1374, 1393, 1395, 1418, 1426, 1437, 1617, 1631, 1648, 1655, 1687, 1724, 1742, 1812, 1813, 1817, 1909, 1912, 1941, 1953, 1966, 417, 419, 425, 430, 433, 445, 462, 475, 484, 548, 582, 591, 594, 605, 606, 610, 637, 649, 789, 804, 872, 876, 877, 899, 950, 1003, 1024, 1027, 1030, 1050, 1068, 1069, 1072, 1082, 1097, 1160, 1174, 1264, 1275, 1277, 1281, 1304, 1318, 1326, 1365, 1396, 1398, 1654, 1657, 1684, 1714, 1719, 1741, 1743, 1808, 1822, 1853, 1887, 1891, 1908, 1924, 1927, 420, 428, 435, 440, 441, 469, 480, 483, 518, 545, 599, 600, 616, 650, 695, 736, 785, 794, 809, 837, 848, 856, 879, 880, 895, 902, 924, 928, 972, 989, 990, 1049, 1060, 1065, 1107, 1150, 1270, 1301, 1308, 1309, 1333, 1354, 1381, 1388, 1391, 1392, 1402, 1408, 1411, 1612, 1625, 1629, 1634, 1635, 1661, 1711, 1712, 1718, 1769, 1783, 1851, 1862, 1869, 1884, 1885, 1900, 1901, 1906, 1919, 1928, 423, 451, 538, 579, 623, 662, 772, 783, 805, 857, 860, 878, 892, 896, 898, 908, 911, 964, 980, 982, 988, 1001, 1004, 1048, 1053, 1058, 1087, 1102, 1156, 1204, 1265, 1267, 1272, 1280, 1285, 1297, 1298, 1307, 1311, 1319, 1322, 1323, 1334, 1346, 1347, 1349, 1357, 1359, 1368, 1382, 1385, 1386, 1401, 1613, 1623, 1633, 1646, 1660, 1671, 1707, 1708, 1717, 1722, 1729, 1730, 1848, 1860, 1886, 1892, 1970, 456, 496, 671, 755, 773, 788, 793, 795, 806, 838, 839, 868, 869, 873, 882, 891, 934, 957, 967, 968, 976, 978, 991, 1005, 1023, 1031, 1034, 1041, 1045, 1057, 1062, 1064, 1080, 1091, 1151, 1193, 1224, 1235, 1247, 1302, 1342, 1352, 1362, 1406, 1407, 1427, 1605, 1621, 1636, 1643, 1653, 1662, 1678, 1731, 1913, 1940, 509, 747, 749, 756, 759, 762, 770, 775, 781, 786, 787, 796, 810, 816, 841, 852, 853, 855, 893, 939, 944, 960, 965, 977, 983, 1011, 1016, 1071, 1074, 1085, 1090, 1208, 1216, 1221, 1238, 1252, 1255, 1260, 1343, 1428, 1609, 1620, 1627, 1628, 1649, 1675, 1683, 1797, 1890, 758, 763, 765, 769, 780, 792, 797, 803, 830, 875, 885, 901, 951, 970, 975, 998, 1000, 1033, 1084, 1100, 1114, 1127, 1136, 1145, 1155, 1178, 1415, 1596, 1598, 1615, 1644, 1650, 1664, 1784, 768, 782, 821, 824, 825, 931, 942, 953, 958, 963, 992, 999, 1019, 1020, 1081, 1130, 1138, 1139, 1171, 1206, 1227, 1257, 1261, 1602, 1608, 1610, 1637, 1645, 1663, 1676, 1693, 1771, 840, 862, 933, 954, 981, 1032, 1046, 1153, 1158, 1166, 1188, 1226, 1256, 1599, 1603, 1622, 1626, 1767, 1142, 1167, 1184, 1192, 1194, 1201, 1211, 1215, 1222, 1225, 1234, 1243, 1245, 1250, 1253, 987, 1168, 1175, 1182, 1195, 1198, 1199, 1200, 1203, 1254, 1793, 376, 1129, 1133, 1134, 1143, 1144, 1147, 1169, 1191, 1196, 1202, 1214, 1232, 1242, 1632, 1736, 1128, 1159, 1179, 1180, 1209, 1217, 1223, 1237, 1241, 1248, 1751, 1131, 1141, 1148, 1170, 1205, 1212, 1240, 1680, 1791, 190, 1419, 1967, 1123, 1652, 258, 336, 576, 1825, 159, 335, 684, 741, 936, 1122, 1125, 1164, 1534, 1595, 1837, 1978, 4, 28, 140, 160, 161, 163, 169, 197, 317, 337, 339, 412, 439, 498, 557, 577, 578, 580, 586, 603, 614, 651, 678, 679, 681, 683, 685, 686, 687, 688, 689, 690, 691, 692, 693, 694, 704, 706, 714, 716, 725, 740, 742, 743, 744, 745, 751, 779, 842, 904, 909, 937, 938, 940, 946, 974, 1035, 1093, 1111, 1112, 1113, 1115, 1117, 1118, 1119, 1120, 1121, 1124, 1126, 1132, 1135, 1146, 1149, 1152, 1154, 1157, 1162, 1163, 1183, 1185, 1187, 1189, 1218, 1229, 1230, 1231, 1233, 1251, 1259, 1295, 1335, 1337, 1380, 1403, 1424, 1429, 1431, 1442, 1443, 1444, 1445, 1446, 1447, 1448, 1449, 1450, 1451, 1452, 1453, 1454, 1455, 1456, 1457, 1458, 1459, 1460, 1461, 1462, 1463, 1464, 1465, 1466, 1467, 1468, 1469, 1470, 1471, 1472, 1473, 1474, 1475, 1476, 1477, 1478, 1479, 1480, 1481, 1482, 1483, 1484, 1485, 1486, 1487, 1488, 1489, 1490, 1491, 1492, 1493, 1494, 1495, 1496, 1497, 1498, 1499, 1500, 1501, 1502, 1503, 1504, 1505, 1506, 1507, 1508, 1509, 1510, 1511, 1512, 1513, 1514, 1515, 1516, 1517, 1518, 1519, 1520, 1521, 1522, 1523, 1524, 1525, 1526, 1527, 1528, 1529, 1530, 1531, 1532, 1533, 1535, 1536, 1537, 1538, 1539, 1540, 1541, 1542, 1543, 1544, 1545, 1546, 1547, 1548, 1549, 1550, 1551, 1552, 1553, 1554, 1555, 1556, 1557, 1558, 1559, 1560, 1561, 1562, 1563, 1564, 1565, 1566, 1567, 1568, 1569, 1570, 1571, 1572, 1573, 1574, 1575, 1576, 1577, 1578, 1579, 1580, 1581, 1582, 1583, 1584, 1585, 1586, 1587, 1588, 1589, 1590, 1591, 1592, 1593, 1594, 1597, 1619, 1665, 1691, 1692, 1694, 1696, 1697, 1698, 1699, 1700, 1701, 1702, 1703, 1704, 1705, 1706, 1728, 1732, 1735, 1737, 1738, 1744, 1745, 1746, 1747, 1752, 1753, 1754, 1755, 1756, 1757, 1758, 1759, 1760, 1761, 1762, 1763, 1764, 1765, 1766, 1768, 1770, 1773, 1774, 1780, 1785, 1786, 1787, 1790, 1792, 1794, 1795, 1798, 1799, 1800, 1802, 1804, 1805, 1806, 1807, 1810, 1827, 1828, 1829, 1831, 1834, 1835, 1836, 1839, 1845, 1846, 1847, 1849, 1855, 1863, 1865, 1866, 1867, 1868, 1870, 1871, 1872, 1874, 1875, 1876, 1877, 1878, 1879, 1880, 1881, 1882, 1883, 1888, 1894, 1895, 1896, 1930, 1932, 1933, 1939, 1942, 1975, 1976, 1977, 1979]
