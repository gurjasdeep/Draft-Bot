import xml.etree.ElementTree as ET
import re
import math


# ------------------------------------------------------------
# Basic geometry
# ------------------------------------------------------------

class Point:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def __repr__(self):
        return f"({self.x:.2f}, {self.y:.2f})"


class Path:
    def __init__(self):
        self.points = []

    def add_point(self, x, y):
        self.points.append(Point(x, y))

    def __repr__(self):
        return f"Path({len(self.points)} points)"


# ------------------------------------------------------------
# SVG path parser
# ------------------------------------------------------------

# ------------------------------------------------------------
# SVG path parser
# ------------------------------------------------------------

COMMANDS = "MmLlHhVvCcSsQqTtAaZz"


def tokenize_path(d):
    """
    Convert SVG path string into tokens.

    Example:
        M 10 20 L 50 20 L 50 50

    becomes:
        ['M', '10', '20', 'L', '50', '20', 'L', '50', '50']
    """

    pattern = (
        r"[MmLlHhVvCcSsQqTtAaZz]"
        r"|[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?"
    )

    return re.findall(pattern, d)


def is_command(token):
    return token in COMMANDS

# ------------------------------------------------------------
# Curve interpolation
# ------------------------------------------------------------

def lerp(a, b, t):
    return a + (b - a) * t


def cubic_bezier(p0, p1, p2, p3, t):
    """
    Cubic Bezier curve.
    """

    x = (
        (1 - t) ** 3 * p0.x
        + 3 * (1 - t) ** 2 * t * p1.x
        + 3 * (1 - t) * t ** 2 * p2.x
        + t ** 3 * p3.x
    )

    y = (
        (1 - t) ** 3 * p0.y
        + 3 * (1 - t) ** 2 * t * p1.y
        + 3 * (1 - t) * t ** 2 * p2.y
        + t ** 3 * p3.y
    )

    return Point(x, y)


def quadratic_bezier(p0, p1, p2, t):
    """
    Quadratic Bezier curve.
    """

    x = (
        (1 - t) ** 2 * p0.x
        + 2 * (1 - t) * t * p1.x
        + t ** 2 * p2.x
    )

    y = (
        (1 - t) ** 2 * p0.y
        + 2 * (1 - t) * t * p1.y
        + t ** 2 * p2.y
    )

    return Point(x, y)

# ------------------------------------------------------------
# SVG Arc → points
# ------------------------------------------------------------

def arc_to_points(
    p0,
    rx,
    ry,
    rotation,
    large_arc,
    sweep,
    p1,
    steps=40
):
    """
    Convert an SVG elliptical arc into points.

    p0:
        Starting point

    rx, ry:
        Ellipse radii

    rotation:
        Rotation of ellipse in degrees

    large_arc:
        0 = smaller arc
        1 = larger arc

    sweep:
        0 = counter-clockwise
        1 = clockwise

    p1:
        Ending point
    """

    rx = abs(rx)
    ry = abs(ry)

    if rx == 0 or ry == 0:
        return [p1]

    if (
        abs(p0.x - p1.x) < 1e-9
        and
        abs(p0.y - p1.y) < 1e-9
    ):
        return []

    phi = math.radians(rotation % 360)

    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)

    # --------------------------------------------------------
    # Step 1:
    # Transform midpoint into ellipse coordinate system
    # --------------------------------------------------------

    dx = (p0.x - p1.x) / 2
    dy = (p0.y - p1.y) / 2

    x_prime = (
        cos_phi * dx
        + sin_phi * dy
    )

    y_prime = (
        -sin_phi * dx
        + cos_phi * dy
    )

    # --------------------------------------------------------
    # Step 2:
    # Correct radii if they are too small
    # --------------------------------------------------------

    radius_check = (
        (x_prime ** 2) / (rx ** 2)
        + (y_prime ** 2) / (ry ** 2)
    )

    if radius_check > 1:

        scale = math.sqrt(radius_check)

        rx *= scale
        ry *= scale

    # --------------------------------------------------------
    # Step 3:
    # Calculate ellipse center in transformed coordinates
    # --------------------------------------------------------

    numerator = (
        (rx ** 2) * (ry ** 2)
        - (rx ** 2) * (y_prime ** 2)
        - (ry ** 2) * (x_prime ** 2)
    )

    denominator = (
        (rx ** 2) * (y_prime ** 2)
        + (ry ** 2) * (x_prime ** 2)
    )

    if denominator == 0:
        return [p1]

    factor = math.sqrt(
        max(0, numerator / denominator)
    )

    if large_arc == sweep:
        factor = -factor

    cx_prime = (
        factor
        * (rx * y_prime / ry)
    )

    cy_prime = (
        factor
        * (-ry * x_prime / rx)
    )

    # --------------------------------------------------------
    # Step 4:
    # Convert center back to original coordinate system
    # --------------------------------------------------------

    cx = (
        cos_phi * cx_prime
        - sin_phi * cy_prime
        + (p0.x + p1.x) / 2
    )

    cy = (
        sin_phi * cx_prime
        + cos_phi * cy_prime
        + (p0.y + p1.y) / 2
    )

    # --------------------------------------------------------
    # Step 5:
    # Calculate start angle
    # --------------------------------------------------------

    ux = (
        x_prime - cx_prime
    ) / rx

    uy = (
        y_prime - cy_prime
    ) / ry

    vx = (
        -x_prime - cx_prime
    ) / rx

    vy = (
        -y_prime - cy_prime
    ) / ry

    theta1 = math.atan2(
        uy,
        ux
    )

    # --------------------------------------------------------
    # Step 6:
    # Calculate angular sweep
    # --------------------------------------------------------

    dot = ux * vx + uy * vy

    dot = max(-1, min(1, dot))

    delta_theta = math.acos(dot)

    cross = ux * vy - uy * vx

    if cross < 0:
        delta_theta = -delta_theta

    if not sweep and delta_theta > 0:
        delta_theta -= 2 * math.pi

    elif sweep and delta_theta < 0:
        delta_theta += 2 * math.pi

    # --------------------------------------------------------
    # Step 7:
    # Generate points
    # --------------------------------------------------------

    arc_steps = max(
        2,
        math.ceil(
            abs(delta_theta)
            / (2 * math.pi)
            * steps
        )
    )

    points = []

    for i in range(1, arc_steps + 1):

        theta = (
            theta1
            + delta_theta * i / arc_steps
        )

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        x = (
            cx
            + rx * cos_phi * cos_theta
            - ry * sin_phi * sin_theta
        )

        y = (
            cy
            + rx * sin_phi * cos_theta
            + ry * cos_phi * sin_theta
        )

        points.append(
            Point(x, y)
        )

    return points

# ------------------------------------------------------------
# Parse one SVG path
# ------------------------------------------------------------
# ------------------------------------------------------------
# Parse one SVG path
# ------------------------------------------------------------

def parse_path(d, curve_steps=20, arc_steps=40):

    tokens = tokenize_path(d)

    paths = []

    i = 0
    command = None

    current = Point(0, 0)
    start = Point(0, 0)

    current_path = None

    # Previous control point.
    # Used by S and T.
    last_control = None

    # Last command type.
    # Needed to determine whether a control point
    # should actually be reflected.
    last_command = None

    while i < len(tokens):

        if is_command(tokens[i]):

            command = tokens[i]
            i += 1

        if command is None:
            raise ValueError("Invalid SVG path")

        # ====================================================
        # M - MOVETO
        # ====================================================

        if command in ("M", "m"):

            first = True    

            while (
                i < len(tokens)
                and not is_command(tokens[i])
            ):

                x = float(tokens[i])
                y = float(tokens[i + 1])

                i += 2

                if command == "m":
                
                    x += current.x
                    y += current.y

                current = Point(x, y)

                if first:

                    # New independent drawing path
                    current_path = Path()

                    paths.append(current_path)

                    start = Point(
                        x,
                        y
                    )

                    current_path.add_point(
                        x,
                        y
                    )

                    first = False

                else:

                    # Additional coordinate pairs after M
                    # are treated as L commands.

                    current_path.add_point(
                        x,
                        y
                    )

                last_control = None
                last_command = "M"

            command = (
                "l"
                if command == "m"
                else "L"
            )

        # ====================================================
        # L - LINETO
        # ====================================================

        elif command in ("L", "l"):

            while (
                i < len(tokens)
                and not is_command(tokens[i])
            ):

                x = float(tokens[i])
                y = float(tokens[i + 1])

                i += 2

                if command == "l":

                    x += current.x
                    y += current.y

                current = Point(x, y)

                current_path.add_point(
                    x,
                    y
                )

                last_control = None
                last_command = "L"

        # ====================================================
        # H - HORIZONTAL LINETO
        # ====================================================

        elif command in ("H", "h"):

            while (
                i < len(tokens)
                and not is_command(tokens[i])
            ):

                x = float(tokens[i])

                i += 1

                if command == "h":
                    x += current.x

                current = Point(
                    x,
                    current.y
                )

                current_path.add_point(
                    current.x,
                    current.y
                )

                last_control = None
                last_command = "H"

        # ====================================================
        # V - VERTICAL LINETO
        # ====================================================

        elif command in ("V", "v"):

            while (
                i < len(tokens)
                and not is_command(tokens[i])
            ):

                y = float(tokens[i])

                i += 1

                if command == "v":
                    y += current.y

                current = Point(
                    current.x,
                    y
                )

                current_path.add_point(
                    current.x,
                    current.y
                )

                last_control = None
                last_command = "V"

        # ====================================================
        # C - CUBIC BEZIER
        # ====================================================

        elif command in ("C", "c"):

            while (
                i < len(tokens)
                and not is_command(tokens[i])
            ):

                x1 = float(tokens[i])
                y1 = float(tokens[i + 1])

                x2 = float(tokens[i + 2])
                y2 = float(tokens[i + 3])

                x = float(tokens[i + 4])
                y = float(tokens[i + 5])

                i += 6

                if command == "c":

                    p1 = Point(
                        current.x + x1,
                        current.y + y1
                    )

                    p2 = Point(
                        current.x + x2,
                        current.y + y2
                    )

                    p3 = Point(
                        current.x + x,
                        current.y + y
                    )

                else:

                    p1 = Point(x1, y1)
                    p2 = Point(x2, y2)
                    p3 = Point(x, y)

                p0 = current

                for step in range(
                    1,
                    curve_steps + 1
                ):

                    t = (
                        step
                        / curve_steps
                    )

                    p = cubic_bezier(
                        p0,
                        p1,
                        p2,
                        p3,
                        t
                    )

                    current_path.add_point(
                        p.x,
                        p.y
                    )

                current = p3

                # Second control point is needed by S
                last_control = p2
                last_command = "C"

        # ====================================================
        # S - SMOOTH CUBIC BEZIER
        # ====================================================

        elif command in ("S", "s"):

            while (
                i < len(tokens)
                and not is_command(tokens[i])
            ):

                x2 = float(tokens[i])
                y2 = float(tokens[i + 1])

                x = float(tokens[i + 2])
                y = float(tokens[i + 3])

                i += 4

                # Reflect previous cubic control point
                if last_command in ("C", "S"):

                    p1 = Point(
                        2 * current.x
                        - last_control.x,

                        2 * current.y
                        - last_control.y
                    )

                else:

                    p1 = Point(
                        current.x,
                        current.y
                    )

                if command == "s":

                    p2 = Point(
                        current.x + x2,
                        current.y + y2
                    )

                    p3 = Point(
                        current.x + x,
                        current.y + y
                    )

                else:

                    p2 = Point(x2, y2)
                    p3 = Point(x, y)

                p0 = current

                for step in range(
                    1,
                    curve_steps + 1
                ):

                    t = (
                        step
                        / curve_steps
                    )

                    p = cubic_bezier(
                        p0,
                        p1,
                        p2,
                        p3,
                        t
                    )

                    current_path.add_point(
                        p.x,
                        p.y
                    )

                current = p3

                last_control = p2
                last_command = "S"

        # ====================================================
        # Q - QUADRATIC BEZIER
        # ====================================================

        elif command in ("Q", "q"):

            while (
                i < len(tokens)
                and not is_command(tokens[i])
            ):

                x1 = float(tokens[i])
                y1 = float(tokens[i + 1])

                x = float(tokens[i + 2])
                y = float(tokens[i + 3])

                i += 4

                if command == "q":

                    p1 = Point(
                        current.x + x1,
                        current.y + y1
                    )

                    p2 = Point(
                        current.x + x,
                        current.y + y
                    )

                else:

                    p1 = Point(x1, y1)
                    p2 = Point(x, y)

                p0 = current

                for step in range(
                    1,
                    curve_steps + 1
                ):

                    t = (
                        step
                        / curve_steps
                    )

                    p = quadratic_bezier(
                        p0,
                        p1,
                        p2,
                        t
                    )

                    current_path.add_point(
                        p.x,
                        p.y
                    )

                current = p2

                last_control = p1
                last_command = "Q"

        # ====================================================
        # T - SMOOTH QUADRATIC BEZIER
        # ====================================================

        elif command in ("T", "t"):

            while (
                i < len(tokens)
                and not is_command(tokens[i])
            ):

                x = float(tokens[i])
                y = float(tokens[i + 1])

                i += 2

                if command == "t":

                    p2 = Point(
                        current.x + x,
                        current.y + y
                    )

                else:

                    p2 = Point(x, y)

                if last_command in ("Q", "T"):

                    p1 = Point(
                        2 * current.x
                        - last_control.x,

                        2 * current.y
                        - last_control.y
                    )

                else:

                    p1 = Point(
                        current.x,
                        current.y
                    )

                p0 = current

                for step in range(
                    1,
                    curve_steps + 1
                ):

                    t = (
                        step
                        / curve_steps
                    )

                    p = quadratic_bezier(
                        p0,
                        p1,
                        p2,
                        t
                    )

                    current_path.add_point(
                        p.x,
                        p.y
                    )

                current = p2

                last_control = p1
                last_command = "T"

        # ====================================================
        # A - ELLIPTICAL ARC
        # ====================================================

        elif command in ("A", "a"):

            while (
                i < len(tokens)
                and not is_command(tokens[i])
            ):

                rx = float(tokens[i])
                ry = float(tokens[i + 1])

                rotation = float(tokens[i + 2])

                large_arc = int(
                    float(tokens[i + 3])
                )

                sweep = int(
                    float(tokens[i + 4])
                )

                x = float(tokens[i + 5])
                y = float(tokens[i + 6])

                i += 7

                if command == "a":

                    p1 = Point(
                        current.x + x,
                        current.y + y
                    )

                else:

                    p1 = Point(x, y)

                points = arc_to_points(
                    current,
                    rx,
                    ry,
                    rotation,
                    large_arc,
                    sweep,
                    p1,
                    arc_steps
                )

                for p in points:

                    current_path.add_point(
                        p.x,
                        p.y
                    )

                current = p1

                last_control = None
                last_command = "A"

        # ====================================================
        # Z - CLOSE PATH
        # ====================================================

        elif command in ("Z", "z"):

            current = Point(
                start.x,
                start.y
            )

            current_path.add_point(
                current.x,
                current.y
            )

            last_control = None
            last_command = "Z"

            command = None

    return paths
# ------------------------------------------------------------
# SVG shape → Path conversion
# ------------------------------------------------------------

def circle_to_path(element, circle_steps=40):
    """
    Convert an SVG <circle> into a Path.

    SVG:
        <circle cx="100" cy="100" r="50"/>
    """

    cx = float(element.get("cx", 0))
    cy = float(element.get("cy", 0))
    r = float(element.get("r", 0))

    path = Path()

    for step in range(circle_steps + 1):

        theta = 2 * math.pi * step / circle_steps

        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)

        path.add_point(x, y)

    return path


def rectangle_to_path(element):
    """
    Convert an SVG <rect> into a Path.

    Supports basic rectangles without rounded corners.

    SVG:
        <rect x="50" y="50" width="100" height="80"/>
    """

    x = float(element.get("x", 0))
    y = float(element.get("y", 0))

    width = float(element.get("width", 0))
    height = float(element.get("height", 0))

    path = Path()

    path.add_point(x, y)
    path.add_point(x + width, y)
    path.add_point(x + width, y + height)
    path.add_point(x, y + height)

    # Close rectangle
    path.add_point(x, y)

    return path


def polygon_to_path(element):
    """
    Convert an SVG <polygon> into a Path.

    This also handles triangles because an SVG triangle
    is normally represented as a polygon.

    SVG:
        <polygon points="200,150 250,50 300,150"/>
    """

    points_string = element.get("points", "")

    # Extract all numbers from the points attribute
    numbers = re.findall(
        r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?",
        points_string
    )

    if len(numbers) % 2 != 0:
        raise ValueError(
            "Invalid polygon: odd number of coordinates"
        )

    path = Path()

    for i in range(0, len(numbers), 2):

        x = float(numbers[i])
        y = float(numbers[i + 1])

        path.add_point(x, y)

    # Close polygon
    if len(path.points) > 0:

        first = path.points[0]

        path.add_point(
            first.x,
            first.y
        )

    return path

# ------------------------------------------------------------
# Read SVG
# ------------------------------------------------------------
def svg_to_paths(filename):

    tree = ET.parse(filename)
    root = tree.getroot()

    paths = []

    for element in root.iter():

        tag = element.tag.split("}")[-1]

        if tag == "path":

            d = element.get("d")

            if not d:
                continue

            new_paths = parse_path(d)

            paths.extend(new_paths)

        elif tag == "circle":

            path = circle_to_path(element)

            if len(path.points) > 0:
                paths.append(path)

        elif tag == "rect":

            path = rectangle_to_path(element)

            if len(path.points) > 0:
                paths.append(path)

        elif tag == "polygon":

            path = polygon_to_path(element)

            if len(path.points) > 0:
                paths.append(path)

    return paths
# ------------------------------------------------------------
# Test
# ------------------------------------------------------------

if __name__ == "__main__":

    filename = "/Users/gurjasdeep/Documents/Robocon/DraftBot/svg/testDrawing2.svg"

    paths = svg_to_paths(filename)

    print(f"Found {len(paths)} paths\n")

    for path_number, path in enumerate(paths):

        print(f"PATH {path_number + 1}")

        for point in path.points:
            print(f"    {point}")

        print()
