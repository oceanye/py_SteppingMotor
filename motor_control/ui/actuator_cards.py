"""Role-unit canvas rendering shared by X42S and archived STEP/DIR cards."""
import math


def draw_linear(canvas, snapshot):
    canvas.delete("all")
    width, height = 92, 112
    canvas.create_rectangle(37, 9, 55, height - 9, outline="#94a3b8", width=2)
    position = snapshot.get("position")
    minimum = snapshot.get("travel_min")
    maximum = snapshot.get("travel_max")
    valid_range = (
        snapshot.get("position_trusted", False)
        and isinstance(position, (int, float))
        and isinstance(minimum, (int, float))
        and isinstance(maximum, (int, float))
        and math.isfinite(float(position))
        and math.isfinite(float(minimum))
        and math.isfinite(float(maximum))
        and maximum > minimum
    )
    if valid_range:
        fraction = max(0.0, min(1.0, (position - minimum) / (maximum - minimum)))
        y = height - 9 - fraction * (height - 18)
        canvas.create_rectangle(39, y, 53, height - 11, fill="#2f80ed", outline="")
        canvas.create_line(25, y, 67, y, fill="#1565c0", width=2)
        canvas.create_text(46, 4, text="max", anchor="s", fill="#64748b", font=("Arial", 7))
        canvas.create_text(46, height - 3, text="min", anchor="n", fill="#64748b", font=("Arial", 7))
    else:
        canvas.create_text(
            width / 2,
            height / 2,
            text="无有效\n行程范围",
            justify="center",
            fill="#64748b",
            font=("Microsoft YaHei", 9),
        )


def draw_rotary(canvas, snapshot):
    canvas.delete("all")
    center_x, center_y, radius = 46, 55, 36
    canvas.create_oval(
        center_x - radius,
        center_y - radius,
        center_x + radius,
        center_y + radius,
        outline="#94a3b8",
        width=2,
    )
    for degree in range(0, 360, 45):
        angle = math.radians(degree - 90)
        x1 = center_x + (radius - 5) * math.cos(angle)
        y1 = center_y + (radius - 5) * math.sin(angle)
        x2 = center_x + radius * math.cos(angle)
        y2 = center_y + radius * math.sin(angle)
        canvas.create_line(x1, y1, x2, y2, fill="#64748b")
    position = snapshot.get("position")
    if snapshot.get("position_trusted", False) and isinstance(position, (int, float)) and math.isfinite(float(position)):
        angle = math.radians((float(position) % 360.0) - 90.0)
        x = center_x + (radius - 9) * math.cos(angle)
        y = center_y + (radius - 9) * math.sin(angle)
        canvas.create_line(center_x, center_y, x, y, fill="#1565c0", width=3, tags=("role_position",))
        canvas.create_oval(center_x - 3, center_y - 3, center_x + 3, center_y + 3, fill="#1565c0", outline="")
        canvas.create_text(center_x, 101, text=f"{float(position) % 360.0:.1f}°", fill="#334155", font=("Consolas", 9))
    else:
        canvas.create_text(center_x, center_y, text="--", fill="#64748b", font=("Consolas", 12))
