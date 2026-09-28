#!/usr/bin/env python3

import csv
import math
import os

import matplotlib.pyplot as plt


PROJECT_ROOT = os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))
)

DATA_DIR = os.path.join(
    PROJECT_ROOT,
    'data'
)

WAYPOINTS_FILE = os.path.join(
    DATA_DIR,
    'waypoints.csv'
)

ACTUAL_FILE = os.path.join(
    DATA_DIR,
    'actual_trajectory.csv'
)

COMPARISON_FILE = os.path.join(
    DATA_DIR,
    'trajectory_comparison.png'
)

ERROR_FILE = os.path.join(
    DATA_DIR,
    'cross_track_error.png'
)


def load_csv(filename):
    """Load x,y coordinates from a CSV file."""
    points = []

    with open(filename, newline='') as csv_file:
        reader = csv.DictReader(csv_file)

        for row in reader:
            points.append(
                (
                    float(row['x']),
                    float(row['y'])
                )
            )

    return points


def point_to_segment_distance(point, start, end):
    """Calculate the minimum distance from a point to a line segment."""
    px, py = point
    x1, y1 = start
    x2, y2 = end

    dx = x2 - x1
    dy = y2 - y1

    segment_length_squared = dx ** 2 + dy ** 2

    if segment_length_squared == 0:
        return math.hypot(
            px - x1,
            py - y1
        )

    t = (
        (px - x1) * dx
        + (py - y1) * dy
    ) / segment_length_squared

    t = max(
        0.0,
        min(1.0, t)
    )

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return math.hypot(
        px - closest_x,
        py - closest_y
    )


def calculate_cross_track_errors(
    reference,
    actual
):
    """Calculate minimum reference-path distance for each actual point."""
    errors = []

    for point in actual:
        minimum_error = float('inf')

        for index in range(
            len(reference) - 1
        ):
            distance = point_to_segment_distance(
                point,
                reference[index],
                reference[index + 1]
            )

            minimum_error = min(
                minimum_error,
                distance
            )

        errors.append(minimum_error)

    return errors


def cumulative_distance(points):
    """Calculate cumulative distance along a trajectory."""
    distances = [0.0]

    for index in range(1, len(points)):
        dx = (
            points[index][0]
            - points[index - 1][0]
        )
        dy = (
            points[index][1]
            - points[index - 1][1]
        )

        distances.append(
            distances[-1]
            + math.hypot(dx, dy)
        )

    return distances


def main():
    """Analyze and plot the executed trajectory."""
    reference = load_csv(WAYPOINTS_FILE)
    actual = load_csv(ACTUAL_FILE)

    errors = calculate_cross_track_errors(
        reference,
        actual
    )

    mean_error = sum(errors) / len(errors)
    maximum_error = max(errors)
    maximum_index = errors.index(maximum_error)

    actual_distance = cumulative_distance(actual)

    print(
        f'Reference points: {len(reference)}'
    )
    print(
        f'Actual points: {len(actual)}'
    )
    print(
        f'Mean cross-track error: '
        f'{mean_error:.4f} m'
    )
    print(
        f'Maximum cross-track error: '
        f'{maximum_error:.4f} m'
    )
    print(
        f'Maximum error position: '
        f'({actual[maximum_index][0]:.2f}, '
        f'{actual[maximum_index][1]:.2f}) m'
    )
    print(
        f'Distance along trajectory at maximum error: '
        f'{actual_distance[maximum_index]:.2f} m'
    )

    plt.figure()
    plt.plot(
        [point[0] for point in reference],
        [point[1] for point in reference],
        label='Reference'
    )
    plt.plot(
        [point[0] for point in actual],
        [point[1] for point in actual],
        label='Executed'
    )
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Reference vs Executed Trajectory')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(
        COMPARISON_FILE,
        dpi=300
    )
    plt.close()

    plt.figure()
    plt.plot(
        actual_distance,
        errors
    )
    plt.xlabel('Distance along trajectory [m]')
    plt.ylabel('Cross-track error [m]')
    plt.title('Cross-track Error')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(
        ERROR_FILE,
        dpi=300
    )
    plt.close()

    print(
        f'Figure saved to: {COMPARISON_FILE}'
    )
    print(
        f'Error plot saved to: {ERROR_FILE}'
    )


if __name__ == '__main__':
    main()
