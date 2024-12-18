import numpy as np

def interpolate_route(route):
    """
    Interpolates 10 points between each consecutive pair of points in a route.

    Parameters:
        route (list of lists): A list of [x, y] coordinates representing the route.

    Returns:
        list of lists: A new route with interpolated points.
    """
    if len(route) < 2:
        raise ValueError("Route must contain at least two points to interpolate.")

    interpolated_route = []

    for i in range(len(route) - 1):
        x_start, y_start = route[i]
        x_end, y_end = route[i + 1]

        # Create 10 interpolated points between the current point and the next
        x_interp = np.linspace(x_start, x_end, 12)  # 12 includes start and end points
        y_interp = np.linspace(y_start, y_end, 12)

        # Add the interpolated points to the new route
        for j in range(len(x_interp) - 1):  # Exclude the last point to avoid duplication
            interpolated_route.append([x_interp[j], y_interp[j]])

    # Add the last point of the route to the interpolated route
    interpolated_route.append(route[-1])

    return interpolated_route

# Example usage
route = [[0, 0], [1, 1], [2, 0]]
interpolated = interpolate_route(route)
for point in interpolated:
    print(point)