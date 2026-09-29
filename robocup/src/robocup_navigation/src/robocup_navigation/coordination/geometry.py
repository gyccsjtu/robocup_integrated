"""Continuous 3-D distance checks for conservative route reservations."""
import math


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def segment_distance(p, q, r, s):
    """Minimum distance between two CLOSED segments, including degeneracies."""
    u, v, w = sub(q, p), sub(s, r), sub(p, r)
    a, b, c, d, e = dot(u, u), dot(u, v), dot(v, v), dot(u, w), dot(v, w)
    if a <= 1e-20 and c <= 1e-20:
        return math.dist(p, r)
    if a <= 1e-20:
        x, y = 0., max(0., min(1., e / c))
    elif c <= 1e-20:
        x, y = max(0., min(1., -d / a)), 0.
    else:
        den = a * c - b * b
        x = max(0., min(1., (b * e - c * d) / den)) if den > 1e-20 else 0.
        y = (b * x + e) / c
        if y < 0.:
            y, x = 0., max(0., min(1., -d / a))
        elif y > 1.:
            y, x = 1., max(0., min(1., (b - d) / a))
    return math.sqrt(max(0., sum((w[i] + x * u[i] - y * v[i]) ** 2 for i in range(3))))


def route_distance(a, b):
    return min(segment_distance(p, q, r, s)
               for p, q in zip(a, a[1:]) for r, s in zip(b, b[1:]))


def route_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def matching(vehicles, tasks, costs):
    """Max-cardinality then min-cost matching, deterministic, at most six UAVs.

    DP over the vehicle bitmask is O(tasks * 2**vehicles * vehicles).
    Missing cost edges are infeasible, never replaced with straight-line cost.
    """
    vehicles, tasks = sorted(vehicles), sorted(tasks)
    states = {0: (0., ())}
    for task in tasks:
        nxt = dict(states)
        for mask, (cost, pairs) in states.items():
            for i, vehicle in enumerate(vehicles):
                if mask & (1 << i) or (vehicle, task) not in costs:
                    continue
                key = mask | (1 << i)
                candidate = (cost + costs[vehicle, task], pairs + ((vehicle, task),))
                if key not in nxt or candidate < nxt[key]:
                    nxt[key] = candidate
        states = nxt
    return list(min(states.items(), key=lambda item:
                    (-bin(item[0]).count('1'), item[1][0], item[1][1]))[1][1])
