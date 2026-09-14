using System;
using System.Collections.Generic;

namespace SmartAutopilot.Navigation;

internal readonly struct Waypoint
{
    public readonly Vector Position;
    public readonly int    ObstacleIndex;

    public Waypoint(Vector position, int obstacleIndex = -1)
    {
        Position      = position;
        ObstacleIndex = obstacleIndex;
    }
}

internal sealed class RoutePlanner
{
    public string FailureReason { get; private set; } = string.Empty;

    private double[] _cost      = [];
    private double[] _heuristic = [];
    private int[]    _previous  = [];
    private bool[]   _visited   = [];

    private readonly List<Waypoint> _nodes      = new(512);
    private readonly List<Vector>   _directions = new(26);
    private readonly List<Vector>   _goals      = new(64);

    public RoutePlanner()
    {
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                for (int z = -1; z <= 1; z++)
                {
                    var direction = new Vector(x, y, z);
                    if (direction.LengthSquared > 0)
                    {
                        _directions.Add(direction.Unit);
                    }
                }
            }
        }
    }

    public bool TryPlan(Vector                  start,
                        Vector                  goal,
                        IReadOnlyList<Obstacle> obstacles,
                        Vector                  preferredDirection,
                        out Waypoint            next)
        => TryPlan(start, goal, obstacles, preferredDirection, out next, out _, out _);

    public bool TryPlan(Vector                  start,
                        Vector                  goal,
                        IReadOnlyList<Obstacle> obstacles,
                        Vector                  preferredDirection,
                        out Waypoint            next,
                        out Waypoint            following,
                        out double              remainingDistance)
    {
        _goals.Clear();
        if (SegmentClear(goal, goal, obstacles))
        {
            _goals.Add(goal);
        }

        return Search(start, goal, 0, obstacles, preferredDirection, out _, out next, out following, out remainingDistance);
    }

    public bool TryPlanToTarget(Vector                  start,
                                Vector                  target,
                                double                  arrivalRadius,
                                IReadOnlyList<Obstacle> obstacles,
                                Vector                  preferredDirection,
                                out Vector              goal,
                                out Waypoint            next,
                                out Waypoint            following,
                                out double              remainingDistance)
    {
        goal = target + (start - target).Unit * arrivalRadius;
        if (TryPlan(start, goal, obstacles, preferredDirection, out next, out following, out remainingDistance))
        {
            return true;
        }

        // A blocked near-side point does not mean the destination is unreachable.
        _goals.Clear();
        foreach (var direction in _directions)
        {
            AddGoal(target + direction * arrivalRadius, obstacles);
        }

        for (int i = 0; i < obstacles.Count; i++)
        {
            var obstacle = obstacles[i];
            if (obstacle.Radius > 0)
            {
                AddGoal(target + (target - obstacle.Position).Unit * arrivalRadius, obstacles);
            }
        }

        return Search(start, target, arrivalRadius, obstacles, preferredDirection, out goal, out next, out following, out remainingDistance);
    }

    private bool Search(Vector start,
                        Vector target,
                        double arrivalRadius,
                        IReadOnlyList<Obstacle> obstacles,
                        Vector preferredDirection,
                        out Vector goal,
                        out Waypoint next,
                        out Waypoint following,
                        out double remainingDistance)
    {
        FailureReason     = string.Empty;
        goal              = _goals.Count > 0 ? _goals[0] : target;
        next              = new Waypoint(goal);
        following         = next;
        remainingDistance = (goal - start).Length;
        if (_goals.Count == 0)
        {
            FailureReason = "No clear arrival point outside the avoidance zones";

            return false;
        }

        var directDistance = double.PositiveInfinity;
        foreach (var candidate in _goals)
        {
            double candidateDistance = (candidate - start).Length;
            if (candidateDistance < directDistance && SegmentClear(start, candidate, obstacles))
            {
                directDistance = candidateDistance;
                goal           = candidate;
            }
        }

        if (!double.IsPositiveInfinity(directDistance))
        {
            next              = new Waypoint(goal);
            following         = next;
            remainingDistance = directDistance;

            return true;
        }

        _nodes.Clear();
        _nodes.Add(new Waypoint(start));
        foreach (var candidate in _goals)
        {
            _nodes.Add(new Waypoint(candidate));
        }

        for (int i = 0; i < obstacles.Count; i++)
        {
            var obstacle = obstacles[i];
            if (obstacle.Radius <= 0)
            {
                continue;
            }

            var shell = obstacle.Radius * 1.22 + 30;
            foreach (var direction in _directions)
            {
                AddNode(obstacle.Position + direction * shell, i, obstacles);
            }

            AddNode(obstacle.Position + (start  - obstacle.Position).Unit * shell, i, obstacles);
            AddNode(obstacle.Position + (target - obstacle.Position).Unit * shell, i, obstacles);
        }

        var count = _nodes.Count;
        if (_cost.Length < count)
        {
            _cost      = new double[count];
            _heuristic = new double[count];
            _previous  = new int[count];
            _visited   = new bool[count];
        }

        for (var i = 0; i < count; i++)
        {
            _cost[i]      = double.PositiveInfinity;
            _heuristic[i] = Math.Max(0, (_nodes[i].Position - target).Length - arrivalRadius);
            _previous[i]  = -1;
            _visited[i]   = false;
        }

        _cost[0] = 0;
        for (var iteration = 0; iteration < count; iteration++)
        {
            var current = -1;
            var best    = double.PositiveInfinity;
            for (var i = 0; i < count; i++)
            {
                var estimate = _cost[i] + _heuristic[i];
                if (!_visited[i] && estimate < best)
                {
                    best    = estimate;
                    current = i;
                }
            }

            if (current < 0)
            {
                FailureReason = "No connected route to the clear arrival points";

                return false;
            }

            if (current > 0 && current <= _goals.Count)
            {
                goal = _nodes[current].Position;

                var first  = current;
                var second = current;

                remainingDistance = 0;
                while (_previous[first] > 0)
                {
                    remainingDistance += (_nodes[first].Position - _nodes[_previous[first]].Position).Length;
                    second            =  first;
                    first             =  _previous[first];
                }

                next              =  _nodes[first];
                following         =  _nodes[second];
                remainingDistance += (next.Position - start).Length;

                return true;
            }

            _visited[current] = true;
            for (var i = 1; i < count; i++)
            {
                if (_visited[i])
                {
                    continue;
                }

                var edge       = _nodes[i].Position - _nodes[current].Position;
                var edgeLength = edge.Length;
                if (current == 0 && edgeLength < 60)
                {
                    continue;
                }

                var bias = current == 0 ? edgeLength * 0.12 * (1 - Vector.Dot(edge / edgeLength, preferredDirection)) : 0;
                var cost = _cost[current] + edgeLength + bias;
                if (cost < _cost[i] && SegmentClear(_nodes[current].Position, _nodes[i].Position, obstacles))
                {
                    _cost[i]     = cost;
                    _previous[i] = current;
                }
            }
        }

        FailureReason = "Route search exhausted";

        return false;
    }

    private void AddGoal(Vector position, IReadOnlyList<Obstacle> obstacles)
    {
        // Index interface-typed lists to avoid boxing their enumerators during planning.
        for (var i = 0; i < obstacles.Count; i++)
        {
            var obstacle = obstacles[i];
            var tooClose = obstacle.Radius > 0 && (position - obstacle.Position).Length < obstacle.Radius + 50;
            if (tooClose)
            {
                return;
            }
        }

        _goals.Add(position);
    }

    public static bool SegmentClear(Vector start, Vector end, IReadOnlyList<Obstacle> obstacles)
    {
        var delta         = end - start;
        var lengthSquared = delta.LengthSquared;
        for (var i = 0; i < obstacles.Count; i++)
        {
            var obstacle = obstacles[i];
            var offset   = start - obstacle.Position;
            var t        = lengthSquared > 1e-9 ? Math.Max(0, Math.Min(1, -Vector.Dot(offset, delta) / lengthSquared)) : 0;
            if ((offset + delta * t).LengthSquared < obstacle.Radius * obstacle.Radius)
            {
                return false;
            }
        }

        return true;
    }

    private void AddNode(Vector position, int obstacleIndex, IReadOnlyList<Obstacle> obstacles)
    {
        if (SegmentClear(position, position, obstacles))
        {
            _nodes.Add(new Waypoint(position, obstacleIndex));
        }
    }
}
