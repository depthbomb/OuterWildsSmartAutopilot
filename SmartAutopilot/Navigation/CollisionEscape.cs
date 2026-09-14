using System;

namespace SmartAutopilot.Navigation;

internal static class CollisionEscape
{
    public static bool TrySelect(FlightState state,             FlightCommand                      requested,    int               dangerIndex, Vector frameVelocity,
                                 Vector      frameAcceleration, Func<Vector, Vector, double, bool> segmentClear, out FlightCommand selected)
    {
        selected = default;
        var valid = requested.Phase == FlightPhase.Escape && dangerIndex >= 0 && dangerIndex < state.Obstacles.Count;
        if (!valid)
        {
            return false;
        }

        var outward = (state.Position - state.Obstacles[dangerIndex].Position).Unit;
        var axis    = Math.Abs(outward.Y) < 0.8 ? new Vector(0, 1, 0) : new Vector(1, 0, 0);
        var side    = (axis - outward * Vector.Dot(axis, outward)).Unit;
        var other   = new Vector(outward.Y * side.Z - outward.Z * side.Y, outward.Z * side.X - outward.X * side.Z, outward.X * side.Y - outward.Y * side.X);
        for (var attempt = 0; attempt < 9; attempt++)
        {
            var acceleration = requested.Acceleration;
            if (attempt > 0)
            {
                var direction = (attempt - 1) % 4;
                var lateral   = direction < 2 ? side : other;
                lateral *= direction % 2 == 0 ? 1 : -1;

                var weight = attempt <= 4 ? 1 : 3;
                acceleration = (outward + lateral * weight).Unit * state.Thrust;
            }

            // Every alternative must still thrust away from the celestial hazard.
            var escaping = Vector.Dot(acceleration, outward) > 0;
            if (escaping && PathClear(state, acceleration, frameVelocity, frameAcceleration, segmentClear))
            {
                selected = new FlightCommand(acceleration, FlightPhase.Escape);

                return true;
            }
        }

        return false;
    }

    public static bool PathClear(FlightState                        state,
                                 Vector                             acceleration,
                                 Vector                             frameVelocity,
                                 Vector                             frameAcceleration,
                                 Func<Vector, Vector, double, bool> segmentClear)
    {
        const double step = 0.25;

        var velocity = state.Velocity - frameVelocity;
        var position = default(Vector);
        var gravity  = state.ExternalAcceleration - frameAcceleration;
        for (var sample = 0; sample < 60; sample++)
        {
            var time         = sample * step;
            // Check the proposed maneuver plus a complete braking fallback in the obstacle's frame.
            var thrust       = time < 1.5 ? acceleration : (-velocity / step - gravity).Limited(state.Thrust);
            var net          = thrust   + gravity;
            var next         = position + velocity * step + net * (0.5 * step * step);
            var curvePadding = net.Length          * step       * step / 8;
            if (!segmentClear(position, next, curvePadding))
            {
                return false;
            }

            var endTime    = time           + step;
            var worldStart = state.Position + frameVelocity * time    + frameAcceleration * (0.5 * time    * time)    + position;
            var worldEnd   = state.Position + frameVelocity * endTime + frameAcceleration * (0.5 * endTime * endTime) + next;
            foreach (var body in state.Obstacles)
            {
                if (body.Radius <= 0 || body.PhysicalRadius <= 0)
                {
                    continue;
                }

                var startOffset = worldStart - body.Position - body.Velocity * time    - body.Acceleration * (0.5 * time    * time);
                var endOffset   = worldEnd   - body.Position - body.Velocity * endTime - body.Acceleration * (0.5 * endTime * endTime);
                var delta       = endOffset  - startOffset;
                var fraction    = delta.LengthSquared > 1e-9 ? Math.Max(0, Math.Min(1, -Vector.Dot(startOffset, delta) / delta.LengthSquared)) : 0;
                var radius      = body.PhysicalRadius + 15 + (net + frameAcceleration - body.Acceleration).Length * step * step / 8;
                if ((startOffset + delta * fraction).Length < radius)
                {
                    return false;
                }
            }

            velocity += net * step;
            position =  next;
            if (endTime >= 1.5 && velocity.Length < 0.5)
            {
                return true;
            }
        }

        // An unbounded stopping path cannot be certified as a safe escape corridor.
        return false;
    }
}
