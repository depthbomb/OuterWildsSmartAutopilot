using System;

namespace SmartAutopilot.Navigation;

internal readonly struct OrbitalMotion
{
    private readonly Vector _center;
    private readonly Vector _centerVelocity;
    private readonly Vector _centerAcceleration;
    private readonly Vector _radial;
    private readonly Vector _tangent;
    private readonly double _radius;
    private readonly double _radialSpeed;
    private readonly double _angularSpeed;
    private readonly double _accelerationError;
    private readonly bool   _stableRadius;
    private readonly int    _primaryIndex;

    private OrbitalMotion(Obstacle body, Obstacle primary, int primaryIndex, Vector offset, Vector tangentVelocity)
    {
        _primaryIndex       = primaryIndex;
        _center             = primary.Position;
        _centerVelocity     = primary.Velocity;
        _centerAcceleration = primary.Acceleration;
        _radial             = offset.Unit;
        _tangent            = tangentVelocity.Unit;
        _radius             = offset.Length;
        _radialSpeed        = Vector.Dot(body.Velocity - primary.Velocity, _radial);
        _angularSpeed       = tangentVelocity.Length / _radius;

        var centripetal = tangentVelocity.LengthSquared / _radius;

        _accelerationError = (body.Acceleration - primary.Acceleration + _radial * centripetal).Length;
        _stableRadius      = Math.Abs(_radialSpeed) < tangentVelocity.Length * 0.02 && _accelerationError < centripetal * 0.05;
    }

    public Vector Position(double time)
    {
        var angle     = _angularSpeed * time;
        var direction = _radial * Math.Cos(angle) + _tangent * Math.Sin(angle);

        return _center + _centerVelocity * time + _centerAcceleration * (0.5 * time * time) + direction * (_radius + _radialSpeed * time);
    }

    public bool ExcludesLinearPath(Vector position, Vector velocity, double clearance, double horizon)
    {
        if (!_stableRadius || _centerAcceleration.Length > 0.01)
        {
            return false;
        }

        var offset           = position - _center;
        var relativeVelocity = velocity - _centerVelocity;
        var closestTime      = relativeVelocity.LengthSquared > 1e-9 ? Math.Max(0, Math.Min(horizon, -Vector.Dot(offset, relativeVelocity) / relativeVelocity.LengthSquared)) : 0;

        // Enclose the whole orbit, with radius uncertainty, instead of projecting a tangent out of it.
        var envelope = _radius * 1.1 + Math.Abs(_radialSpeed) * horizon + clearance;

        return (offset + relativeVelocity * closestTime).LengthSquared > envelope * envelope;
    }

    public static bool TryCreate(FlightState state, int index, out OrbitalMotion motion)
    {
        motion = default;
        if (index < 0 || index >= state.Obstacles.Count)
        {
            return false;
        }

        return TryCreateAround(state, index, state.Obstacles[index].PrimaryIndex, out motion);
    }

    public static bool ExcludesCollision(FlightState state, int index, double clearance, double horizon)
    {
        if (TryCreate(state, index, out var orbit))
        {
            return orbit.IsProtected(state, clearance, horizon)
                   || orbit.ExcludesLinearPath(state.Position, state.Velocity, clearance, horizon)
                   || orbit.ExcludesCurvedPath(state, clearance, horizon);
        }

        var missingPrimary = index >= 0 && index < state.Obstacles.Count && state.Obstacles[index].PrimaryIndex < 0;
        if (!missingPrimary)
        {
            return false;
        }

        // Some station parts lack an AstroObject primary. Accept only a stable orbit wholly inside another exclusion zone.
        for (var primary = 0; primary < state.Obstacles.Count; primary++)
        {
            if (TryCreateAround(state, index, primary, out orbit) && orbit.IsProtected(state, clearance, horizon))
            {
                return true;
            }
        }

        return false;
    }

    private bool IsProtected(FlightState state, double clearance, double horizon)
    {
        if (!_stableRadius || _centerAcceleration.Length > 0.01)
        {
            return false;
        }

        var primaryRadius = state.Obstacles[_primaryIndex].Radius;
        var envelope      = _radius * 1.1 + Math.Abs(_radialSpeed) * horizon + clearance;

        // The primary's ordinary braking and escape checks remain active and protect this entire nested orbit.
        return envelope < primaryRadius && (state.Position - _center).Length > primaryRadius + 100;
    }

    private bool ExcludesCurvedPath(FlightState state, double clearance, double horizon)
    {
        if (!_stableRadius || horizon <= 0 || horizon > 60)
        {
            return false;
        }

        var movingPrimary = TryCreate(state, _primaryIndex, out var primaryOrbit) && primaryOrbit._stableRadius;
        var centerError   = movingPrimary ? primaryOrbit._accelerationError + primaryOrbit._centerAcceleration.Length : _centerAcceleration.Length * 1.1;

        var curveAcceleration = _angularSpeed * _angularSpeed * (_radius + Math.Abs(_radialSpeed) * horizon) + 2 * _angularSpeed * Math.Abs(_radialSpeed) + _centerAcceleration.Length * 1.1;

        var segments = Math.Max(1, (int)Math.Ceiling(horizon / 0.5));
        var step     = horizon / segments;
        var previous = state.Position - Position(0);
        for (var sample = 1; sample <= segments; sample++)
        {
            var time      = sample * step;
            var predicted = Position(time);
            // Follow the primary's orbit too; its instantaneous acceleration is not a permanent straight-line force.
            predicted += movingPrimary ? primaryOrbit.Position(time) - _center - _centerVelocity * time - _centerAcceleration * (0.5 * time * time)
                : -_centerAcceleration * (0.5 * time * time);
            var current     = state.Position + state.Velocity * time - predicted;
            var delta       = current                                - previous;
            var fraction    = delta.LengthSquared > 1e-9 ? Math.Max(0, Math.Min(1, -Vector.Dot(previous, delta) / delta.LengthSquared)) : 0;
            var uncertainty = _radius * 0.1 + (_accelerationError + centerError)            * (0.5 * time * time);
            var envelope    = clearance     + uncertainty + curveAcceleration * step * step / 8;
            if ((previous + delta * fraction).Length <= envelope)
            {
                return false;
            }

            previous = current;
        }

        return true;
    }

    private static bool TryCreateAround(FlightState state, int index, int primaryIndex, out OrbitalMotion motion)
    {
        motion = default;
        if (primaryIndex < 0 || primaryIndex >= state.Obstacles.Count || primaryIndex == index)
        {
            return false;
        }

        var primary              = state.Obstacles[primaryIndex];
        var body                 = state.Obstacles[index];
        var offset               = body.Position     - primary.Position;
        var velocity             = body.Velocity     - primary.Velocity;
        var acceleration         = body.Acceleration - primary.Acceleration;
        var radialSpeed          = Vector.Dot(velocity, offset.Unit);
        var tangent              = velocity - offset.Unit * radialSpeed;
        var inwardAcceleration   = -Vector.Dot(acceleration, offset.Unit);
        var expectedAcceleration = tangent.LengthSquared / Math.Max(1, offset.Length);

        var orbital = primary.Radius                                      > 0                           &&
                      body.Radius                                         > 0                           &&
                      offset.Length                                       > 100                         &&
                      tangent.Length                                      > 10                          &&
                      Math.Abs(radialSpeed)                               < tangent.Length       * 0.2  &&
                      inwardAcceleration                                  > acceleration.Length  * 0.85 &&
                      Math.Abs(inwardAcceleration - expectedAcceleration) < expectedAcceleration * 0.5;
        if (!orbital)
        {
            return false;
        }

        motion = new OrbitalMotion(body, primary, primaryIndex, offset, tangent);

        return true;
    }
}
