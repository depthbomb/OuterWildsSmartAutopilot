using System;

namespace SmartAutopilot.Navigation;

internal readonly struct CollisionFrame
{
    public readonly Vector Velocity;
    public readonly Vector Acceleration;
    public readonly double RotationWeight;

    private CollisionFrame(Vector velocity, Vector acceleration, double rotationWeight)
    {
        Velocity       = velocity;
        Acceleration   = acceleration;
        RotationWeight = rotationWeight;
    }

    public static CollisionFrame Create(Vector offset,
                                        Vector velocity,
                                        Vector acceleration,
                                        Vector angularVelocity,
                                        Vector angularAcceleration,
                                        double physicalRadius)
    {
        var gap                    = offset.Length - Math.Max(0, physicalRadius);
        var blend                  = Math.Max(0, Math.Min(1, (250 - gap) / 200));
        var weight                 = blend * blend * (3 - 2 * blend);
        var tangent                = Cross(angularVelocity, offset);
        var rotationalAcceleration = Cross(angularVelocity, tangent) + Cross(angularAcceleration, offset);

        return new CollisionFrame(velocity + tangent * weight, acceleration + rotationalAcceleration * weight, weight);
    }

    private static Vector Cross(Vector a, Vector b) => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
}

internal sealed class CollisionHoldRelease
{
    private double _clearSince = -1;

    public bool Step(bool clear, bool holding, double time)
    {
        if (!clear)
        {
            _clearSince = -1;

            return false;
        }

        if (!holding)
        {
            _clearSince = -1;

            return true;
        }

        if (_clearSince < 0 || time < _clearSince)
        {
            _clearSince = time;
        }

        return time - _clearSince >= 0.75;
    }
}
