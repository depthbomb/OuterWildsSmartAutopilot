using System;
using System.Collections.Generic;

namespace SmartAutopilot.Navigation;

internal readonly struct FlightSample
{
    public readonly double Time;
    public readonly double Remaining;
    public readonly double Speed;
    public readonly double Thrust;
    public readonly double AvailableThrust;
    public readonly double Fuel;
    public readonly Vector Position;
    public readonly Vector Velocity;
    public readonly Vector Gravity;
    public readonly string Phase;

    public FlightSample(FlightState state, Vector acceleration, double fuel, string phase)
    {
        Time            = state.Time;
        Remaining       = (state.TargetPosition - state.Position).Length - state.ArrivalRadius;
        Speed           = (state.Velocity - state.TargetVelocity).Length;
        Thrust          = acceleration.Length;
        AvailableThrust = state.Thrust;
        Fuel            = fuel;
        Position        = state.Position             - state.TargetPosition;
        Velocity        = state.Velocity             - state.TargetVelocity;
        Gravity         = state.ExternalAcceleration - state.TargetAcceleration;
        Phase           = phase;
    }

    public override string ToString()
        => FormattableString.Invariant($"t={Time:F2}s; phase={Phase}; remaining={Remaining:F1}m; speed={Speed:F1}m/s; thrust={Thrust:F2}/{AvailableThrust:F2}; fuel={Fuel:P1}; targetRelativePosition=({Position.X:F1},{Position.Y:F1},{Position.Z:F1}); targetRelativeVelocity=({Velocity.X:F1},{Velocity.Y:F1},{Velocity.Z:F1}); relativeGravity=({Gravity.X:F2},{Gravity.Y:F2},{Gravity.Z:F2})");
}

internal sealed class FlightRecorder
{
    public int Count => _samples.Count;

    private readonly Queue<FlightSample> _samples = new(120);

    public void Add(FlightSample sample)
    {
        if (_samples.Count == 120)
        {
            _samples.Dequeue();
        }

        _samples.Enqueue(sample);
    }

    public void Dump(Action<string> write)
    {
        foreach (var sample in _samples)
        {
            write(sample.ToString());
        }
    }

    public void Clear()
    {
        _samples.Clear();
    }
}
