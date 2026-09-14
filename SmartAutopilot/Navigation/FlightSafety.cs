using System;

namespace SmartAutopilot.Navigation;

internal static class FlightValidation
{
    public static void Validate(FlightState state)
    {
        var valid = state.Position.IsFinite          && state.Velocity.IsFinite       && state.ExternalAcceleration.IsFinite
                    && state.TargetPosition.IsFinite && state.TargetVelocity.IsFinite && state.TargetAcceleration.IsFinite
                    && Finite(state.Thrust)          && state.Thrust        > 0       && Finite(state.MaximumThrust) && state.MaximumThrust >= state.Thrust
                    && Finite(state.ArrivalRadius)   && state.ArrivalRadius >= 0      && Finite(state.DeltaTime)     && state.DeltaTime     > 0
                    && Finite(state.Time)            && Finite(state.SpeedLimit)      && state.SpeedLimit                                   >= 0;
        if (!valid)
        {
            throw new InvalidOperationException("The game supplied invalid flight data. Navigation cannot continue safely.");
        }

        foreach (var obstacle in state.Obstacles)
        {
            var usable = Finite(obstacle.Radius) && obstacle.Radius >= 0 && (obstacle.Radius == 0 || obstacle.Position.IsFinite && obstacle.Velocity.IsFinite && obstacle.Acceleration.IsFinite && Finite(obstacle.PhysicalRadius) && obstacle.PhysicalRadius >= 0);
            if (!usable)
            {
                throw new InvalidOperationException("The game supplied invalid obstacle data. Navigation cannot continue safely.");
            }
        }

        foreach (var limit in state.ArrivalThrustLimits)
        {
            bool usable = Finite(limit.Radius) && limit.Radius > 0 && Finite(limit.Thrust) && limit.Thrust >= 0;
            if (!usable)
            {
                throw new InvalidOperationException("The game supplied invalid arrival thrust limits.");
            }
        }
    }

    private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
}

internal sealed class FlightFaults
{
    public bool Locked  => _failures >= 3;
    public bool Faulted => _faulted;

    private int    _failures;
    private double _retryAt;
    private bool   _faulted;

    public bool TryEngage(double time)
    {
        var allowed = !Locked && (!_faulted || time >= _retryAt);
        if (allowed)
        {
            _faulted = false;
        }

        return allowed;
    }

    public bool Record(double time)
    {
        if (_faulted)
        {
            return false;
        }

        _faulted = true;
        _failures++;
        _retryAt = time + 5;

        return true;
    }
}

internal sealed class HoldMonitor
{
    public double Duration       { get; private set; }
    public double FuelUsed       { get; private set; }
    public bool   NeedsAttention { get; private set; }

    private double _started = -1;
    private double _initialFuel;

    public void Step(bool blocked, double time, double fuelFraction)
    {
        if (!blocked)
        {
            _started       = -1;
            Duration       = 0;
            FuelUsed       = 0;
            NeedsAttention = false;

            return;
        }

        if (_started < 0 || time < _started)
        {
            _started     = time;
            _initialFuel = fuelFraction;
        }

        Duration       = Math.Max(0, time         - _started);
        FuelUsed       = Math.Max(0, _initialFuel - fuelFraction);
        NeedsAttention = Duration >= 20 || FuelUsed >= 0.05 || fuelFraction <= 0.15;
    }
}

internal static class CollisionBraking
{
    public static double ProbeDistance(FlightState state, Vector frameVelocity, Vector frameAcceleration)
    {
        var speed   = (state.Velocity - frameVelocity).Length;
        var reserve = Math.Max(0.1, state.Thrust - (state.ExternalAcceleration - frameAcceleration).Length);

        return Math.Max(30, speed * speed / (2 * reserve) + speed * 1.5 + 15);
    }

    public static FlightCommand Stop(FlightState state, Vector frameVelocity, Vector frameAcceleration)
    {
        var acceleration = (frameVelocity - state.Velocity) / Math.Max(0.001, state.DeltaTime) + frameAcceleration - state.ExternalAcceleration;

        return new FlightCommand(acceleration.Limited(state.Thrust), FlightPhase.Hold);
    }
}
