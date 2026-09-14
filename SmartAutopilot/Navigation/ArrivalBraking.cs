using System;

namespace SmartAutopilot.Navigation;

internal static class ArrivalBraking
{
    public static double LimitSpeed(FlightState state, double speed)
    {
        if (state.ArrivalThrustLimits.Count == 0)
        {
            return speed;
        }

        var restricted = false;
        foreach (var limit in state.ArrivalThrustLimits)
        {
            restricted |= limit.Radius > state.ArrivalRadius && limit.Thrust < state.Thrust;
        }

        if (!restricted)
        {
            return speed;
        }

        var distance         = (state.Position             - state.TargetPosition).Length;
        var gravity          = (state.ExternalAcceleration - state.TargetAcceleration).Length;
        var currentAuthority = Math.Max(0, state.Thrust - gravity) * 0.7;
        var finish           = state.ArrivalRadius + 20;
        var availableWork    = (double)0;
        // Integrate the lowest applicable thrust through each boundary, including overlapping volumes.
        for (var section = 0; section <= state.ArrivalThrustLimits.Count && distance > finish; section++)
        {
            var nextBoundary = finish;
            var thrust       = state.Thrust;
            foreach (var limit in state.ArrivalThrustLimits)
            {
                if (limit.Radius >= distance)
                {
                    thrust = Math.Min(thrust, limit.Thrust);
                }
                else if (limit.Radius > nextBoundary)
                {
                    nextBoundary = limit.Radius;
                }
            }

            var authority = Math.Max(0, thrust - gravity) * 0.7;
            availableWork += authority * (distance - nextBoundary);
            distance      =  nextBoundary;
        }

        var margin    = currentAuthority * 1.5;
        var safeSpeed = Math.Sqrt(2 * availableWork + margin * margin) - margin;

        return Math.Min(speed, safeSpeed);
    }
}
