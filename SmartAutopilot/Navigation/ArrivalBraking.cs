using System;

namespace SmartAutopilot.Navigation
{
    internal static class ArrivalBraking
    {
        public static double LimitSpeed(FlightState state, double speed)
        {
            if (state.ArrivalThrustLimits.Count == 0)
            {
                return speed;
            }

            bool restricted = false;
            foreach (var limit in state.ArrivalThrustLimits)
            {
                restricted |= limit.Radius > state.ArrivalRadius && limit.Thrust < state.Thrust;
            }

            if (!restricted)
            {
                return speed;
            }

            double distance = (state.Position - state.TargetPosition).Length;
            double gravity = (state.ExternalAcceleration - state.TargetAcceleration).Length;
            double currentAuthority = Math.Max(0, state.Thrust - gravity) * 0.7;
            double finish = state.ArrivalRadius + 20;
            double availableWork = 0;
            // Integrate the lowest applicable thrust through each boundary, including overlapping volumes.
            for (int section = 0; section <= state.ArrivalThrustLimits.Count && distance > finish; section++)
            {
                double nextBoundary = finish;
                double thrust = state.Thrust;
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

                double authority = Math.Max(0, thrust - gravity) * 0.7;
                availableWork += authority * (distance - nextBoundary);
                distance = nextBoundary;
            }

            double margin = currentAuthority * 1.5;
            double safeSpeed = Math.Sqrt(2 * availableWork + margin * margin) - margin;

            return Math.Min(speed, safeSpeed);
        }
    }
}
