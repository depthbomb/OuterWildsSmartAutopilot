using System;

namespace SmartAutopilot.Navigation;

internal static class GravityClearance
{
    public static double CalculateRadius(double radius, FlightState state, Func<float, float> gravityMagnitude)
    {
        // Local ruleset limits govern immediate control, not engine capacity at distant route segments.
        var maximumGravity = Math.Max(1, state.MaximumThrust * 0.35);
        for (var step = 0; step < 40 && gravityMagnitude((float)radius) > maximumGravity; step++)
        {
            radius *= 1.1;
        }

        return radius;
    }
}
