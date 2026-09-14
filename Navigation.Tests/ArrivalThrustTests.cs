using System;
using SmartAutopilot.Navigation;

internal static class ArrivalThrustTests
{
    public static void Transition()
    {
        var baseline = Fly(false, true, 0.02, default, 1537, 264.7);
        Require(baseline.Minimum < -40, "Fixture no longer reproduces the unanticipated thrust-loss overshoot");
        foreach (double step in new[] { 0.02, 1.0 / 60, 0.05 })
        {
            foreach (bool detoured in new[] { true, false })
            {
                var result = Fly(true, detoured, step, new Vector(100, -20, 40), 1537, 264.7);
                Require(result.Minimum >= -5 && result.Arrival <= 30, "Anticipated thrust reduction missed the arrival shell");
            }
        }

        var longResult = Fly(true, false, 0.02, default, 20000, 0);
        Require(longResult.Minimum >= -5 && longResult.Arrival <= 30, "Long approach lost its future thrust restriction");
        var nestedResult = Fly(true, true, 0.02, default, 1537, 264.7, true);
        Require(nestedResult.Minimum >= -5 && nestedResult.Arrival <= 30, "Overlapping thrust reductions missed the arrival shell");
    }

    public static void BoundsAndValidation()
    {
        var state = new FlightState
        {
            Position      = new Vector(1000, 0, 0),
            ArrivalRadius = 400,
            Thrust        = 50,
            MaximumThrust = 50
        };
        Require(ArrivalBraking.LimitSpeed(state, 100) == 100, "Unrestricted arrival was slowed");
        state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(300, 10));
        Require(ArrivalBraking.LimitSpeed(state, 100) == 100, "A volume below the arrival shell slowed the approach");
        state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(515, 20));
        double bounded = ArrivalBraking.LimitSpeed(state, 1000);
        Require(bounded > 0 && bounded < 200, "Upcoming local limit did not constrain approach speed");
        state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(600, 10));
        Require(ArrivalBraking.LimitSpeed(state, 1000) < bounded, "A stricter overlapping limit was ignored");
        var translation = new Vector(1e7, -2e7, 3e7);
        bounded = ArrivalBraking.LimitSpeed(state, 1000);
        state.Position += translation;
        state.TargetPosition += translation;
        Require(Math.Abs(ArrivalBraking.LimitSpeed(state, 1000) - bounded) < 1e-7, "Floating-origin translation changed the arrival limit");
        state.ArrivalThrustLimits.Clear();
        state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(515, 0));
        state.Position = state.TargetPosition + new Vector(500, 0, 0);
        Require(ArrivalBraking.LimitSpeed(state, 1000) == 0, "A zero-thrust volume retained stopping authority");
        FlightValidation.Validate(state);
        state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(double.NaN, 20));
        bool rejected = false;
        try
        {
            FlightValidation.Validate(state);
        }
        catch (InvalidOperationException)
        {
            rejected = true;
        }

        Require(rejected, "Invalid predicted thrust geometry was accepted");
    }

    private static (double Arrival, double Minimum) Fly(bool forecast, bool detoured, double step, Vector drift, double distance, double speed, bool nested = false)
    {
        var state = new FlightState
        {
            Position       = new Vector(distance, 0, 0),
            Velocity       = drift + new Vector(-speed, 0, 0),
            TargetVelocity = drift,
            ArrivalRadius  = 400,
            Thrust         = 50,
            MaximumThrust  = 50,
            DeltaTime      = step
        };
        var computer = new FlightComputer();
        if (detoured)
        {
            state.Obstacles.Add(new Obstacle
            {
                Position = state.Position + new Vector(10, 0, 0),
                Radius   = 200
            });
            Require(computer.Step(state).Phase == FlightPhase.Escape, "Detoured fixture failed to enter complex-approach mode");
            state.Obstacles[0].Radius = 0;
        }

        if (forecast)
        {
            state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(515, nested ? 10 : 20));
            if (nested)
            {
                state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(715, 20));
            }
        }

        double entrySpeed = 0;
        double minimumError = double.PositiveInfinity;
        bool entered = false;
        for (int frame = 0; frame < 10000; frame++)
        {
            var offset = state.Position - state.TargetPosition;
            double currentDistance = offset.Length;
            minimumError = Math.Min(minimumError, currentDistance - state.ArrivalRadius);
            state.ExternalAcceleration = -offset.Unit * Math.Min(1.6, 250000 / offset.LengthSquared);
            bool restricted = currentDistance <= 500;
            if (restricted && !entered)
            {
                entrySpeed = (state.Velocity - drift).Length;
                entered = true;
            }

            state.Thrust = restricted ? (nested ? 10 : 20) : nested && currentDistance <= 700 ? 20 : 50;
            var command = computer.Step(state);
            Require(command.Acceleration.IsFinite && command.Acceleration.Length <= state.Thrust + 1e-7,
                "Arrival control exceeded actual thrust authority");
            Require(currentDistance > 200, "Arrival crossed the physical clearance margin");
            if (command.Phase == FlightPhase.Arrived)
            {
                double error = currentDistance - state.ArrivalRadius;
                Require((state.Velocity - drift).Length < 0.5, "Arrival retained relative velocity");
                Console.WriteLine($"THRUST forecast={forecast}; detoured={detoured}; nested={nested}; dt={step:F3}; start={distance:F0}m; arrival error={error:F2}m; minimum error={minimumError:F2}m; entry speed={entrySpeed:F2}m/s; time={state.Time:F2}s");

                return (error, minimumError);
            }

            state.Velocity += (command.Acceleration + state.ExternalAcceleration) * step;
            state.Position += state.Velocity * step;
            state.TargetPosition += drift * step;
            state.Time += step;
        }

        throw new Exception("Restricted-thrust arrival did not finish");
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new Exception(message);
        }
    }
}
