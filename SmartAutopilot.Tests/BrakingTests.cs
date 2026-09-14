using Xunit;
using System;
using SmartAutopilot.Navigation;

internal readonly struct BrakingResult
{
    public readonly double TotalTime;
    public readonly double BrakingTime;
    public readonly double SlowTime;
    public readonly double PeakSpeed;
    public readonly double ArrivalError;

    public BrakingResult(double totalTime, double brakingTime, double slowTime, double peakSpeed, double arrivalError)
    {
        TotalTime    = totalTime;
        BrakingTime  = brakingTime;
        SlowTime     = slowTime;
        PeakSpeed    = peakSpeed;
        ArrivalError = arrivalError;
    }
}

internal static class BrakingTests
{
    public static BrakingResult Fly(string name,
                                    double surfaceGravity,
                                    double thrust,
                                    double dt,
                                    Vector crossGravity,
                                    bool trace = false,
                                    double targetRadius = 0,
                                    Vector? initialVelocity = null,
                                    double initialDistance = 11193)
    {
        var drift = new Vector(150, 30, -20);
        var state = new FlightState
        {
            Position       = new Vector(-initialDistance, 0, 0),
            Velocity       = drift + (initialVelocity ?? new Vector(93.9, 0, 0)),
            TargetVelocity = drift,
            ArrivalRadius  = 2500,
            Thrust         = thrust,
            MaximumThrust  = 50,
            DeltaTime      = dt
        };
        var computer = new FlightComputer();
        if (targetRadius > 0)
        {
            state.TargetObstacleIndex = 0;
            state.Obstacles.Add(new Obstacle
            {
                Radius         = targetRadius,
                PhysicalRadius = targetRadius - 100,
                Velocity       = drift
            });
        }

        var brakingStart = (double)-1;
        var slowTime     = (double)0;
        var peakSpeed    = (double)0;
        var nextSample   = (double)0;
        for (var step = 0; step < 15000; step++)
        {
            state.Time = step * dt;
            var offset = state.TargetPosition - state.Position;

            state.ExternalAcceleration = offset.Unit * (surfaceGravity * Math.Pow(2500 / offset.Length, 2)) + crossGravity;

            var relativeVelocity = state.Velocity - state.TargetVelocity;
            var speed            = relativeVelocity.Length;
            var remaining        = offset.Length - state.ArrivalRadius;

            peakSpeed = Math.Max(peakSpeed, speed);

            var command = computer.Step(state);
            Assert.True(command.Phase != FlightPhase.Escape, "The arrival profile needlessly triggered target-body avoidance in " + name + " at " + state.Time + "s, remaining " + remaining + "m, speed " + speed + "m/s");
            Assert.True(command.Acceleration.IsFinite && command.Acceleration.Length <= state.Thrust + 1e-7, "Invalid thrust in " + name);

            if (brakingStart >= 0)
            {
                Assert.True(command.Phase == FlightPhase.Braking || command.Phase == FlightPhase.Arrived, "Approach/braking oscillation in " + name);
            }

            if (command.Phase == FlightPhase.Braking)
            {
                if (brakingStart < 0)
                {
                    brakingStart = state.Time;
                }

                if (speed < 30)
                {
                    slowTime += dt;
                }
            }

            if (trace && state.Time >= nextSample)
            {
                nextSample = state.Time + 1;
                double netBraking = -Vector.Dot(command.Acceleration + state.ExternalAcceleration, relativeVelocity.Unit);
                Console.WriteLine($"SAMPLE {name}: t={state.Time:F2}, phase={command.Phase}, remaining={remaining:F1}, speed={speed:F1}, gravity={state.ExternalAcceleration.Length:F2}, thrust={command.Acceleration.Length:F2}, netBraking={netBraking:F2}");
            }

            if (command.Phase == FlightPhase.Arrived)
            {
                Assert.True(Math.Abs(remaining) < 60 && speed < 0.5, "Unsafe arrival in " + name);

                var result = new BrakingResult(state.Time, state.Time - brakingStart, slowTime, peakSpeed, remaining);
                Console.WriteLine($"RESULT {name}: total={result.TotalTime:F2}s, braking={result.BrakingTime:F2}s, below30={result.SlowTime:F2}s, peak={result.PeakSpeed:F1}m/s, arrivalError={result.ArrivalError:F1}m");

                return result;
            }

            state.Velocity       += (command.Acceleration + state.ExternalAcceleration) * dt;
            state.Position       += state.Velocity * dt;
            state.TargetPosition += drift * dt;
            if (targetRadius > 0)
            {
                state.Obstacles[0].Position = state.TargetPosition;
            }
        }

        throw new Exception("Timed out in " + name);
    }

    public static void VerifyRefinement()
    {
        var clear   = Fly("Clear", 0, 50, 0.02, default);
        var gravity = Fly("Increasing attraction", 12, 50, 0.02, default);
        var cross   = Fly("Cross gravity and moving target", 0, 50, 1.0 / 60, new Vector(0, -8, 0));
        var limited = Fly("Locally limited thrust", 6, 20, 0.02, default);

        // The limited-thrust case must now restore its 25m overshoot before it can finish.
        Assert.True(limited.ArrivalError >= -10, "Limited thrust was accepted before restoring arrival distance");
        Assert.True(clear.BrakingTime < 16 && gravity.BrakingTime < 17 && cross.BrakingTime < 18 && limited.BrakingTime < 29, "Clear-approach braking regressed toward the previous drawn-out profile");
    }

    public static void TargetClearance()
    {
        Fly("Large target exclusion zone", 12, 50, 0.02, default, targetRadius: 2200);
        Fly("Tight target exclusion zone", 6, 30, 0.05, default, targetRadius: 2400);
    }

    public static void LateralMomentum()
    {
        Fly("Braking with initial lateral momentum", 0, 50, 0.02, default, initialVelocity: new Vector(500, 100, 0), initialDistance: 5500);
    }
}
