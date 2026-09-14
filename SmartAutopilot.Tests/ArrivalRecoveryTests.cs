using Xunit;
using System;
using SmartAutopilot.Navigation;

internal static class ArrivalRecoveryTests
{
    public static void SatelliteArrivalBoundary()
    {
        foreach (bool alreadyBraking in new[] { false, true })
        {
            foreach (double remaining in new[] { 20.1, 20.8 })
            {
                var state = Create(210, -remaining, new Vector(-0.1, 0, 0), 50, 0.02);
                state.ExternalAcceleration = state.TargetAcceleration;
                state.TargetObstacleIndex = 0;
                state.Obstacles.Add(new Obstacle
                {
                    Radius         = 130,
                    PhysicalRadius = 30,
                    Velocity       = state.TargetVelocity,
                    Acceleration   = state.TargetAcceleration
                });
                state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(215, 20));
                var computer = new FlightComputer();
                if (alreadyBraking)
                {
                    state.Position = new Vector(500, 0, 0);
                    state.Velocity = state.TargetVelocity + new Vector(-100, 0, 0);

                    Assert.True(computer.Step(state).Phase == FlightPhase.Braking, "Fixture did not enter braking");

                    state.Position = new Vector(210 + remaining, 0, 0);
                    state.Velocity = state.TargetVelocity + new Vector(-0.1, 0, 0);
                }

                Assert.True(computer.Step(state).Phase == FlightPhase.Arrived, "A velocity-matched satellite approach stalled just outside the 20m braking boundary");
            }
        }

        var outside = Create(210, -22, default, 50, 0.02);
        Assert.True(new FlightComputer().Step(outside).Phase != FlightPhase.Arrived, "The boundary tolerance accepted a distant approach");

        var fast = Create(210, -20.1, new Vector(-10, 0, 0), 50, 0.02);
        Assert.True(new FlightComputer().Step(fast).Phase != FlightPhase.Arrived, "The boundary tolerance accepted unmatched velocity");

        var obstructed = Create(210, -20.1, default, 50, 0.02);
        obstructed.Obstacles.Add(new Obstacle
        {
            Position       = obstructed.Position + new Vector(125, 0, 0),
            Velocity       = obstructed.TargetVelocity,
            Acceleration   = obstructed.TargetAcceleration,
            Radius         = 100,
            PhysicalRadius = 50
        });
        Assert.True(new FlightComputer().Step(obstructed).Phase == FlightPhase.Escape, "Arrival tolerance overrode collision avoidance");
    }

    public static void RecoverDistance()
    {
        foreach (double timestep in new[] { 0.02, 1.0 / 60, 0.05 })
        {
            // Distances observed after avoidance in the automated Lantern and Interloper routes.
            Simulate(600, 134.3, default, 50, timestep);
            Simulate(400, 57.5, new Vector(-24, 0, 0), 20, timestep);
            Simulate(1800, 8, new Vector(-25, 0, 0), 30, timestep);
        }
    }

    public static void AvoidancePriority()
    {
        var state = Create(600, 134.3, default, 50, 0.02);
        var computer = new FlightComputer();
        var recovery = computer.Step(state);

        Assert.True(recovery.Phase != FlightPhase.Arrived, "Arrival was accepted before restoring distance");

        state.Obstacles.Add(new Obstacle
        {
            Position       = state.Position + new Vector(125, 0, 0),
            Velocity       = state.TargetVelocity,
            Acceleration   = state.TargetAcceleration,
            Radius         = 100,
            PhysicalRadius = 50
        });
        state.Time += state.DeltaTime;
        var avoidance = computer.Step(state);

        Assert.True(avoidance is { Phase: FlightPhase.Escape, Acceleration.X: < 0 }, "Distance recovery overrode avoidance of a body in the outward path");
    }

    private static void Simulate(double radius, double deficit, Vector relativeVelocity, double thrust, double timestep)
    {
        var state    = Create(radius, deficit, relativeVelocity, thrust, timestep);
        var computer = new FlightComputer();
        for (int step = 0; step < 2000; step++)
        {
            state.Time = step * timestep;
            var command = computer.Step(state);

            Assert.True(command.Acceleration.IsFinite && command.Acceleration.Length <= thrust + 1e-7, "Distance recovery exceeded available thrust");

            var error = (state.Position - state.TargetPosition).Length - radius;
            if (command.Phase == FlightPhase.Arrived)
            {
                Assert.True(error is >= -10 and <= 10, "Arrival was accepted with distance error " + error);
                Assert.True((state.Velocity - state.TargetVelocity).Length < 0.5, "Recovery did not match target velocity");
                Assert.True(state.Time < 25, "Distance recovery took too long");
                return;
            }

            state.Position       += state.Velocity * timestep + (command.Acceleration + state.ExternalAcceleration) * (0.5 * timestep * timestep);
            state.Velocity       += (command.Acceleration + state.ExternalAcceleration) * timestep;
            state.TargetPosition += state.TargetVelocity * timestep + state.TargetAcceleration * (0.5 * timestep * timestep);
            state.TargetVelocity += state.TargetAcceleration * timestep;
        }

        throw new Exception("Distance recovery did not finish");
    }

    private static FlightState Create(double radius, double deficit, Vector relativeVelocity, double thrust, double timestep)
    {
        var targetVelocity = new Vector(130, 20, -60);
        return new FlightState
        {
            Position             = new Vector(radius - deficit, 0, 0),
            Velocity             = targetVelocity + relativeVelocity,
            TargetVelocity       = targetVelocity,
            TargetAcceleration   = new Vector(0.4, -0.3, 0.2),
            ExternalAcceleration = new Vector(-1, -3, 0.5),
            ArrivalRadius        = radius,
            Thrust               = thrust,
            MaximumThrust        = thrust,
            DeltaTime            = timestep
        };
    }
}
