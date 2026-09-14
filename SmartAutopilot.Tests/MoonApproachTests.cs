using System;
using SmartAutopilot.Navigation;

internal static class MoonApproachTests
{
    public static void Verify()
    {
        Run(true);
    }

    public static void Recovery()
    {
        RecoveredApproach(true);
    }

    private static void RecoveredApproach(bool verify)
    {
        // Representative local replay near the first escape's release in the 17:59 playtest.
        foreach (double timestep in new[] { 0.02, 1.0 / 60, 0.05 })
        {
            Flight("Recovered approach", new Vector(1719.0, 25.8, -75.2), new Vector(20.2, 5.7, 91.3), 3.5945, verify, timestep);
        }
    }

    private static void Run(bool verify)
    {
        Flight("Lateral approach", new Vector(-3018.2, -451.1, -2073.2), new Vector(275.1, -32.9, -253.5), 0.75, verify);
        Flight("Long approach", new Vector(12366.2, 12.8, -2093.6), new Vector(-306.4, 92.5, 637.3), 2, verify);
        Flight("Opposite approach", new Vector(-8000, 1200, 3000), new Vector(120, 0, -350), 4, verify);
    }

    private static void Flight(string name, Vector offset, Vector velocity, double phase, bool verify, double timestep = 0.02)
    {
        const double omega = 0.055;
        var state = new FlightState
        {
            Thrust              = 50,
            MaximumThrust       = 50,
            ArrivalRadius       = 600,
            TargetObstacleIndex = 1,
            DeltaTime           = timestep
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius         = 500,
            PhysicalRadius = 350
        });
        state.Obstacles.Add(new Obstacle
        {
            PrimaryIndex   = 0,
            Radius         = 200,
            PhysicalRadius = 100
        });
        var computer = new FlightComputer();
        var arrived  = false;
        var effort   = (double)0;
        var minimum  = double.PositiveInfinity;
        var escapes  = 0;
        var escaping = false;
        for (var step = 0; step < 10000; step++)
        {
            state.Time = step * state.DeltaTime;
            
            var primary = state.Obstacles[0];
            primary.Acceleration = new Vector(2.5, 0, 1);
            primary.Velocity     = new Vector(100, 0, 150) + primary.Acceleration * state.Time;
            primary.Position     = new Vector(100, 0, 150)                        * state.Time + primary.Acceleration * (0.5 * state.Time * state.Time);

            var angle   = phase + omega * state.Time;
            var radial  = new Vector(Math.Cos(angle), 0, Math.Sin(angle));
            var tangent = new Vector(-radial.Z, 0, radial.X);

            state.TargetPosition            = primary.Position     + radial  * 1000;
            state.TargetVelocity            = primary.Velocity     + tangent * 55;
            state.TargetAcceleration        = primary.Acceleration - radial  * 3.025;
            state.Obstacles[1].Position     = state.TargetPosition;
            state.Obstacles[1].Velocity     = state.TargetVelocity;
            state.Obstacles[1].Acceleration = state.TargetAcceleration;
            if (step == 0)
            {
                state.Position = state.TargetPosition + offset;
                state.Velocity = state.TargetVelocity + velocity;
            }

            var toPrimary = primary.Position - state.Position;
            state.ExternalAcceleration = primary.Acceleration + toPrimary.Unit * (3025000 / toPrimary.LengthSquared);
            var command = computer.Step(state);
            if (command.Phase == FlightPhase.Escape && !escaping)
            {
                escapes++;
            }

            escaping = command.Phase == FlightPhase.Escape;
            if (!command.Acceleration.IsFinite || command.Acceleration.Length > state.Thrust + 1e-7)
            {
                throw new Exception("Invalid moon approach thrust");
            }

            foreach (var body in state.Obstacles)
            {
                double gap = (state.Position - body.Position).Length - body.PhysicalRadius;
                minimum = Math.Min(minimum, gap);
                if (gap < 10)
                {
                    throw new Exception("Moon approach collided with a physical body");
                }
            }

            if (command.Phase == FlightPhase.Arrived)
            {
                arrived = true;
                break;
            }

            effort         += command.Acceleration.Length                         * state.DeltaTime;
            state.Velocity += (command.Acceleration + state.ExternalAcceleration) * state.DeltaTime;
            state.Position += state.Velocity                                      * state.DeltaTime;
        }

        var remaining = (state.Position - state.TargetPosition).Length - state.ArrivalRadius;

        Console.WriteLine($"MOON {name}: dt={timestep:F3}; arrived={arrived}; time={state.Time:F2}s; effort={effort:F1}; clearance={minimum:F1}m; escapes={escapes}; remaining={remaining:F1}m");

        var limit = name == "Long approach" ? 55 : 32;
        if (verify && (!arrived || state.Time > limit || (state.Velocity - state.TargetVelocity).Length > 0.5))
        {
            throw new Exception("Moon approach did not complete within its time and velocity limits: " + name);
        }

        var repeatedRecovery = name == "Recovered approach" && (escapes != 0 || effort > 500 || minimum < 240 || state.Time > 20 || Math.Abs(remaining) > 20);
        if (verify && repeatedRecovery)
        {
            throw new Exception("Recovered approach lost its smooth, clear arrival");
        }
    }
}
