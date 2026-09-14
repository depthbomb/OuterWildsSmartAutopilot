using Xunit;
using System;
using SmartAutopilot.Navigation;

internal static class OrbitalApproachTests
{
    public static void Verify()
    {
        Run(true);
    }

    public static void Prediction()
    {
        const double time = Math.PI / 0.1;

        var drift = new Vector(120, -30, 50);
        var translation = new Vector(1e7, -2e7, 3e7);
        var state = new FlightState();
        state.Obstacles.Add(new Obstacle
        {
            Position = translation,
            Velocity = drift,
            Radius   = 2500
        });
        state.Obstacles.Add(new Obstacle
        {
            PrimaryIndex = 0,
            Position     = translation + new Vector(5000, 0, 0),
            Velocity     = drift + new Vector(0, 0, 250),
            Acceleration = new Vector(-12.5, 0, 0),
            Radius       = 300
        });
        Assert.True(OrbitalMotion.TryCreate(state, 1, out var orbit), "Circular orbit was not recognized");
        Assert.True((orbit.Position(time) - translation - drift * time - new Vector(0, 0, 5000)).Length < 1e-6, "Orbit prediction failed after translation or a moving reference frame");
        Assert.True(orbit.ExcludesLinearPath(translation + new Vector(8000, 0, 0), drift, 400, 20), "Distant path was not outside the whole orbit");
        Assert.True(!orbit.ExcludesLinearPath(translation + new Vector(8000, 0, 0), drift + new Vector(-400, 0, 0), 400, 20), "Orbit envelope rejected an actual inward collision course");

        state.Obstacles[1].Velocity = drift + new Vector(20, 0, 250);

        Assert.True(OrbitalMotion.TryCreate(state, 1, out var eccentric), "Mild radial motion prevented bounded target prediction");
        Assert.True(!eccentric.ExcludesLinearPath(translation + new Vector(8000, 0, 0), drift, 400, 20), "An unstable orbit radius suppressed collision avoidance");

        state.Obstacles[1].Acceleration = new Vector(12.5, 0, 0);

        Assert.True(!OrbitalMotion.TryCreate(state, 1, out _), "Outward acceleration was mistaken for an orbit");

        state.Obstacles[1].PrimaryIndex = 50;
        Assert.True(!OrbitalMotion.TryCreate(state, 1, out _), "Invalid primary index was accepted");
    }

    public static void SatelliteTangent()
    {
        var speed = Math.Sqrt(75 * 2300);
        var state = new FlightState
        {
            Position             = new Vector(4906.14, 346.43, 2201.37),
            Velocity             = new Vector(-156.22, -23.07, speed - 108.30),
            ExternalAcceleration = new Vector(-14.88, -2.02, -3.76),
            TargetPosition       = new Vector(7000, 350, 2300),
            ArrivalRadius        = 1000,
            Thrust               = 50,
            MaximumThrust        = 50
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius         = 5246.7,
            PhysicalRadius = 2500
        });
        state.Obstacles.Add(new Obstacle
        {
            PrimaryIndex   = 0,
            Position       = new Vector(2300, 0, 0),
            Velocity       = new Vector(0, 0, speed),
            Acceleration   = new Vector(-75, 0, 0),
            Radius         = 307.8,
            PhysicalRadius = 30
        });
        var computer = new FlightComputer();
        state.Obstacles[1].PrimaryIndex = 99;
        Assert.True(new FlightComputer().Step(state).Phase == FlightPhase.Escape, "Invalid orbital context bypassed conservative avoidance");

        state.Obstacles[1].PrimaryIndex = 0;
        var command = computer.Step(state);
        Assert.True(command.Phase != FlightPhase.Escape, "A distant satellite tangent interrupted a clear approach outside its whole orbit");

        state.Position            = state.Obstacles[1].Position + new Vector(320, 0, 0);
        state.Velocity            = state.Obstacles[1].Velocity;
        state.Obstacles[0].Radius = 1000;

        Assert.True(new FlightComputer().Step(state).Phase == FlightPhase.Escape, "Orbit prediction suppressed an immediate nearby hazard");
    }

    public static void ForecastFallback()
    {
        var state = new FlightState
        {
            Position            = new Vector(-14000, 0, 0),
            TargetPosition      = new Vector(5000, 0, 0),
            TargetVelocity      = new Vector(0, 0, 280),
            TargetAcceleration  = new Vector(-15.68, 0, 0),
            TargetObstacleIndex = 1,
            ArrivalRadius       = 1000,
            Thrust              = 50,
            MaximumThrust       = 50
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius = 5246.7
        });
        state.Obstacles.Add(new Obstacle
        {
            Position     = state.TargetPosition,
            Velocity     = state.TargetVelocity,
            Acceleration = state.TargetAcceleration,
            PrimaryIndex = 0,
            Radius       = 350
        });
        state.Obstacles.Add(new Obstacle
        {
            Position = new Vector(Math.Cos(0.056 * 12), 0, Math.Sin(0.056 * 12)) * 5000,
            Radius   = 1300
        });
        Assert.True(new FlightComputer().Step(state).Phase == FlightPhase.Detour, "A blocked forecast discarded an available route to the current arrival region");
    }

    private static void Run(bool verify)
    {
        Flight("Far side", new Vector(-14000, 0, 0), default, 0, false, verify);
        Flight("Above orbit", new Vector(21000, 19600, 4500), new Vector(100, 0, 0), 0, false, verify);
        Flight("Logged Sun approach", new Vector(3075, 9646, -744), new Vector(-70, -556, 27), -0.59, false, verify);
        Flight("Binary twin", new Vector(-14000, 0, 0), default, 0, true, verify);
    }

    private static void Flight(string name, Vector start, Vector velocity, double phase, bool binary, bool verify)
    {
        const double gravityParameter = 4e8;
        const double orbitRadius      = 5000;

        var omega = Math.Sqrt(gravityParameter / (orbitRadius * orbitRadius * orbitRadius));
        var state = new FlightState
        {
            Position            = start,
            Velocity            = velocity,
            ArrivalRadius       = 1000,
            Thrust              = 50,
            MaximumThrust       = 50,
            TargetObstacleIndex = 1,
            DeltaTime           = 0.02
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius         = 5246.7,
            PhysicalRadius = 2500
        });
        state.Obstacles.Add(new Obstacle
        {
            PrimaryIndex   = 0,
            Radius         = 350,
            PhysicalRadius = 250
        });
        if (binary)
        {
            state.Obstacles.Add(new Obstacle
            {
                PrimaryIndex   = 0,
                Radius         = 450,
                PhysicalRadius = 300
            });
        }

        var computer           = new FlightComputer();
        var escapes            = 0;
        var lastPhase          = FlightPhase.Cruise;
        var minimumSunDistance = start.Length;
        var arrived            = false;
        for (int step = 0; step < 15000; step++)
        {
            state.Time = step * state.DeltaTime;

            var angle         = phase + omega * state.Time;
            var radial        = new Vector(Math.Cos(angle), 0, Math.Sin(angle));
            var tangent       = new Vector(-Math.Sin(angle), 0, Math.Cos(angle));
            var binaryRadial  = binary ? new Vector(Math.Cos(state.Time * 0.12), 0, Math.Sin(state.Time * 0.12)) : default;
            var binaryTangent = new Vector(-binaryRadial.Z, 0, binaryRadial.X);

            state.TargetPosition            = radial  * orbitRadius                    + binaryRadial  * 400;
            state.TargetVelocity            = tangent * (orbitRadius  * omega)         + binaryTangent * 48;
            state.TargetAcceleration        = radial  * (-orbitRadius * omega * omega) - binaryRadial  * 5.76;
            state.Obstacles[1].Position     = state.TargetPosition;
            state.Obstacles[1].Velocity     = state.TargetVelocity;
            state.Obstacles[1].Acceleration = state.TargetAcceleration;

            if (binary)
            {
                state.Obstacles[2].Position     = radial * orbitRadius - binaryRadial * 400;
                state.Obstacles[2].Velocity     = tangent * (orbitRadius * omega) - binaryTangent * 48;
                state.Obstacles[2].Acceleration = radial * (-orbitRadius * omega * omega) + binaryRadial * 5.76;
            }

            state.ExternalAcceleration = -state.Position.Unit * (gravityParameter / state.Position.LengthSquared);
            var command = computer.Step(state);
            if (!verify && name == "Logged Sun approach" && step % 500 == 0)
            {
                Console.WriteLine($"TRACE t={state.Time:F0}; phase={command.Phase}; remaining={(state.Position - state.TargetPosition).Length - state.ArrivalRadius:F0}; speed={(state.Velocity - state.TargetVelocity).Length:F0}; sun={state.Position.Length:F0}; detour={computer.DetourBodyIndex}");
            }

            if (command.Phase == FlightPhase.Escape && lastPhase != FlightPhase.Escape)
            {
                escapes++;
            }

            lastPhase = command.Phase;
            if (!command.Acceleration.IsFinite || command.Acceleration.Length > state.Thrust + 1e-7)
            {
                throw new Exception("Invalid thrust in orbital approach " + name);
            }

            if (command.Phase == FlightPhase.Arrived)
            {
                arrived = true;
                break;
            }

            state.Velocity += (command.Acceleration + state.ExternalAcceleration) * state.DeltaTime;
            state.Position += state.Velocity * state.DeltaTime;
            minimumSunDistance = Math.Min(minimumSunDistance, state.Position.Length);
            foreach (var obstacle in state.Obstacles)
            {
                if ((state.Position - obstacle.Position).Length < obstacle.PhysicalRadius + 10)
                {
                    throw new Exception("Physical collision in orbital approach " + name);
                }
            }

        }

        var remaining = (state.Position - state.TargetPosition).Length - state.ArrivalRadius;
        Console.WriteLine($"ORBIT {name}: arrived={arrived}; time={state.Time:F2}s; remaining={remaining:F1}m; escapes={escapes}; minSunDistance={minimumSunDistance:F1}m");

        if (verify && (!arrived || state.Time > 105 || escapes > 2 || remaining < -10 || remaining > 100))
        {
            throw new Exception("Orbital approach failed to finish reliably: " + name);
        }
    }
}
