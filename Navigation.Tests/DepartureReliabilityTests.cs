using System;
using SmartAutopilot.Navigation;

internal static class DepartureReliabilityTests
{
    public static void SurfaceMatrix()
    {
        // Synthetic environments span small moons, tilted terrain, strong gravity and rotating bodies.
        // These are controller tests, not measurements of individual planets or Unity terrain queries.
        foreach (double dt in new[] { 0.02, 1.0 / 60, 0.05 })
        {
            Fly(180, 4, 30, 0, 0, dt);
            Fly(600, 12, 30, 0.015, 0, dt);
            Fly(1000, 25, 50, 0.01, 0.6, dt);
            Fly(300, 6, 20, 0.025, 0.8, dt);
        }
    }

    public static void ResumeDuringDescent()
    {
        var state = new DepartureState
        {
            Up                   = new Vector(0, 1, 0),
            Offset               = new Vector(0, 120, 0),
            ExternalAcceleration = new Vector(0, -12, 0),
            Thrust               = 30,
            ClearanceAltitude    = 400,
            PathClear            = true
        };
        var computer = new DepartureComputer(1);
        computer.Step(state);
        state.Time     = 3;
        state.Velocity = new Vector(0, -30, 0);
        Require(computer.TryResume(state, 0), "Nearby descending ship could not resume");
        for (int step = 0; step < 4000; step++)
        {
            var thrust = computer.Step(state);
            Require(thrust.IsFinite && thrust.Length <= state.Thrust + 1e-7, "Descent recovery exceeded thrust");
            Require(state.Altitude > 60, "Resumed descent consumed more than its stopping clearance");
            Require(computer.Phase != DeparturePhase.Failed, "Resumed descent failed to recover");
            if (computer.Phase == DeparturePhase.Complete)
            {
                return;
            }

            state.Velocity += (thrust + state.ExternalAcceleration) * 0.02;
            state.Offset   += state.Velocity * 0.02;
            state.Time     += 0.02;
        }

        throw new Exception("Resumed descending launch never completed");
    }

    public static void AirborneStall()
    {
        var state = new DepartureState
        {
            Up                = new Vector(0, 1, 0),
            Offset            = new Vector(0, 80, 0),
            Thrust            = 30,
            ClearanceAltitude = 400,
            PathClear         = true
        };
        var computer = new DepartureComputer(1);
        computer.Step(state);
        state.Time = 13;
        Require(computer.Step(state).Length == 0 && computer.Phase == DeparturePhase.Failed,
            "An airborne ship trapped without progress kept applying thrust");
        state.Time   = 14;
        state.Offset = new Vector(0, 150, 0);
        Require(computer.Step(state).Length == 0 && !computer.TryResume(state, 13), "Failed launch restarted itself");
    }

    public static void RotatingResume()
    {
        var state = new DepartureState
        {
            Up                = new Vector(0, 1, 0),
            Offset            = new Vector(0, 80, 0),
            Thrust            = 30,
            ClearanceAltitude = 400,
            PathClear         = true
        };
        var computer = new DepartureComputer(1);
        computer.Step(state);
        state.Time            = 5;
        state.Up              = new Vector(1, 0, 0);
        state.Offset          = new Vector(85, 0, 0);
        state.SurfaceVelocity = new Vector(150, 30, -80);
        state.Velocity        = state.SurfaceVelocity + state.Up * 10;
        Require(computer.TryResume(state, 0), "Resume used world speed or the old surface orientation");
        Require(computer.Step(state).X > 0 && computer.Phase == DeparturePhase.Climb, "Resumed climb did not follow the current surface orientation");
        state.Time = -1;
        Require(!computer.TryResume(state, 0), "Rewound scene time reused old launch context");
    }

    private static void Fly(double radius, double surfaceGravity, double thrustLimit, double rotationRate, double tilt, double dt)
    {
        var computer = new DepartureComputer(1);
        var translation = new Vector(80, -15, 30);
        var bodyAcceleration = new Vector(0.6, -0.2, 0.1);
        var position = new Vector(radius, 0, 0);
        var velocity = translation + Spin(position, rotationRate);
        var state = new DepartureState
        {
            Thrust           = thrustLimit,
            PathClear        = true,
            IgnitionRequired = true,
            Grounded         = true
        };
        double maxLateral = 0;
        for (int step = 0; step < 120 / dt; step++)
        {
            double time  = step * dt;
            double angle = rotationRate * time;
            var radial   = new Vector(Math.Cos(angle), Math.Sin(angle), 0);
            var up       = new Vector(Math.Cos(angle + tilt), Math.Sin(angle + tilt), 0);
            var center   = translation * time + bodyAcceleration * (0.5 * time * time);
            var bodyVelocity = translation + bodyAcceleration * time;
            var origin   = center + radial * radius;
            if (state.Grounded)
            {
                position = origin;
                velocity = bodyVelocity + Spin(radial * radius, rotationRate);
            }

            var bodyOffset = position - center;
            state.Up                   = up;
            state.Offset               = position - origin;
            state.Velocity             = velocity;
            state.SurfaceVelocity      = bodyVelocity + Spin(bodyOffset, rotationRate);
            state.SurfaceAcceleration  = bodyAcceleration + Spin(Spin(bodyOffset, rotationRate), rotationRate);
            state.ExternalAcceleration = bodyAcceleration - bodyOffset.Unit * (surfaceGravity * radius * radius / bodyOffset.LengthSquared);
            state.ClearanceAltitude    = DepartureComputer.ClearanceAltitude(radial * radius, up, radius + 350);
            state.Time                 = time;
            var command = computer.Step(state);
            Require(command.IsFinite && command.Length <= thrustLimit + 1e-7, "Surface matrix exceeded available thrust");
            Require(computer.Phase != DeparturePhase.Failed && computer.Phase != DeparturePhase.Blocked, "Feasible open surface launch failed");
            maxLateral = Math.Max(maxLateral, (state.Offset - up * state.Altitude).Length);
            if (computer.Phase == DeparturePhase.Complete)
            {
                Require(bodyOffset.Length >= radius + 350 - 0.1, "Rotating or tilted launch handed off inside the clearance sphere");
                Require(maxLateral < 15, "Launch drifted too far from its rotating departure column");
                Console.WriteLine($"  Surface simulation: radius={radius:F0}m gravity={surfaceGravity:F0} thrust={thrustLimit:F0} rotation={rotationRate:F3} tilt={tilt:F1} dt={dt:F3}; complete={time:F2}s lateral={maxLateral:F2}m");

                return;
            }

            if (computer.Phase == DeparturePhase.Ignition)
            {
                continue;
            }

            velocity      += (command + state.ExternalAcceleration) * dt;
            position      += velocity * dt;
            state.Grounded = false;
            Require((position - center).Length >= radius - 1, "Launch intersected the modeled surface");
        }

        throw new Exception("Surface matrix launch did not complete");
    }

    private static Vector Spin(Vector offset, double rate) => new(-rate * offset.Y, rate * offset.X, 0);

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new Exception(message);
        }
    }
}
