using System;
using System.Collections.Generic;
using SmartAutopilot.Navigation;

internal static class BetaSafetyTests
{
    public static void DetourStatus()
    {
        var state = new FlightState
        {
            Position       = new Vector(-3500, 0, 0),
            TargetPosition = new Vector(3500, 0, 0),
            ArrivalRadius  = 300,
            Thrust         = 30
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius = 700
        });
        var computer = new FlightComputer();
        var command  = computer.Step(state);
        Require(command.Phase == FlightPhase.Detour && computer.DetourBodyIndex == 0, "Planned diversion did not identify its obstacle");
        Require(command.Acceleration.IsFinite && command.Acceleration.Length > 0, "Detour lost steering thrust");
        var points = new Vector[4];
        int count = computer.GetDebugRoute(state, points);
        Require(count == 4 && (points[0] - state.Position).Length < 1e-7
            && RoutePlanner.SegmentClear(points[0], points[1], state.Obstacles), "Debug overlay did not receive the actual planned route");
        Require(computer.GetDebugRoute(state, new Vector[1]) == 0, "Debug route overflowed a small buffer");
        state.Obstacles[0].Radius = 0;
        state.Time = 1;
        command = computer.Step(state);
        Require(command.Phase == FlightPhase.Cruise && computer.DetourBodyIndex == -1, "Clear route retained a stale diversion");
    }

    public static void CollisionStop()
    {
        foreach (double dt in new[] { 0.02, 1.0 / 60, 0.05 })
        {
            foreach (double thrust in new[] { 10.0, 30.0, 50.0 })
            {
                var state = new FlightState
                {
                    Velocity             = new Vector(180, 30, 0),
                    ExternalAcceleration = new Vector(thrust * 0.2, 0, 0),
                    Thrust               = thrust,
                    DeltaTime            = dt
                };
                double obstacleDistance = CollisionBraking.ProbeDistance(state, default, default);
                var heading = state.Velocity.Unit;
                for (int step = 0; step < 5000; step++)
                {
                    var command = CollisionBraking.Stop(state, default, default);
                    Require(command.Phase == FlightPhase.Hold && command.Acceleration.IsFinite
                        && command.Acceleration.Length <= thrust + 1e-7, "Collision braking exceeded available control");
                    state.Velocity += (command.Acceleration + state.ExternalAcceleration) * dt;
                    state.Position += state.Velocity * dt;
                    Require(Vector.Dot(state.Position, heading) < obstacleDistance - 10, "Ship reached obstacle before stopping");
                }

                Require(state.Velocity.Length < 0.01, "Collision hold did not stop relative movement");
            }
        }
    }

    public static void MovingCollisionHold()
    {
        var state = new FlightState
        {
            Velocity             = new Vector(100, 50, -20),
            ExternalAcceleration = new Vector(0, -8, 0),
            Thrust               = 30,
            DeltaTime            = 0.02
        };
        var velocity     = new Vector(100, 50, -20);
        var acceleration = new Vector(2, 0, 0);
        for (int step = 0; step < 1000; step++)
        {
            var command = CollisionBraking.Stop(state, velocity, acceleration);
            state.Velocity += (command.Acceleration + state.ExternalAcceleration) * state.DeltaTime;
            velocity       += acceleration * state.DeltaTime;
            Require((state.Velocity - velocity).Length < 1e-7, "Hold drifted from a moving body");
        }

        double restingProbe = CollisionBraking.ProbeDistance(state, velocity, acceleration);
        Require(restingProbe >= 30, "Stationary ship did not probe ahead before accelerating");
        state.Thrust = 0;
        Require(CollisionBraking.Stop(state, velocity, acceleration).Acceleration.Length == 0, "Depleted ship generated thrust");
    }

    public static void FaultRecovery()
    {
        var faults = new FlightFaults();
        Require(faults.TryEngage(0), "Fresh flight was locked out");
        Require(faults.Record(1) && faults.Faulted, "Fault did not stop automatic control");
        Require(!faults.Record(2) && !faults.TryEngage(5.99), "Duplicate callback or early retry bypassed cooldown");
        Require(faults.TryEngage(6) && !faults.Faulted, "Explicit retry after cooldown was rejected");
        Require(faults.Record(7) && faults.TryEngage(12) && faults.Record(13), "Retry accounting failed");
        Require(faults.Locked && !faults.TryEngage(10000), "Repeated errors could run indefinitely");
        Require(new FlightFaults().TryEngage(0), "A new loop did not restore eligibility");
    }

    public static void HoldWarnings()
    {
        var monitor = new HoldMonitor();
        monitor.Step(true, 100, 1);
        monitor.Step(true, 119, 0.99);
        Require(!monitor.NeedsAttention, "Normal brief hold raised an alert");
        monitor.Step(true, 120, 0.99);
        Require(monitor.NeedsAttention && monitor.Duration == 20, "Persistent obstruction escaped its timer");
        monitor.Step(false, 121, 0.99);
        Require(!monitor.NeedsAttention && monitor.Duration == 0 && monitor.FuelUsed == 0, "Resumed flight retained an alert");
        monitor.Step(true, 122, 0.9);
        monitor.Step(true, 123, 0.84);
        Require(monitor.NeedsAttention, "Fuel consumption did not trigger early intervention");
        monitor.Step(false, 124, 0.14);
        monitor.Step(true, 125, 0.14);
        Require(monitor.NeedsAttention, "Low fuel was ignored during a hold");
        monitor.Step(true, 0, 1);
        Require(!monitor.NeedsAttention && monitor.Duration == 0, "Rewound clock retained stale hold state");
    }

    public static void FlightHistory()
    {
        var recorder = new FlightRecorder();
        var state    = new FlightState();
        for (int i = 0; i < 150; i++)
        {
            state.Time = i * 0.5;
            recorder.Add(new FlightSample(state, default, 1, "Cruise"));
        }

        var output = new List<string>();
        recorder.Dump(output.Add);
        Require(recorder.Count == 120 && output.Count == 120, "Diagnostics exceeded bounded storage");
        Require(output[0].StartsWith("t=15.00s") && output[119].StartsWith("t=74.50s"), "Diagnostics lost chronological order");
        recorder.Clear();
        Require(recorder.Count == 0, "New flight retained previous samples");
    }

    public static void InvalidState()
    {
        var state = new FlightState
        {
            Thrust        = 30,
            MaximumThrust = 30
        };
        FlightValidation.Validate(state);
        foreach (Action corrupt in new Action[]
        {
            () => state.Position = new Vector(double.NaN, 0, 0),
            () => state.MaximumThrust = 0,
            () => state.DeltaTime = 0,
            () => state.ArrivalRadius = double.PositiveInfinity,
            () => state.Obstacles.Add(new Obstacle
            {
                Radius = double.NaN
            })
        })
        {
            state.Position      = default;
            state.MaximumThrust = 30;
            state.DeltaTime     = 0.02;
            state.ArrivalRadius = 300;
            state.Obstacles.Clear();
            corrupt();
            bool rejected = false;
            try
            {
                FlightValidation.Validate(state);
            }
            catch (InvalidOperationException)
            {
                rejected = true;
            }

            Require(rejected, "Invalid game state reached native collision processing");
        }
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new Exception(message);
        }
    }
}
