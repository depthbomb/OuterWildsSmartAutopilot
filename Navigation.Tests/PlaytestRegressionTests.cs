using System;
using SmartAutopilot.Navigation;

internal static class PlaytestRegressionTests
{
    public static void BlockedCometHold()
    {
        var state = new FlightState
        {
            Position             = new Vector(6000, 0, 0),
            TargetPosition       = new Vector(4300, 0, 0),
            TargetVelocity       = new Vector(-400, 0, 0),
            ExternalAcceleration = new Vector(-10, 0, 0),
            ArrivalRadius        = 400,
            Thrust               = 50,
            MaximumThrust        = 50,
            DeltaTime            = 0.02
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius         = 5246.7,
            PhysicalRadius = 2500
        });
        var computer = new FlightComputer();
        var command  = computer.Step(state);
        Require(command.Phase == FlightPhase.Hold, "Enclosed comet destination was not held");
        Require(computer.RouteBlocked && computer.HoldBodyIndex == 0, "Blocked destination did not select the Sun as its waiting frame");
        Require((command.Acceleration + state.ExternalAcceleration).X >= -1e-7, "Hold accelerated toward the Sun to match the inaccessible comet");
        for (int step = 1; step <= 2000; step++)
        {
            state.Time = step * state.DeltaTime;
            command = computer.Step(state);
            state.Velocity += (command.Acceleration + state.ExternalAcceleration) * state.DeltaTime;
            state.Position += state.Velocity * state.DeltaTime;
            Require(command.Phase == FlightPhase.Hold && state.Position.X > 5900, "Waiting for the comet repeatedly re-entered emergency avoidance");
        }

        state.TargetPosition = new Vector(9000, 0, 0);
        state.TargetVelocity = default;
        state.Time += 1;
        Require(computer.Step(state).Phase == FlightPhase.Cruise, "Clear destination did not resume navigation after waiting");
        Require(!computer.RouteBlocked && computer.HoldBodyIndex == -1, "Resumed route retained its blocked episode");
    }

    public static void WarningAcrossEscape()
    {
        var state = new FlightState
        {
            Position       = new Vector(6000, 0, 0),
            TargetPosition = new Vector(4300, 0, 0),
            TargetVelocity = new Vector(-400, 0, 0),
            ArrivalRadius  = 400,
            Thrust         = 50
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius       = 5246.7,
            Velocity     = new Vector(40, 0, 0),
            Acceleration = new Vector(2, 0, 0)
        });
        state.Velocity = state.Obstacles[0].Velocity;
        var computer = new FlightComputer();
        var monitor  = new HoldMonitor();
        var command  = computer.Step(state);
        Require(command.Phase == FlightPhase.Hold && Math.Abs(command.Acceleration.X - 2) < 1e-7, "Hold did not follow the obstacle's accelerating frame");
        monitor.Step(command.Phase == FlightPhase.Hold, 0, 1);
        state.Position = new Vector(5260, 0, 0);
        state.Time     = 21;
        command = computer.Step(state);
        Require(command.Phase == FlightPhase.Escape && computer.RouteBlocked, "Emergency clearance discarded the unresolved route failure");
        monitor.Step(command.Phase == FlightPhase.Hold || (computer.RouteBlocked && command.Phase == FlightPhase.Escape), state.Time, 0.99);
        Require(monitor.NeedsAttention && monitor.Duration == 21, "Escape reset the blocked-route warning timer");
    }

    public static void AvoidancePriority()
    {
        var state = new FlightState
        {
            Position       = new Vector(-1000, 0, 0),
            Velocity       = new Vector(50, 0, 0),
            TargetPosition = new Vector(5000, 0, 0),
            ArrivalRadius  = 300,
            Thrust         = 30
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius = 950
        });
        state.Obstacles.Add(new Obstacle
        {
            Radius = 949
        });
        var computer = new FlightComputer();
        computer.Step(state);
        Require(computer.AvoidanceBodyIndex == 0, "Initial priority was wrong");
        state.Obstacles[1].Radius = 951;
        state.Time = 0.02;
        computer.Step(state);
        Require(computer.AvoidanceBodyIndex == 0, "Nearly equal obstacles switched avoidance every physics step");
        state.Obstacles[1].Radius = 980;
        state.Time = 0.04;
        computer.Step(state);
        Require(computer.AvoidanceBodyIndex == 1, "Priority stability delayed an immediate nearby threat");
    }

    public static void IrregularHullEnvelope()
    {
        double centered = HullEnvelope.BoxRadius(default, new Vector(600, 0, 0), new Vector(0, 80, 0), new Vector(0, 0, 600));
        Require(Math.Abs(centered - Math.Sqrt(600 * 600 * 2 + 80 * 80)) < 1e-7, "Hull envelope missed a box corner");
        double rotated = HullEnvelope.BoxRadius(new Vector(0, 0, 800), new Vector(60, 80, 0), new Vector(-160, 120, 0), new Vector(0, 0, 300));
        Require(Math.Abs(rotated - Math.Sqrt(100 * 100 + 200 * 200 + 1100 * 1100)) < 1e-7,
            "Offset, rotation or scale left a hull corner outside the navigation envelope");

        var obstacles = new[]
        {
            new Obstacle
            {
                PhysicalRadius = rotated,
                Radius         = rotated + 100
            }
        };
        var planner = new RoutePlanner();
        var start   = new Vector(-5000, 0, 800);
        var target  = new Vector(5000, 0, 800);
        Require(!RoutePlanner.SegmentClear(start, target, obstacles), "Direct route through the extended hull was accepted");
        Require(planner.TryPlan(start, target, obstacles, default, out var next), "No detour around an irregular hull envelope");
        Require(next.ObstacleIndex == 0 && RoutePlanner.SegmentClear(start, next.Position, obstacles), "First detour leg crosses the envelope");
        Require(planner.TryPlanToTarget(start, default, rotated + 180, obstacles, default, out var arrival, out _, out _, out _),
            "Expanded target envelope prevented an exterior arrival");
        Require(arrival.Length > rotated + 100, "Arrival was placed inside the expanded hull clearance");
    }

    public static void IrregularHullFlight()
    {
        foreach (double initialSpeed in new[] { 0.0, 500.0, 800.0 })
        {
            var state = new FlightState
            {
                Position       = new Vector(-12000, 0, 800),
                Velocity       = new Vector(initialSpeed, 0, 0),
                TargetPosition = new Vector(12000, 0, 800),
                ArrivalRadius  = 1000,
                Thrust         = 50,
                MaximumThrust  = 50,
                DeltaTime      = 0.02
            };
            state.Obstacles.Add(new Obstacle
            {
                PhysicalRadius = 1200,
                Radius         = 1300
            });
            var computer = new FlightComputer();
            bool arrived = false;
            for (int step = 0; step < 15000; step++)
            {
                state.Time = step * state.DeltaTime;
                var command = computer.Step(state);
                state.Velocity += command.Acceleration * state.DeltaTime;
                state.Position += state.Velocity * state.DeltaTime;
                Require(state.Position.Length > 1200, "Detour entered the hull at starting speed " + initialSpeed);
                Require(command.Phase != FlightPhase.Hold, "Clear exterior detour stalled in a route hold");
                if (command.Phase == FlightPhase.Arrived)
                {
                    arrived = true;
                    break;
                }
            }

            Require(arrived, "Exterior detour never reached its destination at starting speed " + initialSpeed);
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
