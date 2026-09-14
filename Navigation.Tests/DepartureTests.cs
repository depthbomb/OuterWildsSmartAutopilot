using System;
using SmartAutopilot.Navigation;

internal static class DepartureTests
{
    public static void Launch(bool cold, double dt)
    {
        var state = State();
        state.IgnitionRequired = cold;
        state.SurfaceVelocity = new Vector(80, 15, -20);
        state.Velocity        = state.SurfaceVelocity;
        var computer = new DepartureComputer(1);
        bool lifted = false;
        bool climbed = false;
        for (int step = 0; step < 10000; step++)
        {
            state.Time = step * dt;
            var acceleration = computer.Step(state);
            Require(acceleration.IsFinite && acceleration.Length <= state.Thrust + 1e-7, "Departure exceeded real thrust");
            Require(computer.Phase != DeparturePhase.Failed && computer.Phase != DeparturePhase.Blocked, "Clear departure failed");
            if (cold && state.Time < 1)
            {
                Require(computer.Phase == DeparturePhase.Ignition && acceleration.Length == 0, "Cold engine applied thrust before ignition completed");
            }

            lifted  |= computer.Phase == DeparturePhase.LiftOff;
            climbed |= computer.Phase == DeparturePhase.Climb;
            if (computer.Phase == DeparturePhase.Complete)
            {
                Require(lifted && climbed && !state.Grounded && state.Altitude >= state.ClearanceAltitude, "Handoff happened before clearing the surface");
                var lateral = state.Offset - state.Up * state.Altitude;
                Require(lateral.Length < 0.01, "Departure drifted across the moving surface");

                return;
            }

            if (computer.Phase != DeparturePhase.Ignition)
            {
                state.Velocity += (acceleration + state.ExternalAcceleration) * dt;
                state.Offset   += (state.Velocity - state.SurfaceVelocity) * dt;
                state.Grounded  = state.Altitude < 0.01;
            }
        }

        throw new Exception("Launch did not complete");
    }

    public static void BlockedIgnition()
    {
        var state = State();
        var computer = new DepartureComputer(1);
        state.PathClear = false;
        Require(computer.Step(state).Length == 0 && computer.Phase == DeparturePhase.Blocked, "Obstructed launch ignited or applied thrust");
        state.PathClear = true;
        state.Time = 2;
        Require(computer.Step(state).Length == 0 && computer.Phase == DeparturePhase.Ignition, "Clearance did not start normal ignition");
        state.Time = 2.8;
        state.PathClear = false;
        Require(computer.Step(state).Length == 0 && computer.Phase == DeparturePhase.Blocked, "New obstruction did not interrupt ignition");
        state.Time = 4;
        state.PathClear = true;
        computer.Step(state);
        state.Time = 4.9;
        Require(computer.Step(state).Length == 0 && computer.Phase == DeparturePhase.Ignition, "Interrupted ignition resumed from a stale timer");
        state.Time = 5.1;
        Require(computer.Step(state).Length > 0 && computer.Phase == DeparturePhase.LiftOff, "Clear launch never resumed");
    }

    public static void ObstructionDuringClimb()
    {
        var state = State();
        state.IgnitionRequired = false;
        state.Grounded        = false;
        state.Offset          = state.Up * 100;
        state.Velocity        = state.Up * 70;
        state.PathClear       = false;
        var computer = new DepartureComputer(1);
        double originalProbe = computer.GetProbeDistance(state);
        for (int step = 0; step < 400; step++)
        {
            state.Time = step * 0.02;
            var acceleration = computer.Step(state);
            Require(computer.Phase == DeparturePhase.Blocked, "Obstructed climb continued");
            Require(acceleration.Length <= state.Thrust + 1e-7, "Obstruction braking exceeded thrust");
            Require(computer.GetProbeDistance(state) >= originalProbe, "Slowing shrank the obstruction check and could cause repeated restarts");
            state.Velocity += (acceleration + state.ExternalAcceleration) * 0.02;
            state.Offset   += state.Velocity * 0.02;
        }

        Require(state.Velocity.Length < 0.1, "Blocked climb did not hold velocity");
        Require(state.Altitude - 100 < originalProbe - 10, "Climb braking used more than the checked clearance");
        state.PathClear = true;
        state.ClearanceAltitude = state.Altitude + 200;
        Require(computer.Step(state).Length > 0 && computer.Phase == DeparturePhase.Climb, "Climb failed to resume after obstruction cleared");
    }

    public static void ThrustAndProgressLimits()
    {
        var state = State();
        state.Thrust = 10;
        var weak = new DepartureComputer(1);
        Require(weak.Step(state).Length == 0 && weak.Phase == DeparturePhase.Failed, "Launch ignored insufficient thrust against gravity");
        Require(state.ThrustReserve < 2 && weak.FailureReason == "Insufficient thrust for a controlled climb", "Low-reserve cancellation lost its diagnostic reason");

        state = State();
        state.Up = default;
        var invalidDirection = new DepartureComputer(1);
        Require(invalidDirection.Step(state).Length == 0 && invalidDirection.Phase == DeparturePhase.Failed
            && invalidDirection.FailureReason == "Unable to determine a safe climb direction", "Invalid orientation was reported as insufficient thrust");

        state = State();
        state.SurfaceAcceleration = new Vector(40, -20, 10);
        state.ExternalAcceleration = new Vector(43, -24, 10);
        Require(state.RequiredAcceleration == 5 && state.ThrustReserve == state.Thrust - 5, "Departure reserve used absolute gravity instead of surface-relative acceleration");

        state = State();
        state.IgnitionRequired = false;
        var stuck = new DepartureComputer(1);
        stuck.Step(state);
        state.Time = 9;
        Require(stuck.Step(state).Length == 0 && stuck.Phase == DeparturePhase.Failed, "Stuck grounded ship kept applying thrust");

        state = State();
        state.IgnitionRequired = false;
        state.Thrust = 20;
        var limited = new DepartureComputer(1);
        Require(limited.Step(state).Length <= 20 + 1e-7 && limited.Phase == DeparturePhase.LiftOff, "Local thrust limit was bypassed or prevented a feasible lift");
    }

    public static void CancelAndRestart()
    {
        var state = State();
        var original = new DepartureComputer(1);
        original.Step(state);
        state.Time = 0.8;
        original.Step(state);
        // The adapter discards the computer on abort. A new activation must get a full ignition interval.
        var restarted = new DepartureComputer(1);
        state.Time = 4;
        Require(restarted.Step(state).Length == 0 && restarted.Phase == DeparturePhase.Ignition, "Re-engagement reused cancelled launch state");
        state.Time = 4.9;
        Require(restarted.Step(state).Length == 0, "Re-engagement skipped the ignition interval");
    }

    public static void ClearanceGeometry()
    {
        var origin = new Vector(1000, 0, 0);
        var up = new Vector(1, 1, 0).Unit;
        double altitude = DepartureComputer.ClearanceAltitude(origin, up, 1500);
        Require(Math.Abs((origin + up * altitude).Length - 1500) < 1e-8, "Tilted climb does not exit the departure clearance sphere");
        Require(DepartureComputer.ClearanceAltitude(new Vector(2000, 0, 0), new Vector(1, 0, 0), 1500) == 120, "An elevated platform should still get a minimum vertical climb");
    }

    public static void NavigationHandoff()
    {
        var state = State();
        state.IgnitionRequired = false;
        state.Up = new Vector(-1, 0, 0);
        state.ExternalAcceleration = new Vector(12, 0, 0);
        state.ClearanceAltitude = DepartureComputer.ClearanceAltitude(new Vector(-600, 0, 0), state.Up, 950);
        var departure = new DepartureComputer(1);
        for (int step = 0; step < 3000; step++)
        {
            state.Time = step * 0.02;
            var acceleration = departure.Step(state);
            if (departure.Phase == DeparturePhase.Complete)
            {
                var flight = new FlightState
                {
                    Position       = new Vector(-600, 0, 0) + state.Offset,
                    Velocity       = state.Velocity,
                    TargetPosition = new Vector(3500, 0, 0),
                    ArrivalRadius  = 300,
                    Thrust         = state.Thrust
                };
                flight.Obstacles.Add(new Obstacle
                {
                    Radius         = 800,
                    PhysicalRadius = 500
                });
                var command = new FlightComputer().Step(flight);
                Require(command.Phase == FlightPhase.Detour && command.Acceleration.IsFinite, "Launch failed to hand off to the required detour");
                Require(flight.Position.Length >= 950, "Normal navigation started inside departure clearance");

                return;
            }

            state.Velocity += (acceleration + state.ExternalAcceleration) * 0.02;
            state.Offset   += state.Velocity * 0.02;
            state.Grounded  = state.Altitude < 0.01;
        }

        throw new Exception("Launch did not hand off");
    }

    public static void ResumeInterruptedClimb()
    {
        var state = State();
        state.IgnitionRequired = false;
        state.Grounded        = false;
        state.Offset          = state.Up * 80;
        state.Velocity        = state.Up * 25;
        state.Time            = 2;
        var computer = new DepartureComputer(1);
        computer.Step(state);
        Require(computer.CanSuspend, "An airborne climb could not be suspended");

        // Time spent under manual control must not consume the resumed climb's progress timeout.
        state.Time     = 16;
        state.Offset   = state.Up * 90;
        state.Velocity = state.Up * 8;
        Require(computer.TryResume(state, 2), "Nearby interrupted climb did not resume");
        for (int step = 0; step < 2000; step++)
        {
            var acceleration = computer.Step(state);
            Require(computer.Phase != DeparturePhase.Ignition && computer.Phase != DeparturePhase.Failed, "Warm resumed climb re-ignited or used stale progress timing");
            Require(acceleration.IsFinite && acceleration.Length <= state.Thrust + 1e-7, "Resumed climb exceeded thrust");
            if (computer.Phase == DeparturePhase.Complete)
            {
                Require(state.Altitude >= state.ClearanceAltitude, "Resumed climb handed off before clearing the surface");

                return;
            }

            state.Velocity += (acceleration + state.ExternalAcceleration) * 0.02;
            state.Offset   += state.Velocity * 0.02;
            state.Time     += 0.02;
        }

        throw new Exception("Resumed departure did not complete");
    }

    public static void RejectUnrelatedResume()
    {
        var state = State();
        state.IgnitionRequired = false;
        state.Grounded        = false;
        state.Offset          = state.Up * 80;
        var computer = new DepartureComputer(1);
        computer.Step(state);
        state.Time = 16;
        Require(!computer.TryResume(state, 0), "Expired launch context was reused");
        state.Time = 4;
        state.Offset = state.Up * 400;
        Require(!computer.TryResume(state, 0), "Already-clear ship was sent back into a departure climb");
        state.Offset = state.Up * 80 + new Vector(1000, 0, 0);
        Require(!computer.TryResume(state, 0), "A distant manual flight reused the launch column");
        state.Offset = state.Up * 80;
        state.Velocity = new Vector(80, 0, 0);
        Require(!computer.TryResume(state, 0), "Fast sideways flight snapped back to the launch column");
        state.Velocity = default;
        state.Grounded = true;
        Require(!computer.TryResume(state, 0), "A new landing resumed an old airborne climb");
        state.Grounded = false;
        Require(!new DepartureComputer(1).TryResume(state, 0), "A fresh scene/computer inherited a previous departure");
    }

    public static void ResumeRechecksObstruction()
    {
        var state = State();
        state.Grounded        = false;
        state.IgnitionRequired = false;
        state.Offset          = state.Up * 80;
        var computer = new DepartureComputer(1);
        computer.Step(state);
        state.Time = 2;
        Require(computer.TryResume(state, 0), "Fixture did not resume");
        state.PathClear = false;
        var acceleration = computer.Step(state);
        Require(computer.Phase == DeparturePhase.Blocked, "Resumed climb ignored a new overhead obstacle");
        Require((acceleration + state.ExternalAcceleration).Length < 1e-7, "Blocked resumed climb failed to counter gravity");
    }

    private static DepartureState State()
    {
        var up = new Vector(0.1, 1, 0.2).Unit;

        return new DepartureState
        {
            Up                   = up,
            ExternalAcceleration = -up * 12,
            SurfaceAcceleration  = default,
            Thrust               = 30,
            ClearanceAltitude    = 350,
            Grounded             = true,
            PathClear            = true,
            IgnitionRequired     = true
        };
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new Exception(message);
        }
    }
}
