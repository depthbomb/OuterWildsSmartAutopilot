using System;

namespace SmartAutopilot.Navigation;

internal enum DeparturePhase
{
    Checking,
    Ignition,
    LiftOff,
    Climb,
    Blocked,
    Complete,
    Failed
}

internal sealed class DepartureState
{
    public Vector Up;
    public Vector Offset;
    public Vector Velocity;
    public Vector SurfaceVelocity;
    public Vector SurfaceAcceleration;
    public Vector ExternalAcceleration;
    public double Thrust;
    public double Time;
    public double ClearanceAltitude;
    public bool   Grounded;
    public bool   PathClear;
    public bool   IgnitionRequired;

    public double Altitude             => Vector.Dot(Offset, Up);
    public double RequiredAcceleration => (SurfaceAcceleration - ExternalAcceleration).Length;
    public double ThrustReserve        => Thrust - RequiredAcceleration;
}

internal sealed class DepartureComputer
{
    public DeparturePhase Phase         { get; private set; }
    public string         FailureReason { get; private set; } = string.Empty;
    public bool           CanSuspend    => Phase is DeparturePhase.LiftOff or DeparturePhase.Climb or DeparturePhase.Blocked;

    private double _ignitionStart = -1;
    private double _takeoffStart  = -1;
    private double _lastProgress;
    private double _bestAltitude;
    private double _blockedProbeDistance;
    private bool   _engineReady;

    private readonly double _ignitionDuration;

    public DepartureComputer(double ignitionDuration)
    {
        _ignitionDuration = Math.Max(0.1, ignitionDuration);
    }

    public Vector Step(DepartureState state)
    {
        if (Phase == DeparturePhase.Failed || Phase == DeparturePhase.Complete)
        {
            return default;
        }

        if (!state.Up.IsFinite || state.Up.LengthSquared < 0.9)
        {
            Phase         = DeparturePhase.Failed;
            FailureReason = "Unable to determine a safe climb direction";

            return default;
        }

        double reserve = state.ThrustReserve;
        if (reserve < 2)
        {
            Phase         = DeparturePhase.Failed;
            FailureReason = "Insufficient thrust for a controlled climb";

            return default;
        }

        if (!state.PathClear)
        {
            Phase                 = DeparturePhase.Blocked;
            _ignitionStart        = -1;
            _lastProgress         = state.Time;
            _blockedProbeDistance = GetProbeDistance(state);
            if (state.Grounded && _takeoffStart >= 0)
            {
                _takeoffStart = state.Time;
            }

            return state.Grounded ? default : Control(state, 0);
        }

        _blockedProbeDistance = 0;
        if (!_engineReady && state.IgnitionRequired)
        {
            if (_ignitionStart < 0)
            {
                _ignitionStart = state.Time;
            }

            if (state.Time - _ignitionStart < _ignitionDuration)
            {
                Phase = DeparturePhase.Ignition;

                return default;
            }
        }

        _engineReady = true;
        if (_takeoffStart < 0)
        {
            _takeoffStart = state.Time;
            _lastProgress = state.Time;
            _bestAltitude = state.Altitude;
        }

        if (state.Altitude > _bestAltitude + 2)
        {
            _bestAltitude = state.Altitude;
            _lastProgress = state.Time;
        }

        var stalled = state.Time - _lastProgress > 12 || (state.Grounded && state.Time - _takeoffStart > 8);
        if (stalled)
        {
            Phase         = DeparturePhase.Failed;
            FailureReason = "Unable to lift clear of the surface";

            return default;
        }

        if (!state.Grounded && state.Altitude >= state.ClearanceAltitude)
        {
            Phase = DeparturePhase.Complete;

            return default;
        }

        Phase = state.Altitude < 25 ? DeparturePhase.LiftOff : DeparturePhase.Climb;

        var speed = Phase == DeparturePhase.LiftOff ? 20 : Math.Min(80, Math.Sqrt(2 * reserve * Math.Max(30, state.ClearanceAltitude - state.Altitude)));

        return Control(state, speed);
    }

    public double GetProbeDistance(DepartureState state)
    {
        var speed   = Math.Max(0, Vector.Dot(state.Velocity - state.SurfaceVelocity, state.Up));
        var reserve = Math.Max(1, state.ThrustReserve);

        return Math.Max(_blockedProbeDistance, Math.Max(30, speed * speed / (2 * reserve) + speed * 1.5 + 15));
    }

    public bool TryResume(DepartureState state, double suspendedAt)
    {
        var altitude        = state.Altitude;
        var lateralOffset   = state.Offset   - state.Up * altitude;
        var velocity        = state.Velocity - state.SurfaceVelocity;
        var lateralVelocity = velocity       - state.Up * Vector.Dot(velocity, state.Up);
        var nearby = CanSuspend && !state.Grounded && state.Time >= suspendedAt                                && state.Time - suspendedAt <= 15
                     && altitude                                 >= -5                                         && altitude                 < state.ClearanceAltitude
                     && lateralOffset.Length                     <= Math.Max(25, Math.Max(0, altitude) * 0.25) && lateralVelocity.Length   < 20;
        if (!nearby)
        {
            return false;
        }

        // Recheck clearance and progress from the current surface-relative position after an explicit re-engagement.
        Phase                 = DeparturePhase.Checking;
        _ignitionStart        = -1;
        _takeoffStart         = -1;
        _lastProgress         = state.Time;
        _bestAltitude         = altitude;
        _blockedProbeDistance = 0;
        _engineReady          = !state.IgnitionRequired;

        return true;
    }

    public static double ClearanceAltitude(Vector surfaceOffset, Vector up, double safeRadius)
    {
        var projection   = Vector.Dot(surfaceOffset, up);
        var discriminant = projection * projection + safeRadius * safeRadius - surfaceOffset.LengthSquared;

        return Math.Max(120, -projection + Math.Sqrt(Math.Max(0, discriminant)));
    }

    private static Vector Control(DepartureState state, double climbSpeed)
    {
        var lateralOffset   = state.Offset                                                         - state.Up * state.Altitude;
        var desiredVelocity = state.SurfaceVelocity + state.Up   * climbSpeed                      - (lateralOffset * 0.5).Limited(10);
        var acceleration    = (desiredVelocity - state.Velocity) / 0.8 + state.SurfaceAcceleration - state.ExternalAcceleration;

        return acceleration.Limited(state.Thrust);
    }
}
