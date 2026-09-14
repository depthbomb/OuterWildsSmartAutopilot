using SmartAutopilot.Navigation;
using UnityEngine;
using Vector = SmartAutopilot.Navigation.Vector;

namespace SmartAutopilot;

internal sealed class ShipCollisionGuard
{
    public bool   Active             { get; private set; }
    public bool   Recovering         { get; private set; }
    public bool   NeedsManualControl { get; private set; }
    public string ReferenceName      { get; private set; } = "local space";
    public string Reason             => _hitDistance > 0 ? $"{_reason} ({_hitDistance:F0}m)" : _reason;
    public string Diagnostic => Reason          + "; body=" + (_holdBody != null ? _holdBody.name : "untracked body")
                                + "; collider=" + (_holdCollider         != null ? _holdCollider.name : "unresolved collider") + ScanDiagnostic();
    public DebugSweep DebugVelocityProbe => Recovering ? default : _velocityProbe;
    public DebugSweep DebugSteeringProbe => Recovering ? default : _steeringProbe;

    private OWRigidbody                _holdBody;
    private bool                       _hadHoldBody;
    private DebugSweep                 _velocityProbe;
    private DebugSweep                 _steeringProbe;
    private Bounds                     _recoveryBounds;
    private IReadOnlyList<AstroObject> _bodies;
    private double                     _hitDistance;
    private string                     _reason;
    private Collider                   _holdCollider;
    private OWRigidbody                _scanBody;
    private CollisionFrame             _scanFrame;
    private double                     _scanSpeed;
    private double                     _scanReserve;

    private readonly OWRigidbody                        _ship;
    private readonly OWRigidbody                        _player;
    private readonly Collider[]                         _hull;
    private readonly RaycastHit[]                       _hits     = new RaycastHit[64];
    private readonly Collider[]                         _overlaps = new Collider[64];
    private readonly Func<Vector, Vector, double, bool> _segmentClear;
    private readonly CollisionHoldRelease               _release = new();

    public ShipCollisionGuard(OWRigidbody ship)
    {
        _ship         = ship;
        _player       = Locator.GetPlayerBody();
        _hull         = ship.GetComponentsInChildren<Collider>();
        _segmentClear = RecoverySegmentClear;
    }

    public FlightCommand Step(FlightState state, FlightCommand command, IReadOnlyList<AstroObject> bodies, int dangerIndex)
    {
        _bodies            = bodies;
        NeedsManualControl = false;
        var position = _ship.GetWorldCenterOfMass();
        if ((Active || Recovering) && _hadHoldBody && _holdBody == null)
        {
            throw new InvalidOperationException("The collision hold reference disappeared. Take manual control.");
        }

        if (Recovering && command.Phase == FlightPhase.Escape)
        {
            return Recover(state, command, dangerIndex, position);
        }

        Recovering = false;

        var wasHolding = Active;

        Active = false;

        Scan(state, command, position);

        var obstructed  = Active;
        var canNavigate = command.Phase != FlightPhase.Hold && command.Phase != FlightPhase.Arrived;
        var distantHit  = obstructed                        && _hitDistance  > 500 && canNavigate && command.Phase != FlightPhase.Escape;
        if (distantHit)
        {
            _recoveryBounds = HullBounds(position);
            var frame = ReadFrame(state, _holdBody);
            var clear = CollisionEscape.PathClear(state, command.Acceleration, frame.Velocity, frame.Acceleration, _segmentClear);
            obstructed = !clear;
        }

        if (_release.Step(!obstructed, wasHolding, state.Time))
        {
            Active = false;

            return command;
        }

        Active = true;
        if (command.Phase == FlightPhase.Escape)
        {
            return Recover(state, command, dangerIndex, position);
        }

        var holdFrame = ReadFrame(state, _holdBody);

        return CollisionBraking.Stop(state, holdFrame.Velocity, holdFrame.Acceleration);
    }

    private void Scan(FlightState state, FlightCommand command, Vector3 position)
    {
        _hitDistance = 0;

        var scanBody = default(OWRigidbody);
        var nearest  = double.PositiveInfinity;
        for (int i = 0; i < _bodies.Count; i++)
        {
            var usable = _bodies[i] != null && state.Obstacles[i].Radius > 0;
            if (!usable)
            {
                continue;
            }

            var gap = (state.Position - state.Obstacles[i].Position).Length - state.Obstacles[i].PhysicalRadius;
            if (gap < nearest)
            {
                nearest  = gap;
                scanBody = _bodies[i].GetOWRigidbody();
            }
        }

        var frame    = ReadFrame(state, scanBody);
        var velocity = state.Velocity - frame.Velocity;
        var distance = CollisionBraking.ProbeDistance(state, frame.Velocity, frame.Acceleration);
        var bounds   = HullBounds(position);

        _scanFrame     = frame;
        _scanBody      = scanBody;
        _scanSpeed     = velocity.Length;
        _scanReserve   = Math.Max(0.1, state.Thrust - (state.ExternalAcceleration - frame.Acceleration).Length);
        _velocityProbe = new DebugSweep(bounds.center, bounds.extents, Quaternion.identity, ToUnity(velocity.Unit), (float)Math.Min(distance, float.MaxValue));
        _steeringProbe = default;

        var overlaps = Physics.OverlapBoxNonAlloc(bounds.center, bounds.extents, _overlaps, Quaternion.identity, OWLayerMask.physicalMask, QueryTriggerInteraction.Ignore);
        if (overlaps == _overlaps.Length)
        {
            Block(null, scanBody, "Collision scan crowded");
        }

        for (var i = 0; !Active && i < overlaps; i++)
        {
            if (!IsSelf(_overlaps[i]))
            {
                Block(_overlaps[i], scanBody, "Obstacle beside hull");
            }
        }

        if (!Active)
        {
            Sweep(bounds, velocity.Unit, distance, scanBody);
        }

        if (!Active)
        {
            // Also cover a turn or acceleration from rest before the ship starts moving that way.
            var predicted = velocity + (command.Acceleration + state.ExternalAcceleration - frame.Acceleration) * 1.5;
            _steeringProbe = new DebugSweep(bounds.center, bounds.extents, Quaternion.identity, ToUnity(predicted.Unit), (float)Math.Min(distance, float.MaxValue));
            Sweep(bounds, predicted.Unit, distance, scanBody);
        }
    }

    private CollisionFrame ReadFrame(FlightState state, OWRigidbody body)
    {
        if (body == null)
        {
            return default;
        }

        var radius = (double)30;
        for (var i = 0; i < _bodies.Count; i++)
        {
            bool matches = _bodies[i] != null && _bodies[i].GetOWRigidbody() == body;
            if (matches)
            {
                radius = state.Obstacles[i].PhysicalRadius;
                break;
            }
        }

        return CollisionFrame.Create(
            state.Position - FromUnity(body.GetWorldCenterOfMass()),
            FromUnity(body.GetVelocity()),
            FromUnity(body.GetAcceleration()),
            FromUnity(body.GetAngularVelocity()),
            FromUnity(body.GetAngularAcceleration()),
            radius);
    }

    private FlightCommand Recover(FlightState state, FlightCommand command, int dangerIndex, Vector3 position)
    {
        if (_hadHoldBody && _holdBody == null)
        {
            NeedsManualControl = true;

            return new FlightCommand(default, FlightPhase.Hold);
        }

        _recoveryBounds = HullBounds(position);

        var frame = ReadFrame(state, _holdBody);
        var clear = CollisionEscape.TrySelect(state, command, dangerIndex, frame.Velocity, frame.Acceleration, _segmentClear, out var selected);

        NeedsManualControl = !clear;
        Active             = !clear;
        Recovering         = clear;

        return clear ? selected : new FlightCommand(default, FlightPhase.Hold);
    }

    private bool RecoverySegmentClear(Vector start, Vector end, double padding)
    {
        var center   = _recoveryBounds.center  + ToUnity(start);
        var extents  = _recoveryBounds.extents + Vector3.one * (float)padding;
        var overlaps = Physics.OverlapBoxNonAlloc(center, extents, _overlaps, Quaternion.identity, OWLayerMask.physicalMask, QueryTriggerInteraction.Ignore);
        if (overlaps == _overlaps.Length)
        {
            return false;
        }

        for (int i = 0; i < overlaps; i++)
        {
            if (!IsSelf(_overlaps[i]))
            {
                return false;
            }
        }

        var displacement = end - start;
        if (displacement.Length < 1e-6)
        {
            return true;
        }

        var count = Physics.BoxCastNonAlloc(center, extents, ToUnity(displacement.Unit), _hits, Quaternion.identity, (float)displacement.Length, OWLayerMask.physicalMask, QueryTriggerInteraction.Ignore);
        if (count == _hits.Length)
        {
            return false;
        }

        for (var i = 0; i < count; i++)
        {
            if (!IsSelf(_hits[i].collider))
            {
                return false;
            }
        }

        return true;
    }

    private Bounds HullBounds(Vector3 position)
    {
        var bounds = new Bounds(position, Vector3.zero);
        var found  = false;
        foreach (var collider in _hull)
        {
            var physical = collider != null && collider.enabled && !collider.isTrigger && collider.attachedRigidbody == _ship.GetRigidbody();
            if (!physical)
            {
                continue;
            }

            bounds.Encapsulate(collider.bounds);
            found = true;
        }

        if (!found)
        {
            throw new InvalidOperationException("No physical ship hull was available for collision checks.");
        }

        bounds.Expand(1);

        return bounds;
    }

    private void Sweep(Bounds bounds, Vector direction, double distance, OWRigidbody scanBody)
    {
        if (direction.LengthSquared < 0.5)
        {
            return;
        }

        var count = Physics.BoxCastNonAlloc(bounds.center, bounds.extents, ToUnity(direction), _hits, Quaternion.identity, (float)Math.Min(distance, float.MaxValue), OWLayerMask.physicalMask, QueryTriggerInteraction.Ignore);
        if (count == _hits.Length)
        {
            Block(null, scanBody, "Collision scan crowded");
            return;
        }

        Collider closest = null;

        var nearest = float.PositiveInfinity;

        for (var i = 0; i < count; i++)
        {
            var relevant = !IsSelf(_hits[i].collider) && _hits[i].distance < nearest;
            if (relevant)
            {
                nearest = _hits[i].distance;
                closest = _hits[i].collider;
            }
        }

        if (closest != null)
        {
            _hitDistance = nearest;
            Block(closest, scanBody, "Obstacle ahead");
        }
    }

    private void Block(Collider collider, OWRigidbody fallback, string reason)
    {
        var previousBody = _holdBody;
        _holdBody = collider != null && collider.attachedRigidbody != null
            ? collider.attachedRigidbody.GetComponent<OWRigidbody>() : null;
        if (_holdBody == null)
        {
            _holdBody = fallback;
        }

        _hadHoldBody = _holdBody != null;

        Active        = true;
        _reason       = reason;
        _holdCollider = collider;
        if (previousBody == _holdBody && ReferenceName != "local space")
        {
            return;
        }

        ReferenceName = "nearby obstacle";
        foreach (var body in _bodies)
        {
            var matches = body != null && body.GetOWRigidbody() == _holdBody;
            if (matches)
            {
                var name    = body.GetAstroObjectName();
                var display = name == AstroObject.Name.CustomString ? body.GetCustomName() : AstroObject.AstroObjectNameToString(name);
                ReferenceName = string.IsNullOrEmpty(display) ? "nearby obstacle" : display;
                break;
            }
        }
    }

    private bool IsSelf(Collider collider) => collider == null || collider.attachedRigidbody == _ship.GetRigidbody()
                                                               || _player != null && collider.attachedRigidbody == _player.GetRigidbody()
                                                               || collider.transform.IsChildOf(_ship.transform);

    private string ScanDiagnostic()
    {
        var name         = _scanBody != null ? _scanBody.name : "inertial space";
        var velocity     = _scanFrame.Velocity;
        var acceleration = _scanFrame.Acceleration;

        return $"; scan body={name}; rotation weight={_scanFrame.RotationWeight:F3}; relative speed={_scanSpeed:F2}m/s; braking reserve={_scanReserve:F2}m/s^2; probe distance={_velocityProbe.Distance:F1}m; frame velocity=({velocity.X:F2},{velocity.Y:F2},{velocity.Z:F2}); frame acceleration=({acceleration.X:F2},{acceleration.Y:F2},{acceleration.Z:F2})";
    }

    private static Vector FromUnity(Vector3 value) => new(value.x, value.y, value.z);

    private static Vector3 ToUnity(Vector value) => new((float)value.X, (float)value.Y, (float)value.Z);
}
