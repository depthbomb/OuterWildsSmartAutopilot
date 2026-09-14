using System;

namespace SmartAutopilot.Navigation;

internal sealed class FlightComputer
{
    public int    AvoidanceBodyIndex => _escapeBody;
    public int    DetourBodyIndex    => _hasRoute ? _waypointBody : -1;
    public int    HoldBodyIndex      => _holdBody;
    public bool   RouteBlocked       { get; private set; }
    public string RouteStatus        => _planner.FailureReason;

    private Vector _goalOffset;
    private Vector _waypointOffset;
    private Vector _preferredDirection;
    private Vector _followingOffset;
    private int    _waypointBody  = -1;
    private int    _followingBody = -1;
    private int    _holdBody      = -1;
    private double _remainingAfterWaypoint;
    private double _nextPlanTime;
    private bool   _hasRoute;
    private int    _escapeBody = -1;
    private double _escapeStart;
    private double _escapeClearSince = -1;
    private bool   _finalBraking;
    private bool   _matchingArrival;
    private bool   _recoveringArrival;
    private bool   _complexApproach;

    private readonly RoutePlanner _planner = new();

    public FlightCommand Step(FlightState state)
    {
        var targetOffset = state.Position - state.TargetPosition;
        var distance     = targetOffset.Length;
        var goal         = state.TargetPosition + (_hasRoute ? _goalOffset : targetOffset.Unit * state.ArrivalRadius);

        UpdateAvoidance(state);

        // A diversion or stalled approach can invalidate final braking before reaching the destination.
        var arrivalTolerance   = Math.Max(60, state.ArrivalRadius * 0.1);
        var targetClosingSpeed = -Vector.Dot(state.Velocity - state.TargetVelocity, targetOffset.Unit);
        var recoverApproach    = _finalBraking && !_recoveringArrival && distance > state.ArrivalRadius + arrivalTolerance && (_escapeBody >= 0 || _matchingArrival || targetClosingSpeed <= 0.5);
        if (recoverApproach)
        {
            _finalBraking      = false;
            _matchingArrival   = false;
            _recoveringArrival = false;
            _hasRoute          = false;
            _nextPlanTime      = 0;
        }

        if (_escapeBody >= 0)
        {
            _complexApproach = true;
            _nextPlanTime    = 0;

            var danger          = state.Obstacles[_escapeBody];
            var reserve         = BrakingAuthority(state, danger.Acceleration);
            var outward         = (state.Position - danger.Position).Unit;
            var escapeVelocity  = state.Velocity - danger.Velocity;
            var lateralVelocity = escapeVelocity - outward * Vector.Dot(escapeVelocity, outward);
            var escapeSpeed     = Math.Min(100, Math.Sqrt(2 * reserve * Math.Max(30, danger.Radius + 120 - (state.Position - danger.Position).Length)));
            var desiredVelocity = danger.Velocity + outward * escapeSpeed + lateralVelocity;

            return Control(state, desiredVelocity, danger.Acceleration, FlightPhase.Escape);
        }

        // The restricted-thrust braking envelope settles at +20m. Accept a small
        // distance tolerance at matched velocity so tiny game inputs cannot stall arrival.
        var settledAtBoundary = distance <= state.ArrivalRadius + 21 && (state.Velocity - state.TargetVelocity).Length < 0.5;
        if (distance <= state.ArrivalRadius + 20 || settledAtBoundary)
        {
            _finalBraking    = true;
            _matchingArrival = true;
        }

        if (_finalBraking)
        {
            return BrakeToArrival(state);
        }

        var needsPlan = state.Time >= _nextPlanTime;
        if (needsPlan)
        {
            var planningTarget     = state.TargetPosition;
            var nearArrival        = state.TargetPosition + targetOffset.Unit * state.ArrivalRadius;
            var obstructedApproach = !RoutePlanner.SegmentClear(state.Position, nearArrival, state.Obstacles);
            if (obstructedApproach && OrbitalMotion.TryCreate(state, state.TargetObstacleIndex, out var orbit))
            {
                var remaining = Math.Max(0, distance - state.ArrivalRadius);
                var leadTime  = Math.Min(12, Math.Sqrt(2 * remaining / Math.Max(1, state.Thrust)) * 0.5);
                planningTarget += orbit.Position(leadTime) - state.Obstacles[state.TargetObstacleIndex].Position;
            }

            _hasRoute = _planner.TryPlanToTarget(state.Position, planningTarget, state.ArrivalRadius, state.Obstacles, _preferredDirection, out goal, out var next, out var following, out var routeDistance);
            var forecastBlocked = !_hasRoute && (planningTarget - state.TargetPosition).LengthSquared > 1;
            if (forecastBlocked)
            {
                _hasRoute = _planner.TryPlanToTarget(state.Position, state.TargetPosition, state.ArrivalRadius, state.Obstacles, _preferredDirection, out goal, out next, out following, out routeDistance);
            }

            RouteBlocked = !_hasRoute;
            if (_hasRoute)
            {
                _holdBody = -1;
            }
            _goalOffset             = goal - state.TargetPosition;
            _waypointBody           = next.ObstacleIndex;
            _waypointOffset         = next.Position - (_waypointBody >= 0 ? state.Obstacles[_waypointBody].Position : state.TargetPosition);
            _followingBody          = following.ObstacleIndex;
            _followingOffset        = following.Position - (_followingBody >= 0 ? state.Obstacles[_followingBody].Position : state.TargetPosition);
            _remainingAfterWaypoint = Math.Max(0, routeDistance - (next.Position - state.Position).Length);
            _nextPlanTime           = state.Time + 0.75;
        }

        if (!_hasRoute)
        {
            var staleHold = _holdBody < 0 || _holdBody >= state.Obstacles.Count || state.Obstacles[_holdBody].Radius <= 0;
            if (staleHold)
            {
                _holdBody = FindHoldBody(state);
            }

            var anchor = _holdBody >= 0 ? state.Obstacles[_holdBody] : null;

            // An unreachable moving destination must not pull the ship back into the obstacle it just escaped.
            return Control(state, anchor?.Velocity ?? default, anchor?.Acceleration ?? default, FlightPhase.Hold);
        }

        var detour = _waypointBody >= 0;

        _complexApproach |= detour;

        var anchorPosition         = detour ? state.Obstacles[_waypointBody].Position : state.TargetPosition;
        var anchorVelocity         = detour ? state.Obstacles[_waypointBody].Velocity : state.TargetVelocity;
        var anchorAcceleration     = detour ? state.Obstacles[_waypointBody].Acceleration : state.TargetAcceleration;
        var waypoint               = detour ? anchorPosition + _waypointOffset : goal;
        var displacement           = waypoint            - state.Position;
        var routeDistanceRemaining = displacement.Length + (detour ? _remainingAfterWaypoint : 0);
        var relativeVelocity       = state.Velocity      - anchorVelocity;
        if (detour)
        {
            var followingPosition = (_followingBody >= 0 ? state.Obstacles[_followingBody].Position : state.TargetPosition) + _followingOffset;
            var nextLeg           = followingPosition                                                                       - waypoint;
            var lookAhead         = Math.Max(120, relativeVelocity.Length * 3);
            var advance           = Math.Min(nextLeg.Length, Math.Max(0, lookAhead - displacement.Length));
            for (var attempt = 0; attempt < 8 && advance > 1; attempt++)
            {
                var aim = waypoint + nextLeg.Unit * advance;
                if (RoutePlanner.SegmentClear(state.Position, aim, state.Obstacles))
                {
                    displacement = aim - state.Position;
                    break;
                }

                advance *= 0.5;
            }
        }

        _preferredDirection = displacement.Unit;

        // Accelerate until meeting the braking envelope. Account for the velocity servo's response time.
        var decisiveArrival  = !_complexApproach;
        var brakingAuthority = decisiveArrival ? ArrivalBrakingAuthority(state) : BrakingAuthority(state, anchorAcceleration);
        var responseMargin   = brakingAuthority * (decisiveArrival ? 0.35 : 1.5);
        var speed            = Math.Sqrt(2 * brakingAuthority * routeDistanceRemaining + responseMargin * responseMargin) - responseMargin;
        if (decisiveArrival)
        {
            speed = LimitTargetApproach(state, speed);
        }

        if (!detour)
        {
            speed = ArrivalBraking.LimitSpeed(state, speed);
        }

        var arrivalSpeed = speed;
        var turn         = (displacement.Unit - relativeVelocity.Unit).Length;
        var turning      = detour || (relativeVelocity.Length > 5 && turn > 0.35);
        if (turning)
        {
            // Limit speed by the turn's curvature, not by a requirement to stop at its waypoint.
            var turnRadius = Math.Max(60, displacement.Length) / Math.Max(0.1, turn);
            speed = Math.Min(speed, Math.Sqrt(brakingAuthority * turnRadius));
        }

        if (state.SpeedLimit > 0)
        {
            speed = Math.Min(speed, state.SpeedLimit);
        }

        var closingSpeed = Vector.Dot(relativeVelocity, displacement.Unit);
        if (!detour && (closingSpeed > arrivalSpeed + 1 || distance <= state.ArrivalRadius + 20))
        {
            _finalBraking = true;

            return BrakeToArrival(state);
        }

        var steering = Steer(state, displacement.Unit, speed, anchorVelocity, anchorAcceleration, detour ? FlightPhase.Detour : FlightPhase.Cruise);

        return LimitClosingAcceleration(state, steering);
    }

    public int GetDebugRoute(FlightState state, Vector[] points)
    {
        if (!_hasRoute || points.Length < 4)
        {
            return 0;
        }

        points[0] = state.Position;
        points[1] = (_waypointBody  >= 0 ? state.Obstacles[_waypointBody].Position : state.TargetPosition)  + _waypointOffset;
        points[2] = (_followingBody >= 0 ? state.Obstacles[_followingBody].Position : state.TargetPosition) + _followingOffset;
        points[3] = state.TargetPosition                                                                    + _goalOffset;

        return 4;
    }

    private FlightCommand BrakeToArrival(FlightState state)
    {
        var relativeVelocity = state.Velocity       - state.TargetVelocity;
        var toTarget         = state.TargetPosition - state.Position;
        var speed            = relativeVelocity.Length;
        var remaining        = toTarget.Length - state.ArrivalRadius;
        var closing          = Vector.Dot(relativeVelocity, toTarget.Unit);
        if (remaining < -10 && (speed < 0.5 || closing < -0.5))
        {
            _recoveringArrival = true;
        }

        var clearanceBody = -1;
        if (_recoveringArrival)
        {
            for (var i = 0; i < state.Obstacles.Count; i++)
            {
                var obstacle          = state.Obstacles[i];
                var targetBody        = i                              == state.TargetObstacleIndex || (state.TargetObstacleIndex < 0 && (state.TargetPosition - obstacle.Position).LengthSquared < 1);
                var nearEnclosingBody = !targetBody && obstacle.Radius > 0 && (state.TargetPosition - obstacle.Position).Length < obstacle.Radius && (state.Position - obstacle.Position).Length < obstacle.Radius + 250;
                if (nearEnclosingBody)
                {
                    clearanceBody = i;
                    break;
                }
            }
        }

        var recoveryMargin = clearanceBody >= 0 ? 20 : -1;
        if (_recoveringArrival && remaining < recoveryMargin)
        {
            // Restore distance after avoidance without discarding its useful outward momentum.
            // Step checks collision avoidance before allowing this bounded correction.
            var authority       = BrakingAuthority(state, state.TargetAcceleration);
            var recoverySpeed   = Math.Min(40, Math.Sqrt(2 * authority * Math.Max(0, -remaining - 1)));
            var desiredVelocity = state.TargetVelocity - toTarget.Unit * recoverySpeed;
            if (clearanceBody >= 0)
            {
                // Match-velocity alone can carry an orbiting target's arrival point back toward
                // its primary. Leave room to brake before releasing this correction.
                var obstacle     = state.Obstacles[clearanceBody];
                var outward      = (state.Position - obstacle.Position).Unit;
                var outwardSpeed = Vector.Dot(desiredVelocity - obstacle.Velocity, outward);
                desiredVelocity += outward * Math.Max(0, 40 - outwardSpeed);
            }

            var recoveryAcceleration = (desiredVelocity - state.Velocity) / 0.35 + state.TargetAcceleration - state.ExternalAcceleration;

            return new FlightCommand(recoveryAcceleration.Limited(state.Thrust), FlightPhase.Braking);
        }

        _recoveringArrival = false;
        if (remaining <= 20 || closing <= 0)
        {
            _matchingArrival = true;
        }

        if (_matchingArrival && speed < 0.5 && remaining >= -10 && remaining <= Math.Max(60, state.ArrivalRadius * 0.1))
        {
            return new FlightCommand(default, FlightPhase.Arrived);
        }

        // Within the arrival zone, finish matching velocity without reversing to chase a point.
        Vector desiredAcceleration;
        if (_matchingArrival)
        {
            var deceleration = Math.Min(state.Thrust, speed / Math.Max(0.001, state.DeltaTime));
            desiredAcceleration = -relativeVelocity.Unit * deceleration;
        }
        else if (_complexApproach)
        {
            var authority = BrakingAuthority(state, state.TargetAcceleration);
            var margin    = authority * 1.5;
            var safeSpeed = Math.Sqrt(2 * authority * Math.Max(0, remaining) + margin * margin) - margin;
            safeSpeed = ArrivalBraking.LimitSpeed(state, safeSpeed);

            var desiredVelocity = toTarget.Unit * Math.Min(speed, safeSpeed);
            desiredAcceleration = (desiredVelocity - relativeVelocity) / 1.1;
        }
        else
        {
            // Apply the deceleration needed to stop at the arrival shell, updating for each physics step.
            // Keep a small capture margin and remove sideways motion within the same thrust budget.
            var deceleration    = Math.Min(state.Thrust, closing * closing / (2 * Math.Max(1, remaining - 10)));
            var targetSafeSpeed = LimitTargetApproach(state, double.PositiveInfinity);
            targetSafeSpeed = ArrivalBraking.LimitSpeed(state, targetSafeSpeed);
            deceleration    = Math.Min(state.Thrust, Math.Max(deceleration, (closing - targetSafeSpeed) / Math.Max(0.05, state.DeltaTime)));

            var lateralVelocity   = relativeVelocity         - toTarget.Unit * closing;
            var compensation      = state.TargetAcceleration - state.ExternalAcceleration;
            var compensationAlong = Vector.Dot(compensation, toTarget.Unit);
            var alongThrust       = Math.Max(-state.Thrust, Math.Min(state.Thrust, compensationAlong - deceleration));
            var lateralBudget     = Math.Sqrt(Math.Max(0, state.Thrust * state.Thrust                - alongThrust * alongThrust));
            var lateralThrust     = (-lateralVelocity / 1.1 + compensation - toTarget.Unit * compensationAlong).Limited(lateralBudget);

            return new FlightCommand(toTarget.Unit * alongThrust + lateralThrust, FlightPhase.Braking);
        }

        var acceleration = desiredAcceleration + state.TargetAcceleration - state.ExternalAcceleration;

        return new FlightCommand(acceleration.Limited(state.Thrust), FlightPhase.Braking);
    }

    private static int FindHoldBody(FlightState state)
    {
        var nearestBody = -1;
        var nearestGap  = double.PositiveInfinity;
        for (var i = 0; i < state.Obstacles.Count; i++)
        {
            var obstacle = state.Obstacles[i];
            if (obstacle.Radius <= 0)
            {
                continue;
            }

            var enclosesArrival = (state.TargetPosition - obstacle.Position).Length + state.ArrivalRadius < obstacle.Radius + 50;
            if (enclosesArrival)
            {
                return i;
            }

            var gap = (state.Position - obstacle.Position).Length - obstacle.Radius;
            if (gap < nearestGap)
            {
                nearestGap  = gap;
                nearestBody = i;
            }
        }

        return nearestBody;
    }

    private static FlightCommand Steer(FlightState state,
                                       Vector      direction,
                                       double      desiredSpeed,
                                       Vector      frameVelocity,
                                       Vector      frameAcceleration,
                                       FlightPhase phase)
    {
        var velocity    = state.Velocity - frameVelocity;
        var forward     = velocity.Length > 5 ? velocity.Unit : direction;
        var turningBack = Vector.Dot(forward, direction) < 0.2;
        if (turningBack)
        {
            return Control(state, frameVelocity + direction * desiredSpeed, frameAcceleration, phase);
        }

        var lateralDirection     = direction - forward * Vector.Dot(direction, forward);
        var lateralAcceleration  = (lateralDirection * Math.Max(30, velocity.Length) / 1.1).Limited(state.Thrust * 0.7);
        var compensation         = frameAcceleration   - state.ExternalAcceleration;
        var offset               = lateralAcceleration + compensation;
        var projection           = Vector.Dot(offset, forward);
        var perpendicularSquared = (offset - forward * projection).LengthSquared;
        var availableAlong       = Math.Sqrt(Math.Max(0, state.Thrust * state.Thrust              - perpendicularSquared));
        var along                = Math.Max(-availableAlong - projection, Math.Min(availableAlong - projection, (desiredSpeed - velocity.Length) / 1.1));
        var acceleration         = offset + forward * along;

        return new FlightCommand(acceleration.Limited(state.Thrust), phase);
    }

    private void UpdateAvoidance(FlightState state)
    {
        var staleBody = _escapeBody >= 0 && (_escapeBody >= state.Obstacles.Count || state.Obstacles[_escapeBody].Radius <= 0);
        if (staleBody)
        {
            _escapeBody       = -1;
            _escapeClearSince = -1;
        }

        var urgentBody     = -1;
        var mostUrgent     = (double)0;
        var currentUrgency = (double)0;
        var urgentGap      = double.PositiveInfinity;
        for (var i = 0; i < state.Obstacles.Count; i++)
        {
            var obstacle = state.Obstacles[i];
            if (obstacle.Radius <= 0)
            {
                continue;
            }

            var offset             = state.Position - obstacle.Position;
            var velocity           = state.Velocity - obstacle.Velocity;
            var gap                = offset.Length  - obstacle.Radius;
            var closing            = Math.Max(0, -Vector.Dot(velocity, offset.Unit));
            var braking            = BrakingAuthority(state, obstacle.Acceleration);
            var stopping           = closing * closing / (2 * braking);
            var urgency            = stopping + closing     * 0.3 + 35 - gap;
            var approachingSurface = WillIntersect(offset, velocity, obstacle.Radius + 35, velocity.Length / braking + 1);
            var distantOrbiter     = gap > 100 && approachingSurface && urgency > 0;
            if (distantOrbiter && OrbitalMotion.ExcludesCollision(state, i, obstacle.Radius + 35, velocity.Length / braking + 1))
            {
                approachingSurface = false;
            }

            if (i == _escapeBody && (gap < 35 || approachingSurface))
            {
                currentUrgency = urgency;
            }

            if (urgency > mostUrgent && (gap < 35 || approachingSurface))
            {
                mostUrgent = urgency;
                urgentBody = i;
                urgentGap  = gap;
            }
        }

        var equivalentThreat = urgentBody                  >= 0           &&
                               urgentBody                  != _escapeBody &&
                               currentUrgency              > 0            &&
                               urgentGap                   >= 35          &&
                               mostUrgent - currentUrgency <= Math.Max(10, currentUrgency * 0.1);
        if (equivalentThreat)
        {
            urgentBody = _escapeBody;
        }

        if (urgentBody >= 0)
        {
            if (_escapeBody != urgentBody)
            {
                _escapeBody  = urgentBody;
                _escapeStart = state.Time;
            }

            _escapeClearSince = -1;
            return;
        }

        if (_escapeBody < 0)
        {
            return;
        }

        var danger           = state.Obstacles[_escapeBody];
        var relativePosition = state.Position          - danger.Position;
        var clearance        = relativePosition.Length - danger.Radius;
        var closingSpeed     = -Vector.Dot(state.Velocity - danger.Velocity, relativePosition.Unit);
        var recovered        = clearance > 100 && closingSpeed < 1;
        if (!recovered)
        {
            _escapeClearSince = -1;
            return;
        }

        if (_escapeClearSince < 0)
        {
            _escapeClearSince = state.Time;
        }

        var settled = state.Time - _escapeStart >= 1 && state.Time - _escapeClearSince >= 0.75;
        if (settled)
        {
            _escapeBody       = -1;
            _escapeClearSince = -1;
        }
    }

    private static FlightCommand LimitClosingAcceleration(FlightState state, FlightCommand command)
    {
        // Reduce inward acceleration before reaching the emergency envelope, preserving lateral steering.
        var acceleration = command.Acceleration;
        for (var i = 0; i < state.Obstacles.Count; i++)
        {
            var obstacle = state.Obstacles[i];
            var target   = i == state.TargetObstacleIndex ||state.TargetObstacleIndex < 0 && (obstacle.Position - state.TargetPosition).LengthSquared < 1;
            // Arrival planning handles destinations within another body's exclusion zone.
            var enclosesTarget = (obstacle.Position - state.TargetPosition).Length < obstacle.Radius + 100;
            if (obstacle.Radius <= 0 || target || enclosesTarget)
            {
                continue;
            }

            var offset        = state.Position - obstacle.Position;
            var outward       = offset.Unit;
            var velocity      = state.Velocity - obstacle.Velocity;
            var gap           = offset.Length  - obstacle.Radius;
            var authority     = BrakingAuthority(state, obstacle.Acceleration);
            var margin        = authority * 1.5;
            var safeClosing   = Math.Sqrt(2 * authority * Math.Max(0, gap - 100) + margin * margin) - margin;
            var closing       = -Vector.Dot(velocity, outward);
            var allowedInward = (safeClosing - closing)                                 / 1.1;
            var curvature     = Math.Max(0, velocity.LengthSquared - closing * closing) / Math.Max(1, offset.Length);
            var inward        = -Vector.Dot(acceleration + state.ExternalAcceleration - obstacle.Acceleration, outward) - curvature;
            if (inward <= allowedInward)
            {
                continue;
            }

            var forecastVelocity = velocity + (acceleration + state.ExternalAcceleration - obstacle.Acceleration) * 1.1;
            var horizon          = forecastVelocity.Length                                                        / authority + 1;
            var intersection     = WillIntersect(offset, forecastVelocity, obstacle.Radius + 100, horizon);
            if (!intersection)
            {
                continue;
            }

            acceleration = (acceleration + outward * (inward - allowedInward)).Limited(state.Thrust);
        }

        return new FlightCommand(acceleration, command.Phase);
    }

    private static double BrakingAuthority(FlightState state, Vector frameAcceleration)
        => Math.Max(state.Thrust * 0.1, state.Thrust - (state.ExternalAcceleration - frameAcceleration).Length) * 0.7;

    private static double ArrivalBrakingAuthority(FlightState state)
        => Math.Max(state.Thrust * 0.1, state.Thrust - (state.ExternalAcceleration - state.TargetAcceleration).Length) * 0.85;

    private static double LimitTargetApproach(FlightState state, double speed)
    {
        for (var i = 0; i < state.Obstacles.Count; i++)
        {
            var obstacle = state.Obstacles[i];
            var isTarget = obstacle.Radius > 0 && (i == state.TargetObstacleIndex || state.TargetObstacleIndex < 0 && (obstacle.Position - state.TargetPosition).LengthSquared < 1);
            if (!isTarget)
            {
                continue;
            }

            // Keep the faster arrival envelope outside the existing emergency avoidance threshold.
            var gap       = Math.Max(0, (state.Position - obstacle.Position).Length - obstacle.Radius - 100);
            var authority = BrakingAuthority(state, obstacle.Acceleration);
            var margin    = authority * 0.3;

            speed = Math.Min(speed, Math.Sqrt(2 * authority * gap + margin * margin) - margin);
        }

        return speed;
    }

    private static bool WillIntersect(Vector offset, Vector velocity, double radius, double horizon)
    {
        var closestTime = velocity.LengthSquared > 1e-9 ? Math.Max(0, Math.Min(horizon, -Vector.Dot(offset, velocity) / velocity.LengthSquared)) : 0;

        return (offset + velocity * closestTime).LengthSquared < radius * radius;
    }

    private static FlightCommand Control(FlightState state, Vector desiredVelocity, Vector frameAcceleration, FlightPhase phase)
    {
        var acceleration = (desiredVelocity - state.Velocity) / 1.1 + frameAcceleration - state.ExternalAcceleration;

        return new FlightCommand(acceleration.Limited(state.Thrust), phase);
    }
}
