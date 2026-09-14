using SmartAutopilot.Navigation;
using UnityEngine;

namespace SmartAutopilot;

internal sealed class ShipArrivalLimits
{
    private sealed class Volume
    {
        public readonly ThrustRuleset Rules;
        public readonly Shape[]       Shapes;
        public readonly Collider[]    Colliders;

        public Volume(ThrustRuleset rules)
        {
            Rules     = rules;
            Shapes    = rules.GetComponents<Shape>();
            Colliders = rules.GetComponents<Collider>();
        }
    }

    private readonly List<Volume> _volumes = [];

    public ShipArrivalLimits(OWRigidbody target)
    {
        foreach (var rules in UnityEngine.Object.FindObjectsOfType<ThrustRuleset>())
        {
            bool attached = rules.GetAttachedOWRigidbody() == target;
            if (attached)
            {
                _volumes.Add(new Volume(rules));
            }
        }
    }

    public void Refresh(FlightState state)
    {
        state.ArrivalThrustLimits.Clear();
        var center = new Vector3((float)state.TargetPosition.X, (float)state.TargetPosition.Y, (float)state.TargetPosition.Z);
        foreach (var volume in _volumes)
        {
            var active = volume.Rules != null && volume.Rules.isActiveAndEnabled && volume.Rules.IsVolumeActive();
            if (!active)
            {
                continue;
            }

            var limit = volume.Rules.GetThrustLimit();
            if (double.IsNaN(limit) || limit < 0)
            {
                throw new InvalidOperationException("A destination thrust volume supplied an invalid limit.");
            }

            if (limit >= state.MaximumThrust)
            {
                continue;
            }

            var radius = (double)0;
            foreach (var shape in volume.Shapes)
            {
                if (shape != null && shape.active)
                {
                    var bounds = shape.CalcWorldBounds();
                    radius = Math.Max(radius, Vector3.Distance(center, bounds.center) + bounds.radius);
                }
            }

            foreach (var collider in volume.Colliders)
            {
                if (collider == null || !collider.enabled)
                {
                    continue;
                }

                var bounds = collider.bounds;
                var extent = collider is SphereCollider ? Math.Max(bounds.extents.x, Math.Max(bounds.extents.y, bounds.extents.z)) : bounds.extents.magnitude;
                radius = Math.Max(radius, Vector3.Distance(center, bounds.center) + extent);
            }

            if (radius > 0)
            {
                // Include hull extent and trigger-update delay before the center enters the volume.
                state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(radius + 15, limit));
            }
        }
    }
}
