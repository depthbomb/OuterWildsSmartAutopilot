using SmartAutopilot.Navigation;
using UnityEngine;
using Vector = SmartAutopilot.Navigation.Vector;

namespace SmartAutopilot
{
    internal sealed class NavigationHull
    {
        public int ColliderCount { get; private set; }

        private double _radius;
        private double _nextScan;

        private readonly OWRigidbody _body;
        private readonly OWRigidbody _staticBody;
        private readonly Collider[]  _colliders;

        public NavigationHull(OWRigidbody body)
        {
            _body = body;

            var controller = Locator.GetRingWorldController();

            _staticBody = controller != null && controller.GetRingWorldBody() == body ? GameFields.StaticRingBody.Get(controller) : null;

            var colliders    = new List<Collider>(body.GetComponentsInChildren<Collider>(true));
            var separateBody = _staticBody != null && _staticBody != body && !_staticBody.transform.IsChildOf(body.transform);
            if (separateBody)
            {
                colliders.AddRange(_staticBody.GetComponentsInChildren<Collider>(true));
            }

            _colliders = [.. colliders];
        }

        public double GetRadius(double time)
        {
            if (time < _nextScan)
            {
                return _radius;
            }

            _nextScan = time + 0.75;

            ColliderCount = 0;

            var origin = _body.GetWorldCenterOfMass();
            foreach (var collider in _colliders)
            {
                var physical = collider != null && !collider.isTrigger && (1 << collider.gameObject.layer & OWLayerMask.physicalMask) != 0;
                if (!physical)
                {
                    continue;
                }

                var parentBody = collider.GetComponentInParent<OWRigidbody>();
                var attached   = parentBody == _body || _staticBody != null && parentBody == _staticBody;
                if (!attached)
                {
                    continue;
                }

                // Mesh and primitive geometry remains available when cloaking or LOD disables a collider.
                Bounds bounds;
                switch (collider)
                {
                    case MeshCollider mesh when mesh.sharedMesh != null:
                        bounds = mesh.sharedMesh.bounds;
                        break;
                    case BoxCollider box:
                        bounds = new Bounds(box.center, box.size);
                        break;
                    case SphereCollider sphere:
                        bounds = new Bounds(sphere.center, Vector3.one * (sphere.radius * 2));
                        break;
                    case CapsuleCollider capsule:
                    {
                        var size = Vector3.one * (capsule.radius * 2);
                        size[capsule.direction] = Mathf.Max(size[capsule.direction], capsule.height);
                        bounds                  = new Bounds(capsule.center, size);
                        break;
                    }
                    default:
                    {
                        var active = collider.enabled && collider.gameObject.activeInHierarchy;
                        if (active)
                        {
                            var world = collider.bounds;
                            _radius = Math.Max(_radius, HullEnvelope.BoxRadius(FromUnity(world.center - origin), new Vector(world.extents.x, 0, 0), new Vector(0, world.extents.y, 0), new Vector(0, 0, world.extents.z)));
                            ColliderCount++;
                        }
                        continue;
                    }
                }

                var transform = collider.transform;
                _radius = Math.Max(_radius, HullEnvelope.BoxRadius(FromUnity(transform.TransformPoint(bounds.center) - origin),
                    FromUnity(transform.TransformVector(new Vector3(bounds.extents.x, 0, 0))),
                    FromUnity(transform.TransformVector(new Vector3(0, bounds.extents.y, 0))),
                    FromUnity(transform.TransformVector(new Vector3(0, 0, bounds.extents.z)))));
                ColliderCount++;
            }

            // Retain the largest observed envelope while sails move or collider visibility changes.
            return _radius;
        }

        private static Vector FromUnity(Vector3 value) => new(value.x, value.y, value.z);
    }
}
