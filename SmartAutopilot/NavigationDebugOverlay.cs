using System;
using System.Collections.Generic;
using SmartAutopilot.Navigation;
using UnityEngine;
using Vector = SmartAutopilot.Navigation.Vector;

namespace SmartAutopilot
{
    internal readonly struct DebugSweep
    {
        public readonly Vector3 Center;
        public readonly Vector3 Extents;
        public readonly Quaternion Rotation;
        public readonly Vector3 Direction;
        public readonly float Distance;

        public DebugSweep(Vector3 center, Vector3 extents, Quaternion rotation, Vector3 direction, float distance)
        {
            Center    = center;
            Extents   = extents;
            Rotation  = rotation;
            Direction = direction;
            Distance  = distance;
        }
    }

    internal sealed class NavigationDebugOverlay : IDisposable
    {
        private          int                _used;
        private          Vector3            _cameraPosition;
        private          float              _worldUnitsPerPixel;
        private          double             _nextDraw;
        private readonly GameObject         _root;
        private readonly Material           _material;
        private readonly List<LineRenderer> _lines    = [];
        private readonly Vector[]           _route    = new Vector[4];
        private readonly Vector3[]          _corners  = new Vector3[8];
        private readonly Vector3[]          _ring     = new Vector3[32];
        private readonly Vector3[]          _unitRing = CreateUnitRing();
        private readonly PromptManager      _prompts;
        private readonly ScreenPrompt       _legend;

        public NavigationDebugOverlay()
        {
            var shader = Shader.Find("Sprites/Default");
            if (!shader)
            {
                throw new InvalidOperationException("The navigation debug line shader is unavailable.");
            }

            _material = new Material(shader);
            _root     = new GameObject("Smart Autopilot Debug");
            _prompts  = Locator.GetPromptManager();
            _legend   = new ScreenPrompt("NAV DEBUG: cyan route | green arrival | orange clearance\nred collision sweep | yellow launch | blue velocity (1s) | magenta thrust (10x)");
            try
            {
                if (_prompts)
                {
                    _prompts.AddScreenPrompt(_legend, PromptPosition.LowerLeft);
                }
            }
            catch
            {
                UnityEngine.Object.Destroy(_root);
                UnityEngine.Object.Destroy(_material);
                throw;
            }
        }

        public void Draw(FlightState state, FlightComputer computer, FlightCommand command, ShipCollisionGuard collision, ShipDepartureController departure, float refreshRate)
        {
            double renderTime = Time.unscaledTime;
            if (renderTime + 0.0001 < _nextDraw)
            {
                return;
            }

            double interval = 1.0 / refreshRate;
            _nextDraw = _nextDraw <= 0 || renderTime - _nextDraw > interval ? renderTime + interval : _nextDraw + interval;
            var playerCamera = Locator.GetPlayerCamera();
            var camera = playerCamera ? playerCamera.GetComponent<Camera>() : null;
            if (!camera)
            {
                Clear();

                return;
            }

            _cameraPosition = camera.transform.position;
            _worldUnitsPerPixel = 2 * Mathf.Tan(camera.fieldOfView * Mathf.Deg2Rad * 0.5f) / Mathf.Max(1, camera.pixelHeight);
            _root.SetActive(true);
            _legend.SetVisibility(PlayerState.AtFlightConsole());
            _used = 0;
            var ship = ToUnity(state.Position);
            int count = computer.GetDebugRoute(state, _route);
            for (int i = 1; i < count; i++)
            {
                Segment(ToUnity(_route[i - 1]), ToUnity(_route[i]), Color.cyan);
                Sphere(ToUnity(_route[i]), 12, Color.cyan);
            }

            Sphere(ToUnity(state.TargetPosition), (float)state.ArrivalRadius, Color.green);
            for (int i = 0; i < state.Obstacles.Count && i < 64; i++)
            {
                var obstacle = state.Obstacles[i];
                if (obstacle.Radius <= 0)
                {
                    continue;
                }

                var center = ToUnity(obstacle.Position);
                Sphere(center, (float)obstacle.PhysicalRadius, new Color(0.55f, 0.55f, 0.55f, 0.35f));
                Sphere(center, (float)obstacle.Radius, new Color(1, 0.5f, 0, 0.65f));
            }

            Segment(ship, ship + ToUnity(state.Velocity - state.TargetVelocity), Color.blue);
            Segment(ship, ship + ToUnity(command.Acceleration) * 10, Color.magenta);
            bool launching = departure != null && departure.Active;
            if (launching)
            {
                Sweep(departure.DebugProbe, Color.yellow);
                Segment(ship, departure.DebugClearancePoint, Color.yellow);
                Sphere(departure.DebugClearancePoint, 5, Color.yellow);
            }
            else if (collision != null)
            {
                Sweep(collision.DebugVelocityProbe, Color.red);
                Sweep(collision.DebugSteeringProbe, new Color(1, 0.3f, 0.3f, 0.65f));
            }

            for (int i = _used; i < _lines.Count; i++)
            {
                _lines[i].enabled = false;
            }
        }

        public void Clear()
        {
            _nextDraw = 0;
            if (_root)
            {
                _root.SetActive(false);
            }

            _legend.SetVisibility(false);
        }

        public void Dispose()
        {
            try
            {
                if (_prompts)
                {
                    _prompts.RemoveScreenPrompt(_legend, PromptPosition.LowerLeft);
                }
            }
            finally
            {
                UnityEngine.Object.Destroy(_root);
                UnityEngine.Object.Destroy(_material);
            }
        }

        private void Sphere(Vector3 center, float radius, Color color)
        {
            if (radius <= 0)
            {
                return;
            }

            float width = Width(Mathf.Abs(Vector3.Distance(_cameraPosition, center) - radius));
            for (int axis = 0; axis < 3; axis++)
            {
                var line = Line(32, color, true, width, width);
                for (int point = 0; point < 32; point++)
                {
                    float x = _unitRing[point].x * radius;
                    float y = _unitRing[point].y * radius;
                    var offset = axis == 0 ? new Vector3(x, y, 0) : axis == 1 ? new Vector3(x, 0, y) : new Vector3(0, x, y);
                    _ring[point] = center + offset;
                }

                line.SetPositions(_ring);
            }
        }

        private void Sweep(DebugSweep sweep, Color color)
        {
            if (sweep.Extents.sqrMagnitude < 0.01f)
            {
                return;
            }

            for (int corner = 0; corner < 8; corner++)
            {
                var local = new Vector3((corner & 1) == 0 ? -sweep.Extents.x : sweep.Extents.x,
                    (corner & 2) == 0 ? -sweep.Extents.y : sweep.Extents.y,
                    (corner & 4) == 0 ? -sweep.Extents.z : sweep.Extents.z);
                _corners[corner] = sweep.Center + sweep.Rotation * local;
            }

            var end = sweep.Direction * sweep.Distance;
            for (int corner = 0; corner < 8; corner++)
            {
                Segment(_corners[corner], _corners[corner] + end, color);
                for (int bit = 1; bit <= 4; bit *= 2)
                {
                    if ((corner & bit) == 0)
                    {
                        Segment(_corners[corner], _corners[corner | bit], color);
                        Segment(_corners[corner] + end, _corners[corner | bit] + end, color);
                    }
                }
            }
        }

        private void Segment(Vector3 start, Vector3 end, Color color)
        {
            var line = Line(2, color, false, Width(Vector3.Distance(_cameraPosition, start)), Width(Vector3.Distance(_cameraPosition, end)));
            line.SetPosition(0, start);
            line.SetPosition(1, end);
        }

        private float Width(float distance) =>
            // Approximately one screen pixel, independent of the line's length or the probe's reach.
            Mathf.Clamp(distance * _worldUnitsPerPixel, 0.005f, 2);

        private LineRenderer Line(int count, Color color, bool loop, float startWidth, float endWidth)
        {
            if (_used == _lines.Count)
            {
                var child = new GameObject("Navigation line");
                child.transform.SetParent(_root.transform, false);
                var renderer = child.AddComponent<LineRenderer>();
                renderer.sharedMaterial = _material;
                renderer.useWorldSpace  = true;
                _lines.Add(renderer);
            }

            var line = _lines[_used++];
            line.enabled       = true;
            line.loop          = loop;
            line.positionCount = count;
            color.a           *= 0.55f;
            line.startColor    = color;
            line.endColor      = color;
            line.startWidth    = startWidth;
            line.endWidth      = endWidth;

            return line;
        }

        private static Vector3[] CreateUnitRing()
        {
            var points = new Vector3[32];
            for (int i = 0; i < points.Length; i++)
            {
                float angle = i * Mathf.PI / 16;
                points[i] = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0);
            }

            return points;
        }

        private static Vector3 ToUnity(Vector value) => new((float)value.X, (float)value.Y, (float)value.Z);
    }
}
