using System;
using System.Globalization;
using System.Linq;

namespace SmartAutopilot.Automation
{
    [Serializable]
    internal sealed class AutomationRequest
    {
        public int      protocol             = 1;
        public string   id                   = string.Empty;
        public string   expiresUtc           = string.Empty;
        public string   dllHash              = string.Empty;
        public string[] targets              = [];
        public double   launchAtSeconds      = 35;
        public double   legTimeoutSeconds    = 180;
        public double   setupTimeoutSeconds  = 180;
        public double   cancelAfterSeconds   = -1;
        public double   reengageDelaySeconds = 2;
        public string   startBody            = string.Empty;
        public float[]  startOffset          = [];
        public float[]  startVelocity        = [];
        public string   fault                = string.Empty;

        public void Validate(DateTime utcNow)
        {
            if (protocol != 1 || !Guid.TryParseExact(id, "N", out _))
            {
                throw new InvalidOperationException("Invalid automation protocol or run identifier.");
            }

            bool validExpiry = DateTime.TryParse(expiresUtc, CultureInfo.InvariantCulture, DateTimeStyles.RoundtripKind, out var expiry)
                && expiry.ToUniversalTime() > utcNow && expiry.ToUniversalTime() <= utcNow.AddHours(2);
            if (!validExpiry || string.IsNullOrEmpty(dllHash) || dllHash.Length != 64)
            {
                throw new InvalidOperationException("Automation request expired or lacks its build identity.");
            }

            if (targets == null || targets.Length == 0 || targets.Length > 12)
            {
                throw new InvalidOperationException("Specify between one and twelve navigation targets.");
            }

            foreach (string target in targets)
            {
                if (string.IsNullOrWhiteSpace(target) || target.Length > 80)
                {
                    throw new InvalidOperationException("Invalid navigation target.");
                }
            }

            bool bounded = InRange(launchAtSeconds, 0, 900)       &&
                           InRange(legTimeoutSeconds, 5, 600)     &&
                           InRange(setupTimeoutSeconds, 30, 1200) &&
                           InRange(cancelAfterSeconds, -1, 300)   &&
                           InRange(reengageDelaySeconds, 0.5, 15);
            if (!bounded)
            {
                throw new InvalidOperationException("Automation timing is outside its supported bounds.");
            }

            if (!string.IsNullOrEmpty(startBody))
            {
                ValidateVector(startOffset, 100000);
                ValidateVector(startVelocity, 2000);
            }

            bool supportedFault = string.IsNullOrEmpty(fault) || fault == "fuel-exhaustion" || fault == "autopilot-damage"
                || fault == "insufficient-thrust" || fault == "blocked-route";
            if (!supportedFault || (!string.IsNullOrEmpty(fault) && (targets.Length != 1 || cancelAfterSeconds >= 0 || legTimeoutSeconds < 90)))
            {
                throw new InvalidOperationException("A fault test requires one target, no timed cancellation, at least 90 seconds, and a supported fault name.");
            }
        }

        private static void ValidateVector(float[] vector, double limit)
        {
            if (vector is not { Length: 3 })
            {
                throw new InvalidOperationException("An anchored start needs three offset and velocity components.");
            }

            if (vector.Any(component => !InRange(component, -limit, limit)))
            {
                throw new InvalidOperationException("Invalid anchored start vector.");
            }
        }

        private static bool InRange(double value, double minimum, double maximum) => value >= minimum && value <= maximum;
    }
}
