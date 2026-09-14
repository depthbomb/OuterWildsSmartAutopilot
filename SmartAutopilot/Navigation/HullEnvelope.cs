using System;

namespace SmartAutopilot.Navigation
{
    internal static class HullEnvelope
    {
        public static double BoxRadius(Vector centerOffset, Vector halfX, Vector halfY, Vector halfZ)
        {
            double radiusSquared = 0;
            for (int corner = 0; corner < 8; corner++)
            {
                var point = centerOffset + halfX * ((corner & 1) == 0 ? -1 : 1)
                    + halfY * ((corner & 2) == 0 ? -1 : 1) + halfZ * ((corner & 4) == 0 ? -1 : 1);
                radiusSquared = Math.Max(radiusSquared, point.LengthSquared);
            }

            return Math.Sqrt(radiusSquared);
        }
    }
}
