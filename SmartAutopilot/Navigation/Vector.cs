using System;

namespace SmartAutopilot.Navigation;

internal readonly struct Vector
{
    public double LengthSquared => X * X + Y * Y + Z * Z;
    public double Length        => Math.Sqrt(LengthSquared);
    public Vector Unit          => Length > 1e-9 ? this / Length : default;
    public bool   IsFinite      => !double.IsNaN(LengthSquared) && !double.IsInfinity(LengthSquared);

    public readonly double X;
    public readonly double Y;
    public readonly double Z;

    public Vector(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    public Vector Limited(double maximum) => Length > maximum ? Unit * Math.Max(0, maximum) : this;

    public static double Dot(Vector a, Vector b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    public static Vector operator +(Vector a, Vector b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    public static Vector operator -(Vector a, Vector b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    public static Vector operator -(Vector value) => value * -1;

    public static Vector operator *(Vector value, double scale) => new(value.X * scale, value.Y * scale, value.Z * scale);

    public static Vector operator /(Vector value, double scale) => value * (1 / scale);
}
