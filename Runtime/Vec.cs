// Copyright 2026 Deep Auras. Licensed under the Apache License, Version 2.0.
using System;

namespace Tecpatl
{
    /// <summary>Small float[] helpers (no engine types).</summary>
    internal static class Vec
    {
        public static float Dot(float[] a, float[] b)
        {
            float s = 0f;
            for (int i = 0; i < a.Length; i++) s += a[i] * b[i];
            return s;
        }

        public static float Length(float[] a) => (float)Math.Sqrt(Dot(a, a));

        public static float Dist(float[] a, float[] b)
        {
            float s = 0f;
            for (int i = 0; i < a.Length; i++) { float d = a[i] - b[i]; s += d * d; }
            return (float)Math.Sqrt(s);
        }

        /// <summary>Returns a unit-length copy; a zero vector stays zero.</summary>
        public static float[] Normalized(float[] a)
        {
            float len = Length(a);
            var r = new float[a.Length];
            if (len > 1e-12f) for (int i = 0; i < a.Length; i++) r[i] = a[i] / len;
            return r;
        }

        public static float[] Sub(float[] a, float[] b)
        {
            var r = new float[a.Length];
            for (int i = 0; i < a.Length; i++) r[i] = a[i] - b[i];
            return r;
        }
    }
}
