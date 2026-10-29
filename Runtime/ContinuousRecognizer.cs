// Copyright 2026 Deep Auras. Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;

namespace Tecpatl
{
    /// <summary>
    /// Unsegmented recognition over a stream (SPEC §10): keeps the last WindowSeconds of frames, classifies
    /// every Cadence frames, emits a gesture when two consecutive runs agree and the result is accepted,
    /// then clears the buffer and waits CooldownSeconds.
    /// </summary>
    public sealed class ContinuousRecognizer
    {
        public readonly Recognizer Recognizer;
        public float WindowSeconds, CooldownSeconds;
        public int Cadence;
        public int MinFrames = 8;
        /// <summary>Consecutive runs that must agree (same accepted label) before a gesture is emitted.</summary>
        public int Agreement = 3;
        /// <summary>
        /// Tail lengths (seconds) of the buffer to classify on every run; set these to the expected gesture
        /// durations (e.g. 0.4 s for a flick, 1.5 s for a circle). Fractions of the buffer would depend on the
        /// buffer size instead of the gesture.
        /// </summary>
        public float[] TailSeconds = { 0.4f, 0.8f, 1.5f };

        public event Action<float, string, float> OnGesture;   // (time, label, score)
        public event Action<string> OnDebug;                    // per-run diagnostics (optional)

        readonly List<(float t, float[] f)> m_Buf = new List<(float, float[])>();
        int m_Count;
        string m_Last;
        int m_Streak;
        float m_Suspended = float.NegativeInfinity;

        public ContinuousRecognizer(Recognizer recognizer, float windowSeconds = 4f, int cadence = 10, float cooldownSeconds = 2f)
        {
            Recognizer = recognizer; WindowSeconds = windowSeconds; Cadence = cadence; CooldownSeconds = cooldownSeconds;
        }

        public void Reset() { m_Buf.Clear(); m_Count = 0; m_Last = null; m_Streak = 0; }

        public void Push(float t, float[] frame)
        {
            m_Buf.Add((t, frame));
            while (m_Buf.Count > 0 && t - m_Buf[0].t > WindowSeconds) m_Buf.RemoveAt(0);
            if (t < m_Suspended) return;
            if (++m_Count % Cadence != 0 || m_Buf.Count < MinFrames) return;

            Result best = new Result { Score = float.PositiveInfinity };
            foreach (float secs in TailSeconds)
            {
                int n = 0;
                for (int i = m_Buf.Count - 1; i >= 0 && t - m_Buf[i].t <= secs; i--) n++;
                n = Math.Max(MinFrames, n);
                if (n > m_Buf.Count) continue;
                var frames = new float[n][];
                for (int i = 0; i < n; i++) frames[i] = m_Buf[m_Buf.Count - n + i].f;
                var r = Recognizer.Classify(frames);
                if (r.Accepted && r.Score < best.Score) best = r;
            }
            OnDebug?.Invoke($"t={t:F2} buf={m_Buf.Count} best={best.Label} score={best.Score:F3} accepted={best.Accepted} last={m_Last}");
            if (!best.Accepted) { m_Last = null; m_Streak = 0; return; }
            m_Streak = best.Label == m_Last ? m_Streak + 1 : 1;
            m_Last = best.Label;
            if (m_Streak >= Agreement)
            {
                OnGesture?.Invoke(t, best.Label, best.Score);
                m_Buf.Clear();
                m_Last = null; m_Streak = 0;
                m_Suspended = t + CooldownSeconds;
            }
        }
    }
}
