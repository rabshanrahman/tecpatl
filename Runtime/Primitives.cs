// Copyright 2026 Deep Auras. Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;

namespace Tecpatl
{
    /// <summary>
    /// One observation of a tracked limb (e.g. a hand), one per tracker result. Vectors are float[3] in a
    /// world-fixed frame, metres. Optional fields may be null (or NaN for Contact).
    /// </summary>
    public struct MotionSample
    {
        public float Time;
        /// <summary>Point whose movement makes a stroke (e.g. the index fingertip).</summary>
        public float[] Point;
        /// <summary>Point that moves only when the whole limb moves (e.g. the wrist). Separates limb motion from finger motion.</summary>
        public float[] Body;
        /// <summary>Orientation normal (e.g. the palm normal), for turns.</summary>
        public float[] Normal;
        /// <summary>Unit axes of the plane strokes are measured in (e.g. head-yaw right and world up). Depth is ignored.</summary>
        public float[] Right, Up;
        /// <summary>Contact distance ratio (e.g. thumb-index distance / palm length); NaN when not measured.</summary>
        public float Contact;
        /// <summary>Per-result static label from any classifier (unsmoothed); null when none.</summary>
        public string Pose;
    }

    /// <summary>A completed stroke: the Point moving from motion onset to stop.</summary>
    public struct Stroke
    {
        public float Start, End, TurnDeg;
        /// <summary>In-plane displacement (right, up) of the Point and of the Body, onset to end, metres.</summary>
        public float PointX, PointY, BodyX, BodyY;
        public string PoseAtStart, PoseAtEnd;
        public bool ContactAtStart, ContactAtEnd;
        /// <summary>Point position (world frame) at motion onset: where the stroke started.</summary>
        public float[] Origin;

        public float PointTravel => (float)Math.Sqrt(PointX * PointX + PointY * PointY);
        public float BodyTravel => (float)Math.Sqrt(BodyX * BodyX + BodyY * BodyY);

        /// <summary>Dominant in-plane direction of the Point's displacement.</summary>
        public Direction Direction => Math.Abs(PointX) >= Math.Abs(PointY)
            ? (PointX > 0 ? Direction.Right : Direction.Left)
            : (PointY > 0 ? Direction.Up : Direction.Down);
    }

    [Flags]
    public enum Direction { None = 0, Left = 1, Right = 2, Up = 4, Down = 8, Any = Left | Right | Up | Down }

    /// <summary>Thresholds of the primitives. Defaults are measured on one user's recordings; per-user values come from calibration.</summary>
    public sealed class PrimitiveSettings
    {
        /// <summary>Identical consecutive labels before a pose counts as stable.</summary>
        public int PoseRun = 4;
        /// <summary>Contact hysteresis on the Contact ratio: on below ContactOn, off above ContactOff.</summary>
        public float ContactOn = 0.15f, ContactOff = 0.30f;
        /// <summary>A stable pose that also counts as contact (e.g. a pinch label); null for none.</summary>
        public string ContactPose;
        /// <summary>A stroke is confirmed when the Point's net in-plane displacement over StrokeWindow s exceeds StrokeConfirm m.</summary>
        public float StrokeWindow = 0.8f, StrokeConfirm = 0.106f;
        /// <summary>Speed (m/s) below which the Point counts as still: the stroke starts at the last still sample and ends after two.</summary>
        public float StillSpeed = 0.08f;
        public float MaxStroke = 1.5f, MinStroke = 0.053f;
        /// <summary>Tracking gaps up to Gap s are bridged; a longer gap ends the stroke at the last sample and resets state.</summary>
        public float Gap = 0.7f;
        /// <summary>No new stroke for Refractory s after one ends (the return movement is not a stroke).</summary>
        public float Refractory = 1.0f;
    }

    /// <summary>Stable-pose primitive: a label becomes stable after PoseRun identical consecutive results.</summary>
    public sealed class PoseStabilizer
    {
        public string Stable { get; private set; }
        public float StableSince { get; private set; } = -1f;
        string m_Raw; int m_Run;

        /// <summary>Returns true when the stable pose changed on this sample.</summary>
        public bool Push(float t, string raw, int run)
        {
            m_Run = raw == m_Raw ? m_Run + 1 : 1;
            m_Raw = raw;
            if (m_Run < run || Stable == raw) return false;
            Stable = raw; StableSince = t;
            return true;
        }
    }

    /// <summary>Contact primitive: hysteresis on a distance ratio.</summary>
    public sealed class ContactDetector
    {
        public bool On { get; private set; }
        public bool Push(float ratio, float on, float off)
        {
            if (!float.IsNaN(ratio)) On = On ? ratio <= off : ratio < on;
            return On;
        }
    }

    /// <summary>
    /// Stroke primitive. Confirmed by net displacement over a trailing window (jitter cancels, a stroke accumulates);
    /// anchored at motion onset (the last still sample before confirmation); ended when the Point is still for two
    /// samples, after MaxStroke, or when tracking is lost.
    /// </summary>
    public sealed class StrokeDetector
    {
        struct Entry { public MotionSample S; public float Speed; public string Pose; public bool Contact; }

        readonly List<Entry> m_Hist = new List<Entry>();
        int m_Onset = -1, m_Slow;
        float m_QuietUntil = float.NegativeInfinity;

        public bool Active => m_Onset >= 0;
        public float LastTime => m_Hist.Count > 0 ? m_Hist[m_Hist.Count - 1].S.Time : float.NegativeInfinity;

        public void Reset() { m_Hist.Clear(); m_Onset = -1; m_Slow = 0; m_QuietUntil = float.NegativeInfinity; }

        /// <summary>Adds a sample (with the caller's stable pose and contact state); returns a stroke when one ends.</summary>
        public Stroke? Push(MotionSample s, string stablePose, bool contact, PrimitiveSettings p)
        {
            float speed = 0f;
            if (m_Hist.Count > 0)
            {
                var prev = m_Hist[m_Hist.Count - 1].S;
                float dt = s.Time - prev.Time;
                if (dt > 0.005f) speed = InPlane(Vec.Sub(s.Point, prev.Point), prev).len / dt;
            }
            m_Hist.Add(new Entry { S = s, Speed = speed, Pose = stablePose, Contact = contact });
            int i = m_Hist.Count - 1;
            Stroke? done = null;

            if (m_Onset < 0)
            {
                if (s.Time >= m_QuietUntil)
                {
                    int j = i;
                    while (j > 0 && s.Time - m_Hist[j - 1].S.Time <= p.StrokeWindow) j--;
                    var a = m_Hist[j].S;
                    if (s.Time - a.Time >= 0.6f * p.StrokeWindow && InPlane(Vec.Sub(s.Point, a.Point), a).len > p.StrokeConfirm)
                    {
                        int k = i, trailing = 0;
                        while (k > j && m_Hist[k].Speed < p.StillSpeed) { k--; trailing++; }   // motion may have just ended
                        while (k > j && m_Hist[k].Speed >= p.StillSpeed) k--;                  // back to the last still sample
                        m_Onset = k; m_Slow = trailing;
                        if (m_Slow >= 2) done = Finish(i - m_Slow + 1, s.Time, p);
                    }
                }
            }
            else
            {
                m_Slow = speed < p.StillSpeed ? m_Slow + 1 : 0;
                if (m_Slow >= 2 || s.Time - m_Hist[m_Onset].S.Time > p.MaxStroke)
                    done = Finish(m_Slow >= 2 ? i - 1 : i, s.Time, p);
            }
            Trim(p);
            return done;
        }

        /// <summary>Tracking lost: ends an active stroke at the last sample.</summary>
        public Stroke? Lost(float now, PrimitiveSettings p) => m_Onset >= 0 ? Finish(m_Hist.Count - 1, now, p) : null;

        Stroke? Finish(int end, float now, PrimitiveSettings p)
        {
            var a = m_Hist[m_Onset]; var b = m_Hist[end];
            float turn = 0f;
            if (a.S.Normal != null)
                for (int k = m_Onset; k <= end; k++)
                    if (m_Hist[k].S.Normal != null) turn = Math.Max(turn, Vec.AngleDeg(a.S.Normal, m_Hist[k].S.Normal));
            var pt = InPlane(Vec.Sub(b.S.Point, a.S.Point), a.S);
            var bd = a.S.Body != null && b.S.Body != null ? InPlane(Vec.Sub(b.S.Body, a.S.Body), a.S) : (x: 0f, y: 0f, len: 0f);
            var st = new Stroke
            {
                Start = a.S.Time, End = b.S.Time, TurnDeg = turn,
                PointX = pt.x, PointY = pt.y, BodyX = bd.x, BodyY = bd.y,
                PoseAtStart = a.Pose, PoseAtEnd = b.Pose, ContactAtStart = a.Contact, ContactAtEnd = b.Contact,
                Origin = (float[])a.S.Point.Clone(),
            };
            m_Onset = -1;
            m_QuietUntil = now + p.Refractory;
            return st;   // strokes shorter than MinStroke are still returned (diagnostics); GestureChain ignores them
        }

        void Trim(PrimitiveSettings p)
        {
            float t = m_Hist[m_Hist.Count - 1].S.Time, keep = p.StrokeWindow + p.MaxStroke + 0.5f;
            int drop = 0;
            while (drop < m_Hist.Count - 1 && t - m_Hist[drop].S.Time > keep && (m_Onset < 0 || drop < m_Onset)) drop++;
            if (drop == 0) return;
            m_Hist.RemoveRange(0, drop);
            if (m_Onset >= 0) m_Onset -= drop;
        }

        static (float x, float y, float len) InPlane(float[] d, MotionSample frame)
        {
            float x = Vec.Dot(d, frame.Right), y = Vec.Dot(d, frame.Up);
            return (x, y, (float)Math.Sqrt(x * x + y * y));
        }
    }
}
