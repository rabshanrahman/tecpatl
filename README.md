# tecpatl
Open source motion-gestures for hand tracking

Jacknife is licensed for academic shit sniffers and has a stupid fucking patent. This is for the rest of us.

Few-example dynamic gesture recognizer for any point-sequence input (pen, hand joints, body joints,
controllers). Pure C#, no engine dependencies, usable as a Unity package or plain .NET library.

* 1-nearest-neighbour dynamic time warping on gesture-path **direction vectors** with an inner-product cost
* correction factors (component-wise absolute distance, bounding box)
* **1–2 examples per gesture** are enough to start
* rejection thresholds calibrated from **real** non-gesture recordings
* continuous (unsegmented) recognition with buffer, cadence and cooldown

Method: clean-room implementation from the published description in Taranta et al., *Jackknife*, CHI 2017
(see `SPEC.md`). Synthetic sample generation is intentionally not implemented.

## Install (Unity)

`Packages/manifest.json`:

```json
"com.deepauras.tecpatl": "file:../../path/to/tecpatl"
```

or a git URL once published.

## Use

```csharp
var rec = new Tecpatl.Recognizer(resample: 16, band: 2);
rec.AddTemplate("swipe_left", exampleFrames);   // float[][] : N frames of m components
rec.AddTemplate("swipe_left", anotherExample);
rec.Calibrate(positives, negatives);            // real extra examples and real non-gesture windows (optional)

var r = rec.Classify(queryFrames);              // r.Label, r.Score, r.Accepted
```

Continuous stream:

```csharp
var stream = new Tecpatl.ContinuousRecognizer(rec, windowSeconds: 4f, cadence: 10, cooldownSeconds: 2f);
stream.OnGesture += label => ...;
stream.Push(time, frame);                        // every frame
```

License: Apache-2.0.
