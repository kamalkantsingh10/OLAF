"""Heart emotion logic for the chest app (Story 8.4) — pure, pygame-free.

The chest display is an independent consumer of the companion's expression
interface (one-producer → many-bodies): it reads the SAME `expression_map.yaml`
`heart:` blocks the engine's map uses, and reacts to the SAME `activity` +
`speech_emotion` events. No engine changes, no new topics.

  * `load_heart_config(map_path)` — pull the `heart:{image,bpm,amplitude}` blocks
    out of the expression map (speech_emotion + activity incl. working submodes +
    default).
  * `HeartConfig.target(...)` — the active heart: the speech_emotion's heart while
    speaking, else the current activity's heart, else the default.
  * `HeartDirector.update(dt, ...)` — eases bpm/amplitude toward the target (so the
    heart eases in/out smoothly) and switches the image discretely.

Rendering (the beat) stays in `widgets/heart.py`; this just decides image/bpm/amplitude.
"""

from __future__ import annotations

HEART_EASE_S = 0.6   # bpm/amplitude ease time-constant (smooth in/out)
DEFAULT_HEART = {"image": "calm", "bpm": 25.0, "amplitude": 0.15}  # boot = asleep (calm img, slowest bpm); companion awakens


def smooth_damp(current, target, velocity, smooth_time, dt):
    """Critically-damped approach (no overshoot). Returns (value, velocity)."""
    smooth_time = max(1e-4, smooth_time)
    if dt <= 0:
        return current, velocity
    omega = 2.0 / smooth_time
    x = omega * dt
    exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)
    change = current - target
    temp = (velocity + omega * change) * dt
    new_vel = (velocity - omega * temp) * exp
    new = target + (change + temp) * exp
    if (target - current > 0.0) == (new > target):
        new = target
        new_vel = (new - current) / dt
    return new, new_vel


class HeartConfig:
    """The per-emotion / per-activity heart blocks read from the map."""

    def __init__(self, speech: dict, activity: dict, default: dict):
        self.speech = speech        # {emotion_name: {image,bpm,amplitude}}
        self.activity = activity    # {state or "working.<submode>": {...}}
        self.default = default

    def target(self, activity_state=None, working_submode=None, speech_emotion=None) -> dict:
        """The active heart spec. While the activity is ``speaking`` and a
        speech_emotion is known, the emotion's heart wins; otherwise the
        current activity's heart (sleeping/listening/working/...) ; else default."""
        base = self.default
        if activity_state == "working" and working_submode:
            base = self.activity.get(f"working.{working_submode}", base)
        elif activity_state:
            base = self.activity.get(activity_state, base)

        if activity_state == "speaking" and speech_emotion in self.speech:
            return self.speech[speech_emotion]
        return base


def load_heart_config(map_path) -> HeartConfig:
    """Read the heart blocks out of expression_map.yaml."""
    import yaml

    with open(map_path) as fh:
        raw = yaml.safe_load(fh)

    speech = {
        k: v["heart"]
        for k, v in (raw.get("speech_emotion") or {}).items()
        if isinstance(v, dict) and isinstance(v.get("heart"), dict)
    }
    activity = {}
    for k, v in (raw.get("activity") or {}).items():
        if k == "working" and isinstance(v, dict):
            for sk, sv in v.items():
                if isinstance(sv, dict) and isinstance(sv.get("heart"), dict):
                    activity[f"working.{sk}"] = sv["heart"]
        elif isinstance(v, dict) and isinstance(v.get("heart"), dict):
            activity[k] = v["heart"]
    default = (raw.get("defaults") or {}).get("heart") or dict(DEFAULT_HEART)
    return HeartConfig(speech, activity, default)


class HeartDirector:
    """Holds the current eased heart state; `update` returns (image, bpm, amplitude)."""

    def __init__(self, config: HeartConfig, ease: float = HEART_EASE_S):
        self.cfg = config
        self.ease = ease
        d = config.default
        self.image = str(d.get("image", "calm"))
        self.bpm = float(d.get("bpm", 25.0))
        self.amp = float(d.get("amplitude", 0.15))
        self._bv = 0.0
        self._av = 0.0

    def update(self, dt, activity_state=None, working_submode=None, speech_emotion=None):
        t = self.cfg.target(activity_state, working_submode, speech_emotion)
        self.image = str(t.get("image", self.image))
        self.bpm, self._bv = smooth_damp(self.bpm, float(t.get("bpm", self.bpm)), self._bv, self.ease, dt)
        self.amp, self._av = smooth_damp(self.amp, float(t.get("amplitude", self.amp)), self._av, self.ease, dt)
        return self.image, self.bpm, self.amp
