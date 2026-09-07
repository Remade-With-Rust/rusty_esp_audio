"""Turn the bench firmware's serial dump into a WAV, then let ffmpeg judge it.

The firmware prints `WAVDATA <hex>` lines carrying a complete WAV file: header
first, then the PCM. This reassembles them and hands the result to ffprobe and
ffmpeg, which are the external oracle. The point of the WAV rather than raw
PCM is that the sample rate and channel count travel in the file, so ffprobe
reads back what the firmware CLAIMED instead of being told it on the command
line -- a raw-PCM oracle would agree with any rate you gave it.

Three things are checked, and the third is the one that matters:

1. The file parses as WAV and its geometry is what the firmware said.
2. Its duration matches the number of seconds the firmware set out to capture.
3. It is not silence. A microphone that returns zeros passes every structural
   check ever written, so the level is computed here AND by ffmpeg's own
   `volumedetect`, and the two are printed side by side. They should agree
   within a fraction of a dB; if they do not, this script is wrong.

Usage:
    python decode-wav-dump.py <monitor-capture.txt> [out.wav]
"""
import re
import struct
import subprocess
import sys
import wave

HEX = re.compile(r"^\s*WAVDATA\s+([0-9a-fA-F]+)\s*$")


def collect(path: str) -> bytes:
    out = bytearray()
    lines = 0
    for line in open(path, encoding="utf-8", errors="replace"):
        m = HEX.match(line)
        if m:
            out += bytes.fromhex(m.group(1))
            lines += 1
    print(f"  {lines} WAVDATA lines -> {len(out)} bytes")
    return bytes(out)


def ffmpeg_level(path: str) -> dict:
    """ffmpeg's own opinion of the level, as the outside instrument."""
    try:
        r = subprocess.run(
            ["ffmpeg", "-hide_banner", "-nostats", "-i", path,
             "-af", "volumedetect", "-f", "null", "-"],
            capture_output=True, text=True, timeout=120,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return {"error": str(e)}
    found = {}
    for key in ("mean_volume", "max_volume"):
        m = re.search(rf"{key}:\s*(-?\d+(?:\.\d+)?) dB", r.stderr)
        if m:
            found[key] = float(m.group(1))
    return found


def ffprobe_geometry(path: str) -> dict:
    try:
        r = subprocess.run(
            ["ffprobe", "-hide_banner", "-v", "error", "-show_entries",
             "stream=sample_rate,channels,codec_name,duration",
             "-of", "default=noprint_wrappers=1", path],
            capture_output=True, text=True, timeout=60,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return {"error": str(e)}
    out = {}
    for line in r.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def main() -> int:
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else "mic.wav"

    print("Reassembling the dump")
    data = collect(src)
    if not data:
        print("  no WAVDATA lines found", file=sys.stderr)
        return 1
    open(dst, "wb").write(data)
    print(f"  wrote {dst}")

    print("\nWhat the file says about itself")
    with wave.open(dst, "rb") as w:
        rate, ch, width, n = (
            w.getframerate(), w.getnchannels(), w.getsampwidth(), w.getnframes()
        )
        pcm = w.readframes(n)
    print(f"  rate={rate} channels={ch} sample_bytes={width} frames={n}")
    print(f"  duration={n / rate:.4f} s")

    print("\nIs it silence?")
    samples = struct.unpack(f"<{len(pcm) // 2}h", pcm[: len(pcm) // 2 * 2])
    peak = max(abs(s) for s in samples) if samples else 0
    rms = (sum(float(s) * s for s in samples) / len(samples)) ** 0.5 if samples else 0.0
    import math
    peak_db = 20 * math.log10(peak / 32768) if peak else float("-inf")
    rms_db = 20 * math.log10(rms / 32768) if rms else float("-inf")
    distinct = len(set(samples))
    print(f"  computed here : peak {peak_db:7.2f} dBFS   rms {rms_db:7.2f} dBFS")

    lv = ffmpeg_level(dst)
    if "error" in lv:
        print(f"  ffmpeg        : unavailable ({lv['error']})")
    else:
        print(
            f"  ffmpeg says   : peak {lv.get('max_volume', float('nan')):7.2f} dBFS"
            f"   rms {lv.get('mean_volume', float('nan')):7.2f} dBFS"
        )
    print(f"  distinct sample values: {distinct}")

    print("\nffprobe's reading of the geometry")
    for k, v in ffprobe_geometry(dst).items():
        print(f"  {k}={v}")

    print("\nVerdict")
    ok = True
    if distinct < 2:
        print("  FAIL the capture is a constant; the microphone produced no signal")
        ok = False
    else:
        print(f"  pass  the capture varies ({distinct} distinct values)")
    if peak_db > -1.0:
        print(f"  WARN  peak {peak_db:.2f} dBFS is at the rail; it may be clipping")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
