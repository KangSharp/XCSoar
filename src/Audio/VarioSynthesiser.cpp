// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "VarioSynthesiser.hpp"
#include "Math/FastMath.hpp"

#include <algorithm>
#include <cassert>

/**
 * The minimum and maximum vario range for the constants below [cm/s].
 */
static constexpr int min_vario = -500, max_vario = 500;

/**
 * Duration of the fade-out in samples (when silence is triggered)
 */
static constexpr unsigned fade_out_samples = 1000;

unsigned
VarioSynthesiser::VarioToFrequency(int ivario)
{
  return ivario > 0
    ? (zero_frequency + (unsigned)ivario * (max_frequency - zero_frequency)
       / (unsigned)max_vario)
    : (zero_frequency - (unsigned)(ivario * (int)(zero_frequency - min_frequency) / min_vario));
}

void
VarioSynthesiser::SetVario(double vario)
{
  const std::lock_guard lock{mutex};

  const int ivario = std::clamp((int)(vario * 100), min_vario, max_vario);

  if (dead_band_enabled && InDeadBand(ivario)) {
    /* inside the "dead band" */
    UnsafeSetSilence();
    return;
  }

  /* update the ToneSynthesiser base class */
  SetTone(VarioToFrequency(ivario));

  if (ivario > 0) {
    /* while climbing, the vario sound gets interrupted by silence
       periodically */

    const unsigned period_ms = sample_rate
      * (min_period_ms + (max_vario - ivario)
         * (max_period_ms - min_period_ms) / max_vario)
      / 1000;

    silence_count = period_ms / 3;
    audible_count = period_ms - silence_count;

    /* preserve the old "_remaining" values as much as possible, to
       avoid chopping off the previous tone */

    if (audible_remaining > audible_count)
      audible_remaining = audible_count;

    if (silence_remaining > silence_count)
      silence_remaining = silence_count;
  } else {
    /* continuous tone while sinking */
    audible_count = 1;
    silence_count = 0;
  }
}

void
VarioSynthesiser::SetSilence()
{
  const std::lock_guard lock{mutex};
  UnsafeSetSilence();
}

void
VarioSynthesiser::UnsafeSetSilence()
{
  audible_count = 0;
  silence_count = 1;

  if (audible_remaining > 0)
    /* fade out the current period smoothly */
    audible_remaining = fade_out_samples;

  silence_remaining = 0;
}

void
VarioSynthesiser::FadeOut(int16_t *buffer, size_t n)
{
  /* Generate a fade-out period over 'fade_out_samples' samples */
  for (size_t i = 0; i < n; ++i) {
    float fade_factor = 1.0f - (float)i / (float)fade_out_samples;
    buffer[i] = (int16_t)(buffer[i] * fade_factor);  // apply fade
  }
}

void
VarioSynthesiser::Synthesise(int16_t *buffer, size_t n)
{
  const std::lock_guard lock{mutex};

  assert(audible_count > 0 || silence_count > 0);

  if (silence_count == 0) {
    /* magic value for "continuous tone" */
    ToneSynthesiser::Synthesise(buffer, n);
    return;
  }

  while (n > 0) {
    if (audible_remaining > 0) {
      /* generate a period of audible tone */

      unsigned o = silence_count > 0
        ? std::min(n, audible_remaining)
        : n;

      ToneSynthesiser::Synthesise(buffer, o);

      if (audible_remaining <= fade_out_samples) {
        /* apply fade-out during the last phase of the tone */
        FadeOut(buffer, o);
      }

      buffer += o;
      n -= o;
      audible_remaining -= o;

      if (audible_remaining == 0 && silence_remaining > 0) {
        /* finished fade-out, now we can emit a period of silence */
        Restart();
      }
    } else if (silence_remaining > 0) {
      /* generate a period of silence (climbing) */

      unsigned o = audible_count > 0
        ? std::min(n, silence_remaining)
        : n;
      /* the "silence" PCM sample value is zero */
      std::fill_n(buffer, o, 0);
      buffer += o;
      n -= o;
      silence_remaining -= o;
    } else {
      /* period finished, begin next one */

      audible_remaining = audible_count;
      silence_remaining = silence_count;
    }
  }
}
