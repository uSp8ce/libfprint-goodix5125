/*
 * Goodix 27c6:5125 driver for libfprint
 * Copyright (C) 2026 The libfprint-goodix5125 contributors
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/* Loads the Goodix engine from $GOODIX5125_ENGINE_DLL; skips (77) when unset,
 * because the DLL is proprietary and never part of this repository. */

#include <glib.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "drivers/goodixtls/goodix_engine.h"

/* Deterministic synthetic ridge fields; no person's fingerprint is used.
 * A gallery/probe round trip exercises actual feature construction and catches
 * ABI mistakes that a successful DLL load or enroll_start cannot detect. */
static void
make_pattern (guint8 out[5120], gboolean different)
{
  for (int y = 0; y < 80; y++)
    for (int x = 0; x < 64; x++)
      {
        double xx = x - 31.0, yy = y - 38.0;
        double phase = different ?
          0.79 * yy + 0.009 * xx * xx + 2.8 * atan2 (xx, yy - 9.0) :
          0.82 * xx + 0.007 * yy * yy + 2.2 * atan2 (yy, xx + 12.0);
        out[y * 64 + x] = (guint8) (128.0 + 85.0 * sin (phase));
      }
}

static void
test_fresh_probe_roundtrip (void)
{
  guint8 enrolled[5120], different[5120], blank[5120] = {0};
  guint8 *blob = NULL;
  size_t length = 0;
  int count = 0, progress = 0, max_images = 0, score = 0, index = -1;
  void *ctx;

  make_pattern (enrolled, FALSE);
  make_pattern (different, TRUE);
  ctx = goodix_engine_enroll_start (&max_images);
  g_assert_nonnull (ctx);
  for (int i = 0; i < 12; i++)
    g_assert_cmpint (goodix_engine_enroll_add_image (ctx, enrolled, 64, 80,
                                                    &count, &progress), ==, 0);
  g_assert_cmpint (count, ==, 12);
  g_assert_cmpint (progress, ==, 100);
  g_assert_cmpint (goodix_engine_enroll_commit (ctx, &blob, &length), ==, 0);
  g_assert_nonnull (blob);
  g_assert_cmpuint (length, >, 0);
  goodix_engine_enroll_finish (ctx);

  /* Repeat after different and blank probes to detect stale-feature reuse. */
  for (int round = 0; round < 3; round++)
    {
      g_assert_cmpint (goodix_engine_verify_image (enrolled, 64, 80, blob, length, &score), ==, 1);
      g_assert_cmpint (score, >=, 30);
      g_assert_cmpint (goodix_engine_verify_image (different, 64, 80, blob, length, &score), ==, 0);
      g_assert_cmpint (goodix_engine_verify_image (blank, 64, 80, blob, length, &score), ==, 0);
    }
  const guint8 *gallery[] = {blob};
  g_assert_cmpint (goodix_engine_identify_image (enrolled, 64, 80, gallery, &length,
                                                1, &index, &score), ==, 1);
  g_assert_cmpint (index, ==, 0);
  g_assert_cmpint (score, >=, 30);
  g_assert_cmpint (goodix_engine_identify_image (different, 64, 80, gallery, &length,
                                                1, &index, &score), ==, 0);
  g_assert_cmpint (index, ==, -1);
  free (blob);
}

int
main (int argc, char *argv[])
{
  const char *dll = g_getenv ("GOODIX5125_ENGINE_DLL");
  g_autoptr(GError) error = NULL;
  int max_images = 0;
  void *ctx;

  if (!dll || !*dll)
    {
      g_print ("GOODIX5125_ENGINE_DLL not set, skipping\n");
      return 77;
    }

  if (!goodix_engine_init (dll, &error))
    {
      g_printerr ("engine init failed: %s\n", error->message);
      return 1;
    }
  if (!g_str_has_prefix (goodix_engine_get_version (), "Milan_v_"))
    {
      g_printerr ("unexpected engine version '%s'\n", goodix_engine_get_version ());
      return 1;
    }
  ctx = goodix_engine_enroll_start (&max_images);
  if (!ctx || max_images != 16)
    {
      g_printerr ("enroll_start: ctx=%p max_images=%d\n", ctx, max_images);
      return 1;
    }
  goodix_engine_enroll_finish (ctx);
  test_fresh_probe_roundtrip ();
  g_print ("engine %s ok\n", goodix_engine_get_version ());
  return 0;
}
