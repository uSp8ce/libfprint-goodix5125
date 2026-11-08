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

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <gio/gio.h>

#include "goodix5125_calib.h"

/* ChicagoHS type-12 register table, from Rockytkg/goodix-linux-27c6-5125.
 * GF_ST411SEC_APP_12508 uses the native 5120-pixel transfer. */
const guint8 g5125_config_12508[224] = {
    0x70, 0x11, 0x74, 0x85, 0x00, 0x85, 0x2c, 0xb1, 0x18, 0xc9, 0x14, 0xdd, 0x00, 0xdd, 0x00, 0xdd,
    0x00, 0xba, 0x00, 0x01, 0x80, 0xca, 0x00, 0x04, 0x00, 0x84, 0x00, 0x15, 0xb3, 0x86, 0x00, 0x00,
    0xc4, 0x88, 0x00, 0x00, 0xba, 0x8a, 0x00, 0x00, 0xb2, 0x8c, 0x00, 0x00, 0xaa, 0x8e, 0x00, 0x00,
    0xc1, 0x90, 0x00, 0xbb, 0xbb, 0x92, 0x00, 0xb1, 0xb1, 0x94, 0x00, 0x00, 0xa8, 0x96, 0x00, 0x00,
    0xb6, 0x98, 0x00, 0x00, 0x00, 0x9a, 0x00, 0x00, 0x00, 0xd2, 0x00, 0x00, 0x00, 0xd4, 0x00, 0x00,
    0x00, 0xd6, 0x00, 0x00, 0x00, 0xd8, 0x00, 0x00, 0x00, 0x50, 0x00, 0x01, 0x05, 0xd0, 0x00, 0x00,
    0x00, 0x70, 0x00, 0x00, 0x00, 0x72, 0x00, 0x78, 0x56, 0x74, 0x00, 0x34, 0x12, 0x20, 0x00, 0x10,
    0x40, 0x5c, 0x00, 0x80, 0x01, 0x20, 0x02, 0x08, 0x08, 0x36, 0x02, 0x80, 0x00, 0x38, 0x02, 0x80,
    0x00, 0x3a, 0x02, 0x80, 0x00, 0x2a, 0x01, 0x82, 0x03, 0x22, 0x00, 0x01, 0x20, 0x24, 0x00, 0x14,
    0x00, 0x80, 0x00, 0x01, 0x00, 0x5c, 0x00, 0x00, 0x01, 0x56, 0x00, 0x04, 0x20, 0x58, 0x00, 0x03,
    0x02, 0x32, 0x00, 0x0c, 0x02, 0x66, 0x00, 0x03, 0x00, 0x7c, 0x00, 0x00, 0x58, 0x82, 0x00, 0x80,
    0x15, 0x2a, 0x01, 0x08, 0x00, 0x54, 0x00, 0x10, 0x01, 0x62, 0x00, 0x04, 0x03, 0x64, 0x00, 0x19,
    0x00, 0x66, 0x00, 0x03, 0x00, 0x7c, 0x00, 0x00, 0x58, 0x2a, 0x01, 0x08, 0x00, 0x52, 0x00, 0x08,
    0x00, 0x54, 0x00, 0x00, 0x01, 0x66, 0x00, 0x03, 0x00, 0x7c, 0x00, 0x00, 0x58, 0x00, 0x00, 0x00,
};

const guint16 g5125_dac_regs[4] = { 0x0220, 0x0236, 0x0238, 0x023a };

/* Start base for the first FDT-manual measurement (goodix-fp-dump
 * driver_51x0.py, MIT); g5125_fdt_payload() prepends op byte and 0x01. */
const guint8 g5125_fdt_start_base[G5125_FDT_BASE_LEN] = {
  0xae, 0xae, 0xbf, 0xbf, 0xa4, 0xa4, 0xb8, 0xb8, 0xa8, 0xa8, 0xb7, 0xb7,
};

static guint8
crc8 (const guint8 *data, gsize len)
{
  guint8 crc = 0;

  for (gsize i = 0; i < len; i++)
    {
      crc ^= data[i];
      for (int b = 0; b < 8; b++)
        crc = (crc & 0x80) ? (guint8) ((crc << 1) ^ 0x07) : (guint8) (crc << 1);
    }
  return crc;
}

/* ~CRC-8 over the concatenation of the given [start, end) OTP ranges */
static guint8
otp_crc (const guint8 *otp, const guint8 (*ranges)[2], gsize n_ranges)
{
  guint8 buf[G5125_OTP_LEN];
  gsize len = 0;

  for (gsize r = 0; r < n_ranges; r++)
    for (guint8 i = ranges[r][0]; i < ranges[r][1]; i++)
      buf[len++] = otp[i];
  return (guint8) ~crc8 (buf, len);
}

gboolean
g5125_otp_check (const guint8 *otp, gsize len, GError **error)
{
  static const guint8 cp[][2] = { { 0, 11 }, { 36, 40 } };
  static const guint8 mt[][2] = { { 20, 28 }, { 29, 36 }, { 40, 50 }, { 54, 56 } };
  static const guint8 ft[][2] = { { 11, 20 }, { 28, 29 }, { 50, 54 }, { 56, 60 }, { 62, 63 } };
  static const guint8 dac_ft[][2] = { { 50, 54 } };
  static const guint8 dac_mt[][2] = { { 46, 50 } };

  if (len < G5125_OTP_LEN)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "OTP too short: %" G_GSIZE_FORMAT " bytes", len);
      return FALSE;
    }
  if (otp_crc (otp, cp, G_N_ELEMENTS (cp)) != otp[60] ||
      otp_crc (otp, mt, G_N_ELEMENTS (mt)) != otp[63] ||
      otp_crc (otp, ft, G_N_ELEMENTS (ft)) != otp[61] ||
      otp_crc (otp, dac_ft, 1) != otp[62] ||
      otp_crc (otp, dac_mt, 1) != otp[22] ||
      memcmp (otp + 46, otp + 50, 4) != 0)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "OTP checksum mismatch (unsupported sensor or corrupted OTP)");
      return FALSE;
    }
  return TRUE;
}

void
g5125_calib_from_otp (const guint8 *otp, G5125Calib *out)
{
  out->dac[0] = (guint16) (otp[46] << 4 | 8);
  out->dac[1] = otp[47];
  out->dac[2] = otp[48];
  out->dac[3] = otp[49];
  out->tcode = (guint16) (((otp[42] >> 4) + 1) * 16 + 64);
  out->delta_down = (guint8) (((int) (((otp[42] & 0xf) + 2) * 25600.0 / out->tcode / 3) >> 4) & 0xff);
}

guint16
g5125_config_checksum (const guint8 *cfg, gsize len)
{
  guint16 sum = 0xa5a5;

  for (gsize i = 0; i + 2 < len; i += 2)
    sum += (guint16) (cfg[i] | cfg[i + 1] << 8);
  return (guint16) (0x10000 - sum);
}

guint
g5125_config_patch (guint8 *cfg, gsize len, const G5125Calib *calib)
{
  guint patched = 0;
  guint16 cs;

  for (int sec = 0; sec < 8; sec++)
    {
      gsize base = cfg[1 + 2 * sec], size = cfg[2 + 2 * sec];

      for (gsize e = base; e + 3 < base + size && e + 3 < len - 2; e += 4)
        {
          guint16 tag = cfg[e] | cfg[e + 1] << 8;
          guint16 val = cfg[e + 2] | cfg[e + 3] << 8;
          gint nv = -1;

          if (len == sizeof g5125_config_12508)
            {
              if (sec == 0 && tag == 0x5c)
                nv = calib->tcode;
              for (guint i = 0; sec == 0 && i < 4; i++)
                if (tag == g5125_dac_regs[i])
                  nv = calib->dac[i];
              if (sec == 2 && tag == 0x82)
                nv = calib->delta_down << 8 | (val & 0xff);
            }
          else if (tag == 0x5c && val == 0x0100)
            nv = calib->tcode;
          else if (len != sizeof g5125_config_12508 && tag == 0x82 && val == 0x1f80)
            nv = calib->delta_down << 8 | 0x80;
          if (nv < 0)
            continue;
          cfg[e + 2] = nv & 0xff;
          cfg[e + 3] = nv >> 8;
          patched++;
        }
    }
  cs = g5125_config_checksum (cfg, len);
  cfg[len - 2] = cs & 0xff;
  cfg[len - 1] = cs >> 8;
  return patched;
}

gboolean
g5125_fdt_channels (const guint8 *payload, gsize len, guint16 raw[G5125_CHANNELS])
{
  if (len < 4 + 2 * G5125_CHANNELS)
    return FALSE;
  for (int c = 0; c < G5125_CHANNELS; c++)
    raw[c] = payload[4 + 2 * c] | payload[5 + 2 * c] << 8;
  return TRUE;
}

gboolean
g5125_fdt_base (const guint16 raw[G5125_CHANNELS], guint8 base[G5125_FDT_BASE_LEN])
{
  for (int c = 0; c < G5125_CHANNELS; c++)
    {
      guint8 v = (raw[c] >> 1) & 0xff;

      if (v == 0x00 || v == 0xff)
        return FALSE;
      base[2 * c] = 0x80;
      base[2 * c + 1] = v;
    }
  return TRUE;
}

guint16
g5125_irq_status (const guint8 *payload, gsize len)
{
  if (len < 2)
    return 0xffff;
  return payload[0] | payload[1] << 8;
}

gsize
g5125_fdt_payload (guint8 op, const guint8 base[G5125_FDT_BASE_LEN],
                   gboolean with_timestamp, guint16 timestamp, guint8 *out)
{
  gsize n = 0;

  out[n++] = op;
  out[n++] = 0x01;
  memcpy (out + n, base, G5125_FDT_BASE_LEN);
  n += G5125_FDT_BASE_LEN;
  if (with_timestamp)
    {
      out[n++] = timestamp & 0xff;
      out[n++] = timestamp >> 8;
    }
  return n;
}

guint16
g5125_timestamp_now (void)
{
  g_autoptr(GDateTime) now = g_date_time_new_now_local ();

  return (guint16) (g_date_time_get_second (now) * 1000 +
                    g_date_time_get_microsecond (now) / 1000);
}

gboolean
g5125_decode_image (const guint8 *plain, gsize len, guint16 pix[G5125_PIXELS])
{
  const guint8 *d = plain + 8;

  gboolean native = len == G5125_NATIVE_PLAIN_LEN;
  int raw_width = native ? G5125_WIDTH : G5125_RAW_WIDTH;
  int raw_pixels = raw_width * G5125_HEIGHT;

  if (!native && len != G5125_IMAGE_PLAIN_LEN)
    return FALSE;
  if (native)
    {
      const gsize packed_len = G5125_PIXELS / 4 * 6;
      const guint8 *tail = d + packed_len;
      guint32 expected = ((guint32) tail[2] << 24) | ((guint32) tail[3] << 16) |
                         ((guint32) tail[0] << 8) | tail[1];
      guint32 crc = 0xffffffff;

      for (gsize i = 0; i < packed_len; i++)
        {
          crc ^= (guint32) d[i] << 24;
          for (int bit = 0; bit < 8; bit++)
            crc = (crc & 0x80000000) ? (crc << 1) ^ 0x04c11db7 : crc << 1;
        }
      if (crc != expected)
        return FALSE;
    }
  for (int i = 0; i < raw_pixels / 4; i++, d += 6)
    {
      const guint16 v[4] = {
        ((d[0] & 0xf) << 8) + d[1],
        (d[3] << 4) + (d[0] >> 4),
        ((d[5] & 0xf) << 8) + d[2],
        (d[4] << 4) + (d[5] >> 4),
      };

      for (int k = 0; k < 4; k++)
        {
          int idx = 4 * i + k;
          int x = idx % raw_width, y = idx / raw_width;

          if (x < G5125_WIDTH)
            pix[y * G5125_WIDTH + x] = v[k];
        }
    }
  return TRUE;
}

static gint
cmp_gint (gconstpointer a, gconstpointer b)
{
  gint x = *(const gint *) a, y = *(const gint *) b;

  return (x > y) - (x < y);
}

void
g5125_image_to_8bit (const guint16 pix[G5125_PIXELS], guint8 out[G5125_PIXELS])
{
  gint v[G5125_PIXELS], sorted[G5125_PIXELS];
  gint lo, hi;

  /* a finger lowers the raw value: map low values (ridges) to bright */
  for (int i = 0; i < G5125_PIXELS; i++)
    v[i] = sorted[i] = -(gint) pix[i];

  /* stretch between the 1st and 99th percentile so single hot or dead
   * pixels do not squash the contrast */
  qsort (sorted, G5125_PIXELS, sizeof (gint), cmp_gint);
  lo = sorted[G5125_PIXELS / 100];
  hi = sorted[G5125_PIXELS - 1 - G5125_PIXELS / 100];

  for (int i = 0; i < G5125_PIXELS; i++)
    {
      gint c = CLAMP (v[i], lo, hi);

      out[i] = hi > lo ? (guint8) ((c - lo) * 255 / (hi - lo)) : 0;
    }
}

void
g5125_engine_prep (const guint16 pix[G5125_PIXELS], guint8 out[G5125_PIXELS])
{
  /* 3x3 high-pass around 128: the form the Goodix engine matches best with */
  for (int y = 0; y < G5125_HEIGHT; y++)
    for (int x = 0; x < G5125_WIDTH; x++)
      {
        gint sum = 0, n = 0;
        gint v;

        for (int yy = MAX (0, y - 1); yy <= MIN (G5125_HEIGHT - 1, y + 1); yy++)
          for (int xx = MAX (0, x - 1); xx <= MIN (G5125_WIDTH - 1, x + 1); xx++)
            {
              sum += pix[yy * G5125_WIDTH + xx];
              n++;
            }
        v = (gint) lroundf (128.0f + (pix[y * G5125_WIDTH + x] - (gfloat) sum / n) * G5125_ENGINE_GAIN);
        out[y * G5125_WIDTH + x] = (guint8) CLAMP (v, 0, 255);
      }
}

GVariant *
g5125_template_to_variant (const guint8 *blob, gsize len)
{
  return g_variant_new ("(y@ay)", G5125_TEMPLATE_VERSION,
                        g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, blob, len, 1));
}

gboolean
g5125_template_from_variant (GVariant *v, const guint8 **blob, gsize *len)
{
  g_autoptr(GVariant) data = NULL;
  guint8 version;

  if (!v || !g_variant_is_of_type (v, G_VARIANT_TYPE ("(yay)")))
    return FALSE;
  g_variant_get (v, "(y@ay)", &version, &data);
  if (version != G5125_TEMPLATE_VERSION)
    return FALSE;
  /* data is owned by v, so the pointer stays valid as long as v does */
  *blob = g_variant_get_fixed_array (data, len, 1);
  return *len > 0;
}

/* Version 4 prints hold several engine templates of the same finger, e.g. two
 * enrollment sessions. Matching takes the best score over all of them. */
GVariant *
g5125_templates_to_variant (const guint8 * const *blobs, const gsize *lens, guint n)
{
  GVariantBuilder b;

  g_variant_builder_init (&b, G_VARIANT_TYPE ("aay"));
  for (guint i = 0; i < n; i++)
    g_variant_builder_add_value (&b, g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                                                blobs[i], lens[i], 1));
  return g_variant_new ("(y@aay)", G5125_TEMPLATE_VERSION_MULTI, g_variant_builder_end (&b));
}

/* Reads a version 3 or version 4 print. Returns the number of templates (0 if
 * the data is not a goodixtls5125 print). Pointers stay owned by v. */
guint
g5125_templates_from_variant (GVariant *v, const guint8 **blobs, gsize *lens, guint max)
{
  g_autoptr(GVariant) arr = NULL;
  guint8 version;
  guint n;

  if (max == 0 || !v)
    return 0;
  if (g5125_template_from_variant (v, &blobs[0], &lens[0]))
    return 1;
  if (!g_variant_is_of_type (v, G_VARIANT_TYPE ("(yaay)")))
    return 0;
  g_variant_get (v, "(y@aay)", &version, &arr);
  if (version != G5125_TEMPLATE_VERSION_MULTI)
    return 0;
  n = MIN (g_variant_n_children (arr), max);
  for (guint i = 0; i < n; i++)
    {
      g_autoptr(GVariant) child = g_variant_get_child_value (arr, i);

      blobs[i] = g_variant_get_fixed_array (child, &lens[i], 1);
      if (lens[i] == 0)
        return 0;
    }
  return n;
}
