/* Builds a multi-template goodixtls5125 print from captured touches.
 *
 * usage: g5125-mkprint <engine.dll> <calibration> <baseline> <base-print> <out-print>
 *                      <set-dir>[:first-last] ... [--keep-base]
 *
 * Each <set-dir> holds t<NNN>-f0.raw frames from the capture tool; frame 0 of
 * every touch in the given range becomes one enrollment image. Images are
 * packed into templates of up to 30 images. --keep-base also keeps the base
 * print's own template(s). The output copies the base print's metadata
 * (user, finger, driver, device) and stores a version-4 print.
 */
#include "../libfprint/drivers/goodixtls/goodix_engine.c"
#include "../libfprint/drivers/goodixtls/goodix5125_calib.c"
#include <fprint.h>

#define PER_TEMPLATE 30

int
main (int argc, char **argv)
{
  GError *error = NULL;
  const guint8 *blobs[G5125_MAX_TEMPLATES];
  gsize lens[G5125_MAX_TEMPLATES];
  guint nt = 0;
  gboolean keep_base = FALSE;
  static guint8 prep[512][5120];
  int nimg = 0;

  if (argc < 7 || !goodix_engine_init (argv[1], &error) ||
      !goodix_engine_native_preprocess_init (argv[2], argv[3], &error))
    {
      fprintf (stderr, "init: %s\n", error ? error->message : "usage");
      return 1;
    }

  for (int a = 6; a < argc; a++)
    {
      if (!strcmp (argv[a], "--keep-base"))
        {
          keep_base = TRUE;
          continue;
        }
      g_auto(GStrv) parts = g_strsplit (argv[a], ":", 2);
      int first = 1, last = 999;
      if (parts[1])
        sscanf (parts[1], "%d-%d", &first, &last);
      for (int t = first; t <= last && nimg < 512; t++)
        {
          g_autofree gchar *p = g_strdup_printf ("%s/t%03d-f0.raw", parts[0], t);
          g_autofree gchar *raw = NULL;
          gsize len = 0;
          guint q = 0, c = 0;
          if (!g_file_get_contents (p, &raw, &len, NULL))
            break;
          if (len == 10240 && goodix_engine_native_prepare ((const guint16 *) raw, prep[nimg], &q, &c))
            nimg++;
        }
      printf ("%s: %d images so far\n", argv[a], nimg);
    }

  /* base print */
  g_autofree gchar *data = NULL;
  gsize dlen = 0;
  FpPrint *base;
  g_autoptr(GVariant) bv = NULL;
  if (!g_file_get_contents (argv[4], &data, &dlen, &error) ||
      !(base = fp_print_deserialize ((guchar *) data, dlen, &error)))
    {
      fprintf (stderr, "base print: %s\n", error->message);
      return 1;
    }
  g_object_get (base, "fpi-data", &bv, NULL);

  /* split images evenly into templates of at most PER_TEMPLATE */
  int ntemp = (nimg + PER_TEMPLATE - 1) / PER_TEMPLATE;
  for (int k = 0; k < ntemp && nt < G5125_MAX_TEMPLATES; k++)
    {
      int from = k * nimg / ntemp, to = (k + 1) * nimg / ntemp;
      int max_images = 0, count = 0, progress = 0;
      void *ctx = goodix_engine_enroll_start (&max_images);
      goodix_engine_enroll_set_target (ctx, to - from);
      for (int i = from; i < to; i++)
        goodix_engine_enroll_add_image (ctx, prep[i], 80, 64, &count, &progress);
      guint8 *blob = NULL;
      size_t blen = 0;
      if (goodix_engine_enroll_commit (ctx, &blob, &blen) != 0)
        {
          fprintf (stderr, "commit failed\n");
          return 1;
        }
      goodix_engine_enroll_finish (ctx);
      blobs[nt] = blob;
      lens[nt] = blen;
      nt++;
      printf ("template %d: images %d-%d, count %d, %zu bytes\n", k, from, to - 1, count, blen);
    }
  if (keep_base)
    {
      const guint8 *bb[G5125_MAX_TEMPLATES];
      gsize bl[G5125_MAX_TEMPLATES];
      guint n = g5125_templates_from_variant (bv, bb, bl, G5125_MAX_TEMPLATES);
      for (guint i = 0; i < n && nt < G5125_MAX_TEMPLATES; i++)
        {
          blobs[nt] = bb[i];
          lens[nt] = bl[i];
          nt++;
          printf ("template from base print: %zu bytes\n", bl[i]);
        }
    }

  /* optional sanity check against other-finger frames */
  const char *imp = g_getenv ("G5125_IMPOSTOR_DIR");
  if (imp)
    {
      int max = 0, frames = 0;
      for (int t = 1; t < 999; t++)
        for (int f = 0; f < 3; f++)
          {
            g_autofree gchar *p = g_strdup_printf ("%s/t%03d-f%d.raw", imp, t, f);
            g_autofree gchar *raw = NULL;
            gsize len = 0;
            guint q = 0, c = 0;
            guint8 pr[5120];
            if (!g_file_get_contents (p, &raw, &len, NULL) || len != 10240 ||
                !goodix_engine_native_prepare ((const guint16 *) raw, pr, &q, &c))
              continue;
            frames++;
            for (guint i = 0; i < nt; i++)
              {
                int sc = 0;
                goodix_engine_verify_image (pr, 80, 64, blobs[i], lens[i], &sc);
                max = MAX (max, sc);
              }
          }
      printf ("impostor check: %d frames, max score %d (threshold %d)\n", frames, max, G5125_MATCH_THRESHOLD);
      if (max >= G5125_MATCH_THRESHOLD)
        return 3;
    }

  g_object_set (base, "fpi-data", g5125_templates_to_variant (blobs, lens, nt), NULL);
  guint8 *out = NULL;
  gsize olen = 0;
  if (!fp_print_serialize (base, &out, &olen, &error) ||
      !g_file_set_contents_full (argv[5], (gchar *) out, olen, G_FILE_SET_CONTENTS_CONSISTENT, 0600, &error))
    {
      fprintf (stderr, "write: %s\n", error->message);
      return 1;
    }
  printf ("wrote %s: %u templates, %zu bytes, finger %d, user %s\n", argv[5], nt, olen,
          fp_print_get_finger (base), fp_print_get_username (base));
  return 0;
}
