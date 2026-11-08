/* Offline evaluation of Goodix 27c6:5125 matching on captured touches.
 *
 * usage: g5125-eval <engine.dll> <calibration> <baseline> <capture-dir> [old-print]
 * capture-dir holds enroll/, casual/, other/ with t<NNN>-f<k>.raw frames.
 * Enrolls templates from the "enroll" touches under several settings and
 * reports casual-touch acceptance and other-finger false accepts.
 */
#include "../libfprint/drivers/goodixtls/goodix_engine.c"
#include "../libfprint/drivers/goodixtls/goodix5125_calib.c"
#include <fprint.h>

#define MAXT 64
#define NF 3

typedef struct {
  int n;                          /* touches */
  guint8 prep[MAXT][NF][5120];
  gboolean ok[MAXT][NF];
  guint q[MAXT][NF];
} Set;

static void
load_set (const char *dir, const char *name, Set *s)
{
  memset (s, 0, sizeof *s);
  for (int t = 1; t <= MAXT; t++)
    {
      gboolean any = FALSE;
      for (int f = 0; f < NF; f++)
        {
          g_autofree gchar *p = g_strdup_printf ("%s/%s/t%03d-f%d.raw", dir, name, t, f);
          g_autofree gchar *raw = NULL;
          gsize len = 0;
          guint c = 0;
          if (!g_file_get_contents (p, &raw, &len, NULL) || len != 10240)
            continue;
          any = TRUE;
          s->ok[t - 1][f] = goodix_engine_native_prepare ((const guint16 *) raw,
                                                          s->prep[t - 1][f], &s->q[t - 1][f], &c);
        }
      if (!any)
        break;
      s->n = t;
    }
  printf ("%s: %d touches\n", name, s->n);
}

/* Enroll from a list of (set, touch, frame) samples, up to target images. */
typedef struct { Set *s; int t, f; } Sample;

static int
enroll (Sample *samples, int n, int target, guint8 **blob, size_t *len, int *used)
{
  int max_imgs = 32;
  g_rec_mutex_lock (&g_milan_mutex);
  ensure_gs ();
  void *ctx = m_enrolStartEx (&max_imgs);
  *(u16 *) ((char *) ctx + 8) = target;
  g_rec_mutex_unlock (&g_milan_mutex);
  *used = 0;
  int count = 0, progress = 0;
  for (int i = 0; i < n && count < target; i++)
    {
      Sample *x = &samples[i];
      if (!x->s->ok[x->t][x->f])
        continue;
      int r = goodix_engine_enroll_add_image (ctx, x->s->prep[x->t][x->f], 80, 64, &count, &progress);
      if (r == 0)
        (*used)++;
    }
  int rc = goodix_engine_enroll_commit (ctx, blob, len);
  goodix_engine_enroll_finish (ctx);
  return rc ? -1 : count;
}

/* Best score of one touch against a gallery, using the first k frames. */
static int
touch_score (Set *s, int t, int k, guint8 **blobs, size_t *lens, int nb)
{
  int best = 0;
  for (int f = 0; f < k; f++)
    {
      if (!s->ok[t][f])
        continue;
      for (int b = 0; b < nb; b++)
        {
          int score = 0;
          goodix_engine_verify_image (s->prep[t][f], 80, 64, blobs[b], lens[b], &score);
          best = MAX (best, score);
        }
    }
  return best;
}

static void
report (const char *label, Set *gen, int gen_from, Set *imp, guint8 **blobs, size_t *lens, int nb)
{
  printf ("%-44s", label);
  for (int k = 1; k <= NF; k++)
    {
      int acc30 = 0, acc20 = 0, n = 0;
      for (int t = gen_from; t < gen->n; t++, n++)
        {
          int sc = touch_score (gen, t, k, blobs, lens, nb);
          acc30 += sc >= 30;
          acc20 += sc >= 20;
        }
      printf (" k=%d %2d/%d (%2d@20)", k, acc30, n, acc20);
    }
  int fa = 0, maxi = 0;
  for (int t = 0; t < imp->n; t++)
    {
      int sc = touch_score (imp, t, NF, blobs, lens, nb);
      fa += sc >= 20;
      maxi = MAX (maxi, sc);
    }
  printf ("  impostor max %d, FA@20 %d/%d\n", maxi, fa, imp->n);
}

int
main (int argc, char **argv)
{
  GError *error = NULL;
  static Set en, ca, ot;
  setvbuf (stdout, NULL, _IOLBF, 0);
  if (argc < 5 || !goodix_engine_init (argv[1], &error) ||
      !goodix_engine_native_preprocess_init (argv[2], argv[3], &error))
    {
      fprintf (stderr, "init: %s\n", error ? error->message : "usage");
      return 1;
    }
  load_set (argv[4], "enroll", &en);
  load_set (argv[4], "casual", &ca);
  load_set (argv[4], "other", &ot);

  guint8 *blobs[8];
  size_t lens[8];
  guint8 *oldb = NULL; size_t oldl = 0;
  int nb = 0;

  /* Old print (Oct 1 live enrollment) as the baseline. */
  if (argc > 5)
    {
      g_autofree gchar *data = NULL;
      gsize dlen = 0;
      FpPrint *p;
      g_autoptr(GVariant) v = NULL;
      const guint8 *b;
      gsize bl;
      if (g_file_get_contents (argv[5], &data, &dlen, NULL) &&
          (p = fp_print_deserialize ((guchar *) data, dlen, &error)))
        {
          g_object_get (p, "fpi-data", &v, NULL);
          if (g5125_template_from_variant (v, &b, &bl))
            {
              oldb = g_memdup2 (b, bl);
              oldl = bl;
              report ("old Oct-1 print", &ca, 0, &ot, &oldb, &oldl, 1);
            }
        }
    }

  Sample s[3 * MAXT];
  int ns, used;
  /* samples: 20 deliberate (frame 0) + casual 1-12 (frame 0); test casual 13-40 */
  ns = 0;
  for (int t = 0; t < en.n; t++) s[ns++] = (Sample) { &en, t, 0 };
  for (int t = 0; t < 12; t++) s[ns++] = (Sample) { &ca, t, 0 };
  int splits[] = { 1, 2 };
  for (int with_old = 0; with_old <= (argc > 5); with_old++)
    for (int i = 0; i < 2; i++)
      {
        int k = splits[i], per = (ns + k - 1) / k;
        nb = 0;
        for (int j = 0; j < k; j++)
          {
            int from = j * per, n = MIN (per, ns - from);
            enroll (s + from, n, n, &blobs[nb], &lens[nb], &used);
            nb++;
          }
        guint8 *g[6]; size_t gl[6];
        for (int j = 0; j < nb; j++) { g[j] = blobs[j]; gl[j] = lens[j]; }
        if (with_old) { g[nb] = oldb; gl[nb] = oldl; }
        g_autofree gchar *lab = g_strdup_printf ("32 touches in %d template(s)%s [13-40]", k, with_old ? " + old" : "");
        report (lab, &ca, 12, &ot, g, gl, nb + with_old);
        for (int j = 0; j < nb; j++) free (blobs[j]);
      }
  /* 24 deliberate-style touches only (what a live enrollment gives): enroll set only */
  ns = 0;
  for (int t = 0; t < en.n; t++) s[ns++] = (Sample) { &en, t, 0 };
  for (int k = 1; k <= 2; k++)
    {
      int per = (ns + k - 1) / k;
      nb = 0;
      for (int j = 0; j < k; j++)
        {
          int from = j * per, n = MIN (per, ns - from);
          enroll (s + from, n, n, &blobs[nb], &lens[nb], &used);
          nb++;
        }
      g_autofree gchar *lab = g_strdup_printf ("20 deliberate in %d template(s) [1-40]", k);
      report (lab, &ca, 0, &ot, blobs, lens, nb);
      for (int j = 0; j < nb; j++) free (blobs[j]);
    }

  /* capacity: all 60 right-index touches today in one template */
  {
    Sample all[3 * MAXT]; int na = 0, used2 = 0;
    for (int t = 0; t < en.n; t++) all[na++] = (Sample) { &en, t, 0 };
    for (int t = 0; t < ca.n; t++) all[na++] = (Sample) { &ca, t, 0 };
    int cnt = enroll (all, na, na, &blobs[0], &lens[0], &used2);
    gint64 t0 = g_get_monotonic_time ();
    int sc = 0;
    for (int r = 0; r < 20; r++) goodix_engine_verify_image (ot.prep[0][0], 80, 64, blobs[0], lens[0], &sc);
    printf ("capacity: %d samples -> count %d, accepted %d, template %zu B, verify %.1f ms\n",
            na, cnt, used2, lens[0], (g_get_monotonic_time () - t0) / 20000.0);
    guint8 *g2[2] = { blobs[0], oldb }; size_t l2[2] = { lens[0], oldl };
    report ("60 today + old (contaminated, all casual)", &ca, 0, &ot, g2, l2, oldb ? 2 : 1);
    free (blobs[0]);
  }
  return 0;
}
