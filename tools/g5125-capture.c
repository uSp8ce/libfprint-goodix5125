/* Touch-capture loop for the Goodix 27c6:5125 driver.
 *
 * Runs verify against one stored fprintd print, once per touch, and prints one
 * line per result. With GOODIX5125_CAPTURE_DIR set, the driver also saves the
 * raw frames of every touch. Needs the device to be free (stop fprintd).
 *
 * usage: g5125-capture <print-file> <touches>
 */
#include <stdio.h>
#include <stdlib.h>
#include <fprint.h>

static void
on_match (FpDevice *dev, FpPrint *match, FpPrint *print, gpointer data, GError *error)
{
  if (error)
    printf ("  retry: %s\n", error->message);
}

int
main (int argc, char **argv)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpContext) ctx = NULL;
  g_autofree gchar *blob = NULL;
  gsize len = 0;
  GPtrArray *devices;
  FpDevice *dev;
  FpPrint *enrolled;
  int touches;

  setvbuf (stdout, NULL, _IOLBF, 0);
  if (argc != 3)
    {
      fprintf (stderr, "usage: %s <print-file> <touches>\n", argv[0]);
      return 2;
    }
  touches = atoi (argv[2]);
  if (!g_file_get_contents (argv[1], &blob, &len, &error) ||
      !(enrolled = fp_print_deserialize ((guchar *) blob, len, &error)))
    {
      fprintf (stderr, "print: %s\n", error->message);
      return 1;
    }

  ctx = fp_context_new ();
  devices = fp_context_get_devices (ctx);
  if (!devices || devices->len == 0)
    {
      fprintf (stderr, "no fingerprint device\n");
      return 1;
    }
  dev = g_ptr_array_index (devices, 0);
  if (!fp_device_open_sync (dev, NULL, &error))
    {
      fprintf (stderr, "open: %s\n", error->message);
      return 1;
    }

  for (int i = 1; i <= touches; i++)
    {
      gboolean match = FALSE;

      printf ("READY %d\n", i);
      if (!fp_device_verify_sync (dev, enrolled, NULL, on_match, NULL, &match, NULL, &error))
        {
          printf ("TOUCH %d error: %s\n", i, error->message);
          g_clear_error (&error);
          continue;
        }
      printf ("TOUCH %d %s\n", i, match ? "MATCH" : "no-match");
    }

  fp_device_close_sync (dev, NULL, NULL);
  return 0;
}
