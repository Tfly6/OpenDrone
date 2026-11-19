"""Exercise the publication bundle outside the author's workspace layout."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


PACKAGE = Path(__file__).resolve().parents[1]


class PortablePublicationBundleTest(unittest.TestCase):
    def test_full_suite_covers_all_attempts_and_preserves_repetition_identity(self):
        import collections
        from run_suite import load_suite, select_batches
        suite = load_suite()
        cases = [c for b in select_batches(suite) for c in b['cases']]
        self.assertEqual(collections.Counter(c['group'] for c in cases),
                         {'hover':21, 'tracking':105, 'generation':140, 'online':300, 'controlled':80})
        self.assertEqual(sum(len(b['cases']) for b in select_batches(suite, 'online', sensor='Mid360')), 108)
        self.assertEqual(sum(len(b['cases']) for b in select_batches(suite, 'online', sensor='depth_camera')), 192)
        self.assertEqual(len({(b['id'],c['source_case']) for b in suite['batches'] for c in b['cases']}), 646)
        self.assertTrue(all(c['repetition'] >= 1 and c['profile'] in suite['profiles'] for c in cases))

    def test_renamed_checkout_verifies_without_ros_or_author_paths(self):
        with tempfile.TemporaryDirectory(prefix='opendrone_data_portability_') as directory:
            clone = Path(directory) / 'Renamed Checkout/opendrone/data_code'
            shutil.copytree(PACKAGE, clone, ignore=shutil.ignore_patterns(
                '__pycache__', '*.pyc', 'outputs', 'release_checksums.json'))
            environment = dict(os.environ)
            for key in ('PYTHONPATH', 'OPENDRONE_WORKSPACE', 'OPENDRONE_ARTIFACTS', 'PX4_SOURCE_DIR'):
                environment.pop(key, None)
            result = subprocess.run([sys.executable, str(clone / 'verify_example.py')],
                                    cwd=directory, env=environment, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('Synthetic example verified', result.stdout)
            listing = subprocess.run([sys.executable, str(clone / 'run_suite.py'), 'list'],
                                     cwd=directory, env=environment, capture_output=True, text=True, check=True)
            self.assertEqual(json.loads(listing.stdout)['attempts'], 646)
            path_check = subprocess.run([sys.executable, '-c',
                'from revision_support.paths import REPOSITORY_ROOT, SCRIPTS_ROOT; '
                'print(REPOSITORY_ROOT); print(SCRIPTS_ROOT)'],
                cwd=clone, env=environment, capture_output=True, text=True, check=True)
            self.assertIn(str(clone.parents[1]), path_check.stdout)
            self.assertNotIn('/home/test/', path_check.stdout)


if __name__ == '__main__':
    unittest.main()
