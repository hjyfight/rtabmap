"""Exercise the actual MITI bundles and server entrypoints, including upgrades."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('miti_installer', ROOT / 'scripts/install_miti_loop_panel.py')
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class FrontendInstallTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='miti_frontend_install_')
        self.addCleanup(self.directory.cleanup)
        self.app = Path(self.directory.name)
        (self.app / 'package.json').write_text('{"version":"3.3.6"}')
        node = self.app / 'node/bin/node'
        node.parent.mkdir(parents=True)
        node.symlink_to(installer.APP / 'node/bin/node')
        for name in installer.UPSTREAM | installer.ENTRYPOINTS:
            original = installer.APP / 'rtabmap-panel-originals' / name
            source = original if original.exists() else installer.APP / name
            target = self.app / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)

    def install(self):
        with contextlib.redirect_stdout(io.StringIO()):
            installer.install(self.app)
        return json.loads((self.app / 'public/rtabmap-panel-assets.json').read_text())

    def test_entrypoints_and_lazy_renderer_use_the_installed_content_versions(self):
        manifest = self.install()
        renderer_url, page_url = manifest['assets']
        renderer = (self.app / renderer_url.split('?', 1)[0].replace('/_next/', '.next/', 1)).read_bytes()
        page = (self.app / page_url.replace('/_next/', '.next/', 1)).read_bytes()
        renderer_hash = installer.digest(renderer)[:12]
        self.assertTrue(renderer_url.endswith('?rtabmap_panel=' + renderer_hash))
        self.assertIn(('rtabmapVersionPanelChunk(r,"' + renderer_hash + '");').encode(), page)
        self.assertEqual(Path(page_url).name, 'page-' + installer.digest(page)[:16] + '.js')
        for name in installer.ENTRYPOINTS:
            data = (self.app / name).read_text()
            self.assertIn(page_url.removeprefix('/_next/'), data)
            self.assertNotIn(installer.PAGE_CHUNK.removeprefix('.next/'), data)
        self.assertEqual(self.install(), manifest)

    def test_upgrade_changes_eager_script_address_and_preserves_vendor_originals(self):
        before = self.install()
        extend = installer.extend_bundle
        with patch.object(installer, 'extend_bundle', side_effect=lambda *a, **kw: extend(*a, **kw) + '\n// upgrade fixture\n'):
            after = self.install()
        self.assertNotEqual(before['assets'][0], after['assets'][0])
        self.assertNotEqual(before['assets'][1], after['assets'][1])
        for name, expected in (installer.UPSTREAM | installer.ENTRYPOINTS).items():
            saved = (self.app / 'rtabmap-panel-originals' / name).read_bytes()
            self.assertEqual(installer.digest(saved), expected)
        for name in installer.ENTRYPOINTS:
            data = (self.app / name).read_text()
            self.assertIn(after['assets'][1].removeprefix('/_next/'), data)
            self.assertNotIn(before['assets'][1].removeprefix('/_next/'), data)

    def test_unrecognized_entrypoint_change_leaves_all_bundles_untouched(self):
        name = next(iter(installer.ENTRYPOINTS))
        target = self.app / name
        target.write_bytes(target.read_bytes() + b'\n')
        originals = {name: (self.app / name).read_bytes() for name in installer.UPSTREAM}
        with self.assertRaisesRegex(ValueError, 'Unexpected MITI bundle change'):
            self.install()
        for name, original in originals.items():
            self.assertEqual((self.app / name).read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
