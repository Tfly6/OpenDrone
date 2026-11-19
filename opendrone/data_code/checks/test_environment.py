#!/usr/bin/env python3

import os
import tempfile
import unittest
import xml.etree.ElementTree as ET

from flight_eval.environment import _patch_iris_udp


class ContactPluginInjectionTest(unittest.TestCase):
    def test_iris_overlay_contains_one_configured_contact_plugin(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            source_dir = os.path.join(temp_dir, 'source_iris')
            output_dir = os.path.join(temp_dir, 'output_iris')
            os.makedirs(source_dir)
            with open(
                os.path.join(source_dir, 'model.config'),
                'w',
                encoding='utf-8',
            ) as stream:
                stream.write('<model><name>iris</name></model>')
            with open(
                os.path.join(source_dir, 'iris.sdf'),
                'w',
                encoding='utf-8',
            ) as stream:
                stream.write(
                    '<sdf version="1.6"><model name="iris">'
                    '<link name="base_link"><collision name="collision">'
                    '<geometry><box><size>1 1 1</size></box></geometry>'
                    '</collision></link>'
                    '<plugin name="mavlink_interface" filename="libmavlink.so">'
                    '<use_tcp>1</use_tcp>'
                    '</plugin></model></sdf>'
                )

            output_sdf = _patch_iris_udp(source_dir, output_dir, 14560)
            tree = ET.parse(output_sdf)
            plugins = [
                plugin for plugin in tree.getroot().iter('plugin')
                if plugin.get('name') == 'opendrone_contact'
            ]

        self.assertEqual(len(plugins), 1)
        self.assertEqual(
            plugins[0].get('filename'),
            'libopendrone_contact_plugin.so',
        )
        self.assertEqual(
            plugins[0].findtext('ignoredCollisionSubstring'),
            'ground_plane',
        )
        self.assertEqual(
            tree.getroot().find('.//mavlink_udp_port').text,
            '14560',
        )


if __name__ == '__main__':
    unittest.main()
