from glob import glob
from setuptools import find_packages, setup

setup(name='towergo_real', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/towergo_real']),
                  ('share/towergo_real', ['package.xml']),
                  ('share/towergo_real/config', glob('config/*')),
                  ('share/towergo_real/launch', glob('launch/*.py')),
                  ('share/towergo_real/web', glob('web/*'))],
      install_requires=['setuptools'], entry_points={'console_scripts': [
          'keyboard = towergo_real.keyboard:main',
          'chassis = towergo_real.chassis:main',
          'lio_adapter = towergo_real.lio_adapter:main',
          'map_viewer = towergo_real.viewer:main',
          'session = towergo_real.session:main',
          'preflight = towergo_real.preflight:main',
          'audit_capture = towergo_real.audit:main',
      ]})
