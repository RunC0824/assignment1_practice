from setuptools import setup
from glob import glob
setup(name='ira1_sim', version='0.1.0', packages=['ira1_sim'],
 data_files=[('share/ament_index/resource_index/packages',['resource/ira1_sim']),
 ('share/ira1_sim',['package.xml']), ('share/ira1_sim/worlds',glob('worlds/*')),
 ('share/ira1_sim/launch',glob('launch/*')), ('share/ira1_sim/config',glob('config/*'))],
 entry_points={'console_scripts':['drive = ira1_sim.drive:main','record = ira1_sim.record:main','estimate = ira1_sim.estimate:main','evaluate = ira1_sim.evaluate:main']})
