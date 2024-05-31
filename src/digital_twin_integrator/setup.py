from setuptools import setup
import glob

package_name = 'digital_twin_integrator'

setup(
	name=package_name,
	version='0.1.0',
	packages=[package_name, 'digital_twin_integrator.digital_twin_integrator_component_definition'],
	data_files = [
			('share/ament_index/resource_index/packages',
				['resource/' + package_name]),
			('share/' + package_name, ['package.xml']),
			('share/' + package_name + '/launch', glob.glob('launch/*.py')),
			('share/' + package_name + '/launch', glob.glob('launch/*.launch')),
			('share/' + package_name + '/launch/cfg', glob.glob('launch/cfg/*.yaml')),
		],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='Raphaël LALLEMENT',
	maintainer_email='raphael.lallement@cea.fr',
	description='A digital twin integrator node. It interacts with digital twins to get data that come from the perception pipeline, such as numerical poses of objects in the scene, the robot configuration, etc., and enrich assertion statements in an ontology (ABoxes) with those data.',
	license='CeCILL-B',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'digital_twin_integrator = digital_twin_integrator.digital_twin_integrator_component_definition.digital_twin_integrator_main:main',
			]
		}
	)
