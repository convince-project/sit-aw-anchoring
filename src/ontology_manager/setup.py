from setuptools import setup
import glob

package_name = 'ontology_manager'

setup(
	name=package_name,
	version='0.1.0',
	packages=[package_name, 
           'ontology_manager.ontology_manager_component_definition',
           'ontology_manager.ontology_manager_component_definition.func',
           ],
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
	description='An ontology manager node. It that encapsulates 3 logical blocks that realize the semantic anchoring stack. A knowledge base constituted by an ontology and its rules. A populator block that realizes the setup phase and adds individuals (ABoxes) to the knowledge base when this is out of the perception-component scope. A reasoner that discovers and makes explicit all implied information by terminology (TBox) and assertion (ABox) statements (and ensures the consistency of the ontology). All ontology manipulations rely on the Owlready2.',
	license='CeCILL-B',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'ontology_manager = ontology_manager.ontology_manager_component_definition.ontology_manager_main:main',
			]
		}
	)
