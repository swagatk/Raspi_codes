#setup.py
from setuptools import find_packages, setup
package_name = 'hello_world'
setup(
	name=package_name,
	version='0.0.0',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='user',
	maintainer_email='user@todo.todo',
	description='A simple Hello World publisher and subscriber package',
	license='Apache-2.0',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'publisher = hello_world.publisher:main',
			'subscriber = hello_world.subscriber:main',
		],
	},
)
