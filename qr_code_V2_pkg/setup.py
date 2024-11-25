from setuptools import setup

package_name = 'qr_code_V2_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oriane',
    maintainer_email='oriane@todo.todo',
    description='QR Code detection package for Aquabot Competitor.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_code_detector = qr_code_V2_pkg.qr_code_detector:main',
            'qr_code_reader = qr_code_V2_pkg.qr_code_reader:main',
            'wind_turbine_perimeter = qr_code_V2_pkg.wind_turbine_perimeter:main',
        ],
    },
)
