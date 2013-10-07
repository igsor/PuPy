from distutils.core import setup

setup(
    name='PuPy',
    url='http://www.igsor.net/research/PuPy/',
    author='Matthias Baumgartner',
    author_email='research@igsor.net',
    version='1.0',
    license='Free for use',
    long_description=open('README').read(),
    requires=("scipy","numpy","matplotlib"),
    packages=['PuPy'],
    package_data={
        'PuPy' : ['../data/puppy_unit_interval.json', '../data/puppy_zero_mean_unit_var.json']
    },
)
