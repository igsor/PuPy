from distutils.core import setup

setup(
    name='PuPy',
    url='http://www.igsor.net/research/PuPy/',
    author='Matthias Baumgartner',
    author_email='research@igsor.net',
    version='1.0',
    packages=['PuPy'],
    license='Free for use',
    long_description=open('README').read(),
    requires=("scipy","numpy")
)
