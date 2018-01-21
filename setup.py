from setuptools import setup

setup(
   name='ass4',
   version='1.0',
   description='A useful module',
   author='Eyal Segal',
   author_email='DONOT@mail.me',
   packages=['ass4'],  #same as name
   install_requires=['promise'], #external packages as dependencies
)