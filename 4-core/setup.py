import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='rosbag_preprocess',
    version='0.0.1',
    license='',
    description='Consumes rosbags and produces intermediate file products',
    long_description=long_description,
    long_description_content_type="text/markdown",
    author='artificial-agent',
    author_email='20388458+artificial-agent@users.noreply.github.com ',
    packages=setuptools.find_packages(),
    install_requires=[
        'numpy',
    ],
    zip_safe=False,
    python_requires='>=3.10'
)
