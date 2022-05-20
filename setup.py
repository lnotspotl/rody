import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="rody",
    version="0.1.2",
    author="notspot",
    author_email="kubjonkyr@gmail.com",
    description="rody package",
    long_description=long_description,
    url="None",
    package_dir={"": "src"},
    packages=setuptools.find_packages(where="src"),
    python_requires=">=3.6",
)
