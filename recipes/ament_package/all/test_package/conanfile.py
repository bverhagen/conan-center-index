from conans import ConanFile, CMake, tools
import os


class TestPackageConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"

    def test(self):
        self.run("python3 {SOURCE_DIR}/test_package.py".format(SOURCE_DIR = self.source_folder), run_environment=True)
