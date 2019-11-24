from conans import ConanFile, tools
import os
from sys import version

class AmentPackageConan(ConanFile):
    name = "ament_package"
    description = "Python package for parsing package manifest files (package.xml). "
    topics = ("conan", "ros2")
    url = "https://github.com/ament/ament_package"
    homepage = "https://github.com/ament/ament_package"
    author = "Bart Verhagen <bart@verhagenconsultancy.be>"
    license = "Apache-2.0"
    no_copy_source = True
    settings = "os_build", "arch_build", "compiler", "build_type"
    _source_subfolder = "source_subfolder"

    def system_requirements(self):
        import pip
        if hasattr(pip, "main"):
            pip.main(["install", "--user", "colcon-common-extensions"])
        else:
            from pip._internal import main
            main(['install', '--user', "colcon-common-extensions"])

    def source(self):
        tools.get(**self.conan_data["sources"][self.version])
        extracted_dir = self.name + "-" + self.version
        os.rename(extracted_dir, self._source_subfolder)

    def build(self):
        self.run("cd {SOURCE_DIR} && colcon build --merge-install --build-base {BUILD_DIR} --install-base {BUILD_DIR}/install --cmake-args -DCMAKE_BUILD_TYPE={MODE} -DBUILD_TESTING=OFF -DENABLE_TESTING=OFF".format(SOURCE_DIR=self.source_folder + '/' + self._source_subfolder, BUILD_DIR=self.build_folder, INSTALL_DIR=self.package_folder, MODE=self.settings.build_type), run_environment=True)

    def package(self):
        self.copy(pattern="*", dst="lib", src="install/lib")
        self.copy(pattern="*", dst="share", src="install/share")
        self.copy("LICENSE", dst="licenses", src=self._source_subfolder)

    def package_info(self):
        self.env_info.PYTHONPATH.append(self.package_folder + '/lib/python' + version[:3] + '/site-packages')
