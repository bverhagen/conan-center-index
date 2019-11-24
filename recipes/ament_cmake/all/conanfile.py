from conans import ConanFile, tools
import os
import glob


class AmentCmakeConan(ConanFile):
    name = "ament_cmake"
    description = "Supporting CMake packages for working with ament "
    topics = ("conan", "ros2")
    url = "https://github.com/ament/ament_cmake"
    homepage = "https://github.com/ament/ament_cmake"
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

    def requirements(self):
        for requirement in self.conan_data["requires"][self.version]:
            self.requires(requirement)

    def source(self):
        print(self.conan_data["requires"][self.version])
        tools.get(**self.conan_data["sources"][self.version])
        extracted_dir = self.name + "-" + self.version
        os.rename(extracted_dir, self._source_subfolder)

    def build(self):
        self.run("cd {SOURCE_DIR} && colcon build --merge-install --build-base {BUILD_DIR} --install-base {BUILD_DIR}/install --cmake-args -DCMAKE_BUILD_TYPE={MODE} -DBUILD_TESTING=OFF".format(SOURCE_DIR=self.source_folder + '/' + self._source_subfolder, BUILD_DIR=self.build_folder, INSTALL_DIR=self.package_folder, MODE=self.settings.build_type), run_environment=True)

    def package(self):
        self.copy(pattern="*", dst="share", src="install/share")
        self.copy("LICENSE", dst="licenses")

    def package_info(self):
        cmake_dirs = glob.glob("{PACKAGE_DIR}/share/*/cmake".format(PACKAGE_DIR = self.package_folder))
        self.env_info.CMAKE_PREFIX_PATH = cmake_dirs
