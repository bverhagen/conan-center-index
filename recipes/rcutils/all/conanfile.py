from conans import ConanFile, tools
import os
import glob


class AmentCmakeRosConan(ConanFile):
    name = "rcutils"
    description = "Common C functions and data structures used in ROS 2 "
    topics = ("conan", "ros2")
    url = "https://github.com/ros2/rcutils"
    homepage = "https://github.com/ros2/rcutils"
    author = "Bart Verhagen <bart@verhagenconsultancy.be>"
    license = "Apache-2.0"
    no_copy_source = True
    settings = "os_build", "arch_build", "compiler", "build_type"
    _source_subfolder = "source_subfolder"

    requires = ('ament_cmake_ros/0.8.0@')

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
        self.run("cd {SOURCE_DIR} && colcon build --merge-install --build-base {BUILD_DIR} --install-base {BUILD_DIR}/install --cmake-args -DCMAKE_BUILD_TYPE={MODE} -DBUILD_TESTING=OFF".format(SOURCE_DIR=self.source_folder + '/' + self._source_subfolder, BUILD_DIR=self.build_folder, INSTALL_DIR=self.package_folder, MODE=self.settings.build_type), run_environment=True)

    def package(self):
        self.copy(pattern="*", dst="include", src="install/include")
        self.copy(pattern="*", dst="lib", src="install/lib")
        self.copy(pattern="*", dst="share", src="install/share")
        self.copy("LICENSE", dst="licenses")

    def package_info(self):
        cmake_dirs = glob.glob("{PACKAGE_DIR}/share/*/cmake".format(PACKAGE_DIR = self.package_folder))
        self.env_info.CMAKE_PREFIX_PATH = cmake_dirs
