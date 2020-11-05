import os

from conans import ConanFile, tools, AutoToolsBuildEnvironment
from conans.errors import ConanInvalidConfiguration


class GlibmmConan(ConanFile):
    name = "glibmm"
    url = "https://github.com/conan-io/conan-center-index"
    license = "LGPL-2.1-or-later"
    homepage = "https://www.gtkmm.org"
    description = "C++ API for parts of glib that are useful for C++"
    settings = {"os": "Linux"}
    options = {"version": [2]}
    default_options = {"version": 2}
    topics = "glib"

    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
    }
    default_options = {"shared": False, "fPIC": True}
    generators = "pkg_config"
    _autoconf = None

    @property
    def _source_subfolder(self):
        return "source_subfolder"

    @property
    def _build_subfolder(self):
        return "build_subfolder"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if (
            self.settings.compiler == "Visual Studio"
            and "MT" in str(self.settings.compiler.runtime)
            and self.options.shared
        ):
            raise ConanInvalidConfiguration(
                "Visual Studio and Runtime MT is not supported for shared library."
            )
        if self.options.shared:
            del self.options.fPIC

    def source(self):
        tools.get(**self.conan_data["sources"][self.version])
        extracted_dir = self.name + "-" + self.version
        os.rename(extracted_dir, self._source_subfolder)

    def requirements(self):
        self.requires("glib/2.67.0")
        self.requires("libsigcpp/2.10.4")

    def _configure_build(self):
        if self._autoconf:
            return self._autoconf

        self._autoconf = AutoToolsBuildEnvironment(self)
        return self._autoconf

    def build(self):
        autoconf = self._configure_build()

        args = ["--enable-warnings=no"]

        if self.options.shared:
            args.append("--enable-shared")
            args.append("--disable-shared")
        else:
            args.append("--enable-static")
            args.append("--disable-static")

        if self.options.fPIC:
            args.append("--with-pic")
        else:
            args.append("--without-pic")

        autoconf.configure(
            configure_dir=self._source_subfolder,
            args=args,
            vars={"cross_compiling": "no"},
        )

    def package(self):
        autoconf = self._configure_build()
        autoconf.install()
        tools.rmdir(os.path.join(self.package_folder, "share"))
        tools.rmdir(os.path.join(self.package_folder, "lib/pkgconfig"))

        tools.remove_files_by_mask(self.package_folder + "/lib", "*.la")

        # if not self.options.shared:
        # tools.remove_files_by_mask(self.package_folder + '/lib', "*.so*")

    def package_info(self):
        self.cpp_info.includedirs = [
            "include/giomm-2.4",
            "include/glibmm-2.4",
            "lib/giomm-2.4/include",
            "lib/glibmm-2.4/include",
        ]
        self.cpp_info.libs = tools.collect_libs(self)
