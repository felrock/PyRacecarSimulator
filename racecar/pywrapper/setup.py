from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy, os, platform, sys
from os.path import join as pjoin


if __name__ == '__main__':


    try:
        numpy_include = numpy.get_include()
    except:
        numpy_include = numpy.get_numpy_include()


    compiler_flags = ['-w', '-std=c++11', '-march=native', '-ffast-math',
                     '-fno-math-errno', '-lm', '-I.']
    include_dirs = ['../', numpy_include]
    depends = ['include/racecar.hpp']
    sources = ['pywrapper/racecar.pyx', 'src/racecar.cpp']

    #
    setup(name='racecar',
          author='racecar',
          version='0.1',
          ext_modules=[Extension('racecar',  sources,
                       extra_compile_args = compiler_flags,
                       extra_link_args = ['-std=c++11'],
                       include_dirs = include_dirs,
                       depends = depends,
                       language = 'c++')],
          cmdclass = {'build_ext': build_ext})


