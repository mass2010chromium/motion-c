
from distutils.core import setup, Extension

ur = Extension('motion.ur5controller',
                    sources = ['ur5controller.c'])
vo = Extension('motion.vectorops',
                    sources = ['vectorops.c'])

so3 = Extension('motion.so3',
                    sources = ['so3.c'])

se3 = Extension('motion.se3',
                    sources = ['se3.c'])

setup (name = 'Motion',
       version = '1.0',
       description = 'Motion code I guess',
       ext_modules = [ur, vo, so3, se3])
