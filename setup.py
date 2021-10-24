from distutils.core import setup, Extension

all_macros = [('MOTION_DEBUG', None)]

ur = Extension('motion.ur5controller',
                    sources = ['ur5controller.c'],
                    define_macros=all_macros)
vo = Extension('motion.vectorops',
                    sources = ['vectorops.c'],
                    define_macros = all_macros)

so3 = Extension('motion.so3',
                    sources = ['so3.c'],
                    define_macros = all_macros + [('SO3_STRICT', None)])

se3 = Extension('motion.se3',
                    sources = ['se3.c'],
                    define_macros = all_macros)

setup (name = 'Motion',
       version = '1.0',
       description = 'Motion code I guess',
       ext_modules = [ur, vo, so3, se3])
