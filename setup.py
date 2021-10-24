from distutils.core import setup, Extension

all_macros = [('MOTION_DEBUG', None)]
#all_macros = []
vo_macros = [] #[('VO_RESTRICT', None)]
so3_macros = [
        ('SO3_STRICT', None),
        ('SO3_RESTRICT', None),
    ]


ur = Extension('motion.ur5controller',
                    sources = ['ur5controller.c'],
                    extra_compile_args = ["-O3"],
                    define_macros=all_macros)
vo = Extension('motion.vectorops',
                    sources = ['vectorops.c'],
                    extra_compile_args = ["-O3"],
                    define_macros = all_macros + vo_macros)

so3 = Extension('motion.so3',
                    sources = ['so3.c'],
                    extra_compile_args = ["-O3"],
                    define_macros = all_macros + so3_macros)
se3 = Extension('motion.se3',
                    sources = ['se3.c'],
                    extra_compile_args = ["-O3"],
                    define_macros = all_macros + vo_macros + so3_macros + [
                            ('SE3_RESTRICT', None),
                        ])

setup (name = 'Motion',
       version = '1.0',
       description = 'Motion code I guess',
       ext_modules = [ur, vo, so3, se3])
