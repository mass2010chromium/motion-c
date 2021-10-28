from distutils.core import setup, Extension

all_macros = [('MOTION_DEBUG', None)]
#all_macros = []
vo_macros = [] #[('VO_RESTRICT', None)]
so3_macros = [
        ('SO3_STRICT', None),
        ('SO3_RESTRICT', None),
    ]

rtde_macros = [
        ('RTDE_PROTOCOL_VERSION', 2)
    ]


ur = Extension('Motion.ur5controller',
                    sources = ['c/ur5controller.c'],
                    extra_compile_args = ["-O3"],
                    define_macros=all_macros)
vo = Extension('motionlib.vectorops',
                    sources = ['c/vectorops.c'],
                    extra_compile_args = ["-O3"],
                    define_macros = all_macros + vo_macros)

so3 = Extension('motionlib.so3',
                    sources = ['c/so3.c'],
                    extra_compile_args = ["-O3"],
                    define_macros = all_macros + so3_macros)
se3 = Extension('motionlib.se3',
                    sources = ['c/se3.c'],
                    extra_compile_args = ["-O3"],
                    define_macros = all_macros + vo_macros + so3_macros + [
                            ('SE3_RESTRICT', None),
                        ])

setup (name = 'Motion',
       version = '1.0',
       description = 'Motion code I guess',
       ext_modules = [ur, vo, so3, se3])
