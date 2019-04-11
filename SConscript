from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add da270 src files.
src += Glob('sensor_mira_da270.c')
src += Glob('libraries/da270.c')

# add da270 include path.
path  = [cwd, cwd + '/libraries']

# add src and include to group.
group = DefineGroup('da270', src, depend = ['PKG_USING_DA270'], CPPPATH = path)

Return('group')
