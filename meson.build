project('NewtonUnityPlugin', ['cpp','cs'])

NEWTON_DYNAMICS = get_option('NEWTON_DYNAMICS')

scs_assembly_ref = ['-r:../PluginBin/UnityEngine.dll']
scs_editor_assembly_ref = ['-r:../PluginBin/UnityEngine.dll','-r:../PluginBin/UnityEditor.dll']

plugin_src = [
    'NewtonPlugin/NewtonBodyScript.cs',
    'NewtonPlugin/NewtonBody.cs',
    'NewtonPlugin/NewtonBodyCollision.cs',
    'NewtonPlugin/NewtonBodyVehicle.cs',
    'NewtonPlugin/NewtonBodyWheel.cs',
    'NewtonPlugin/NewtonBoxCollider.cs',
    'NewtonPlugin/NewtonCapsuleCollider.cs',
    'NewtonPlugin/NewtonChamferedCylinderCollider.cs',
    'NewtonPlugin/NewtonCollider.cs',
    'NewtonPlugin/NewtonCompoundCollider.cs',
    'NewtonPlugin/NewtonConeCollider.cs',
    'NewtonPlugin/NewtonConvexHullCollider.cs',
    'NewtonPlugin/NewtonCylinderCollider.cs',
    'NewtonPlugin/NewtonGears.cs',
    'NewtonPlugin/NewtonBallAndSocket.cs',
    'NewtonPlugin/NewtonHeighfieldCollider.cs',
    'NewtonPlugin/NewtonHinge.cs',
    'NewtonPlugin/NewtonJoint.cs',
    'NewtonPlugin/NewtonMaterial.cs',
    'NewtonPlugin/NewtonNullCollider.cs',
    'NewtonPlugin/NewtonSceneCollider.cs',
    'NewtonPlugin/NewtonSlider.cs',
    'NewtonPlugin/NewtonSlidingHinge.cs',
    'NewtonPlugin/NewtonSphereCollider.cs',
    'NewtonPlugin/NewtonTreeCollider.cs',
    'NewtonPlugin/NewtonDoubleHinge.cs',
    'NewtonPlugin/NewtonUtils.cs',
    'NewtonPlugin/NewtonWorld.cs',
    'NewtonPlugin/newton_wrap.cs',
    'NewtonPlugin/Properties/AssemblyInfo.cs'
]

editor_plugin_src = [
    'NewtonPluginEditor/MathHelpers.cs',
    'NewtonPluginEditor/NewtonColliderEditor.cs',
    'NewtonPluginEditor/NewtonWorldEditor.cs',
    'NewtonPluginEditor/Properties/AssemblyInfo.cs',
]

wrapper_src = [
	#'NewtonWrapper/CustomJoints.cpp',
	'NewtonWrapper/dNewtonContact.cpp',
	#'NewtonWrapper/NewtonWrapper.cpp',
	'NewtonWrapper/newton_wrap.cxx',
	'NewtonWrapper/stdafx.cpp',
	#'NewtonWrapper/Vehicle.cpp',
	'NewtonWrapper/wrapperSdk/dAlloc.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonBody.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonCollision.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonJoint.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonJointBallAndSocket.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonJointHinge.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonJointRelational.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonJointSlider.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonJointSlidingHinge.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonJointDoubleHinge.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonVehicle.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonVehicleManager.cpp',
	'NewtonWrapper/wrapperSdk/dNewtonWorld.cpp'
]

largs = []

if NEWTON_DYNAMICS.startswith('.')
	largs += '-L' + meson.current_source_dir() + '/' + NEWTON_DYNAMICS + '/build/lib'
else 
	largs += '-L' + NEWTON_DYNAMICS + '/build/lib'
endif

#	'-L' + meson.current_source_dir() + NEWTON_DYNAMICS + '/build/lib',
largs += [
	'-ldgCore',
	'-ldMath',
	'-lnewton',
	'-ldgPhysics',
	'-ldContainers',
	'-ldCustomJoints'
]

inc = include_directories(['NewtonWrapper/','NewtonWrapper/wrapperSdk/',NEWTON_DYNAMICS +'/sdk',NEWTON_DYNAMICS + '/sdk/dgNewton',NEWTON_DYNAMICS + '/sdk/dMath',NEWTON_DYNAMICS + '/sdk/dContainers',NEWTON_DYNAMICS + '/sdk/dCustomJoints'])

r = run_command('swig-3', '-INewtonWrapper/wrapperSdk', '-I' + NEWTON_DYNAMICS + '/sdk/dMath', '-I'+NEWTON_DYNAMICS+'/sdk/dContainers', 
	'-I'+NEWTON_DYNAMICS+'/sdk/dCustomJoints', '-c++', '-csharp', '-outdir', 'NewtonPlugin', '-outfile', 'newton_wrap.cs', 'NewtonWrapper/newton.i')
message(r.stdout())
if r.returncode() != 0
	error(r.stderr())
endif

shared_library('NewtonWrapper', wrapper_src, include_directories : inc, link_args : largs)

plugin = shared_library('NewtonPlugin', plugin_src, cs_args: scs_assembly_ref)

scs_editor_assembly_ref += '-r:' + plugin.full_path()

shared_library('NewtonPluginEditor', editor_plugin_src, cs_args: scs_editor_assembly_ref)