project('NewtonUnityPlugin', 'cpp')

NEWTON_DYNAMICS = get_option('NEWTON_DYNAMICS')

src = [
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

shared_library('NewtonWrapper', src, include_directories : inc, link_args : largs)