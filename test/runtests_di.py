import os
import glob
import subprocess
from testtools import *

experiment_sampler = 'Sampler "random" "integer pixelsamples" 8'
reference_sampler = 'Sampler "random" "integer pixelsamples" 1024'

di_integrator = 'Integrator "guideddi" ' + ' "integer downsamplingfactor" 8 ' ' "float weightthreshold" 16 ' + ' "bool visualizefactors" "true" '

optimal_mis_direct = """Integrator "optmis" "integer seedoffset" 0 "string technique" "L" "string lselect" "EU" "string lsat" "P"
"string misweights" "optimal" "string optimalmode" "alphasum" "string lproject" "none" "string gproject" "sphereprecise" "integer trainsamples" 0
"bool moredatalayers" "false" "integer updatestep" 1 "integer targettime" [-1]"""

optimal_mis_ideal = """Integrator "optmis" "integer seedoffset" 0 "string technique" "L" "string lselect" "EU" "string lsat" "P"
"string misweights" "optimal" "string optimalmode" "fullweights" "string lproject" "none" "string gproject" "sphereprecise" "integer trainsamples" 300
"bool moredatalayers" "false" "integer updatestep" 1 "integer targettime" [-1]"""

optimal_mis_executable = '../../optimalmis/src/pbrt/build/pbrt'

consider_optimal = True

scenes = {
    'staircase1' : {
        'path': '../../pbrt-v3-scenes/staircase1/',
        'template': 'template.pbrt'
    },
    # 'breakfast' : {
    #     'path': '../../pbrt-v3-scenes/breakfast/',
    #     'template': 'template.pbrt'
    # },
    # 'livingroom' : {
    #     'path': '../../pbrt-v3-scenes/white-room/',
    #     'template': 'template.pbrt'
    # },
    # 'bathroom' : {
    #     'path': '../../pbrt-v3-scenes/bathroom/',
    #     'template': 'template.pbrt'
    # }
}

moment = ' "string varmode" "moment" '
variance = ' "string varmode" "variance" '
vanilla = ' "string varmode" "disabled" '
balance = ' "string mis" "balance" '
power = ' "string mis" "power" '
nomis = ' "string mis" "uniform" '
nobsdf = ' "bool enablebsdf" "false" '
noguided = ' "bool enableguided" "false" '
nouniform = ' "bool enableuniform" "false" '

# different integrator configurations to test
variants = {
    # 'nobsdf-balance': balance + vanilla + nobsdf,
    'nobsdf-power': power + vanilla + nobsdf,
    # 'nobsdf-nomis': nomis + vanilla + nobsdf,
    # 'nobsdf-recipvar': balance + variance + nobsdf,
    # 'nobsdf-moment': balance + moment + nobsdf,
    # 'nobsdf-recipvar-only': nomis + variance + nobsdf,
    # 'nobsdf-moment-only': nomis + moment + nobsdf,
    # 'nobsdf-recipvar-power': power + variance + nobsdf,
    'nobsdf-moment-power': power + moment + nobsdf,
    # 'noguided-power': power + vanilla + noguided,
    # 'noguided-moment-power': power + moment + noguided,
}

def di_tester(scene_name, scene, scene_path):
    filenames = []
    for vname, vparams in variants.items():
        workingDir = './' + scene_name + '/' + vname
        if not os.path.exists(workingDir):
            os.makedirs(workingDir)

        # delete images from old runs
        for f in glob.glob(workingDir + '/' + '*.exr'):
            os.remove(f)

        sc = set_integrator(scene, di_integrator + vparams)
        sc = set_sampler(sc, experiment_sampler)
        with open(scene_path + 'scene.pbrtgen', 'w') as f:
            f.write(sc)

        time = run_and_time(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'img.exr'], workingDir)
        print('time for ' + vname + ': ' + str(time))

        imgs = glob.glob(workingDir + '/' + '*.exr')
        filenames.extend(imgs)

    if consider_optimal:
        # render the image with the optimal MIS weights
        workingDir = './' + scene_name + '/optimalmis/'
        if not os.path.exists(workingDir):
            os.makedirs(workingDir)

        # delete images from old runs
        for f in glob.glob(workingDir + '/' + '*.exr'):
            os.remove(f)

        # render with the direct estimator
        sc = set_integrator(scene, optimal_mis_direct)
        sc = set_sampler(sc, experiment_sampler)
        with open(scene_path + 'scene.pbrtgen', 'w') as f:
            f.write(sc)
        time = run_and_time(['../../' + optimal_mis_executable, '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'direct.exr'], workingDir)
        print('time for optimal (direct): ' + str(time))

        # render with the ideal optimal weights
        # sc = set_integrator(scene, optimal_mis_ideal)
        # sc = set_sampler(sc, experiment_sampler)
        # with open(scene_path + 'scene.pbrtgen', 'w') as f:
        #     f.write(sc)
        # subprocess.call(['../../' + optimal_mis_executable, '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'ideal.exr'], cwd=workingDir)

        imgs = glob.glob(workingDir + '/' + '*.exr')
        filenames.extend(imgs)

    return filenames

filenames = run_tests('ref-di.exr', di_integrator, reference_sampler, di_tester, scenes)
show_results(filenames)