import os
import glob
import subprocess
from testtools import *

experiment_sampler = 'Sampler "random" "integer pixelsamples" 8'
reference_sampler = 'Sampler "random" "integer pixelsamples" 1024'

benchmarkRepeats = 1

di_integrator = 'Integrator "guideddi" ' + ' "integer downsamplingfactor" 8 ' ' "float weightthreshold" 16 '
di_integrator += ' "bool visualizefactors" "true" '

optimal_mis_direct = """Integrator "optmis" "integer seedoffset" 0 "string technique" "L" "string lselect" "EU" "string lsat" "P"
"string misweights" "optimal" "string optimalmode" "alphasum" "string lproject" "none" "string gproject" "sphereprecise" "integer trainsamples" 0
"bool moredatalayers" "false" "integer updatestep" 1 "integer targettime" [-1]"""

optimal_mis_balance = """Integrator "optmis" "integer seedoffset" 0 "string technique" "L" "string lselect" "EU" "string lsat" "P"
"string misweights" "balance" "string optimalmode" "alphasum" "string lproject" "none" "string gproject" "sphereprecise" "integer trainsamples" 0
"bool moredatalayers" "false" "integer updatestep" 1 "integer targettime" [-1]"""

optimal_mis_ideal = """Integrator "optmis" "integer seedoffset" 0 "string technique" "L" "string lselect" "EU" "string lsat" "P"
"string misweights" "optimal" "string optimalmode" "fullweights" "string lproject" "none" "string gproject" "sphereprecise" "integer trainsamples" 300
"bool moredatalayers" "false" "integer updatestep" 1 "integer targettime" [-1]"""

optimal_mis_executable = '../../optimalmis/src/pbrt/build/pbrt'

consider_optimal = True

scenes = {
    # 'veach-mis' : {
    #     'path': '../../pbrt-v3-scenes/veach-mis/',
    #     'template': 'template.pbrt'
    # },
    'staircase1' : {
        'path': '../../pbrt-v3-scenes/staircase1/',
        'template': 'template.pbrt'
    },
    'breakfast' : {
        'path': '../../pbrt-v3-scenes/breakfast/',
        'template': 'template.pbrt'
    },
    'livingroom' : {
        'path': '../../pbrt-v3-scenes/white-room/',
        'template': 'template.pbrt'
    },
    'bathroom' : {
        'path': '../../pbrt-v3-scenes/bathroom/',
        'template': 'template.pbrt'
    }
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
    'defsampling-balance': balance + vanilla + nobsdf,
    'defsampling-power': power + vanilla + nobsdf,
    'defsampling-nomis': nomis + vanilla + nobsdf,
    'defsampling-our-balance': balance + moment + nobsdf,
    'defsampling-recipvar-only': nomis + variance + nobsdf,
    'defsampling-our-power': power + moment + nobsdf,
}

def di_tester(scene_name, scene, scene_path):
    print('Testing: ' + scene_name)
    print('==============================')

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

        time = run_and_time(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'img.exr'], workingDir, repeats=benchmarkRepeats)
        print('time for ' + vname + ': ' + str(time[0]) + ' s (+- ' + str(time[1]) + ' s)')

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
        time = run_and_time(['../../' + optimal_mis_executable, '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'direct.exr'], workingDir, repeats=benchmarkRepeats)
        print('time for optimal (direct): ' + str(time[0]) + ' s (+- ' + str(time[1]) + ' s)')

        # render with their balance estimator (for baseline equalization)
        sc = set_integrator(scene, optimal_mis_balance)
        sc = set_sampler(sc, experiment_sampler)
        with open(scene_path + 'scene.pbrtgen', 'w') as f:
            f.write(sc)
        time = run_and_time(['../../' + optimal_mis_executable, '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'balance.exr'], workingDir, repeats=benchmarkRepeats)
        print('time for optimal (balance): ' + str(time[0]) + ' s (+- ' + str(time[1]) + ' s)')

        # render with the ideal optimal weights
        # sc = set_integrator(scene, optimal_mis_ideal)
        # sc = set_sampler(sc, experiment_sampler)
        # with open(scene_path + 'scene.pbrtgen', 'w') as f:
        #     f.write(sc)
        # subprocess.call(['../../' + optimal_mis_executable, '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'ideal.exr'], cwd=workingDir)

        imgs = glob.glob(workingDir + '/' + '*.exr')
        filenames.extend(imgs)

    print('==============================')

    return filenames

filenames = run_tests('ref-di.exr', di_integrator, reference_sampler, di_tester, scenes)

# use this to open all results right away
# show_results(filenames)