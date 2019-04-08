import subprocess
import re
import os
import glob

from testtools import *

# add additional integrator parameters etc here
bdpt_integrator = 'Integrator "bdpt" "integer maxdepth" [5] "bool visualizefactors" "true" ' + ' "float clampthreshold" 16.0 '

reference_integrator = 'Integrator "bdpt" "integer maxdepth" [5] "bool visualizefactors" "false" ' + ' "float clampthreshold" 16.0 '

moment = ' "string mismod" "moment" '
variance = ' "string mismod" "reciprocal" '
vanilla = ' "string mismod" "none" '
balance = ' "string misstrategy" "balance" '
power = ' "string misstrategy" "power" '
nomis = ' "string misstrategy" "uniform" '

# different integrator configurations to test
variants = {
    'balance': balance + vanilla,
    'power': power + vanilla,
    # 'nomis': nomis + vanilla,
    # 'recipvar': balance + variance,
    'moment': balance + moment,
    # 'recipvar-only': nomis + variance,
    # 'moment-only': nomis + moment,
    # 'recipvar-power': power + variance,
    'moment-power': power + moment,
}

scenes = {
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

experiment_sampler = 'Sampler "random" "integer pixelsamples" 2'
reference_sampler = 'Sampler "random" "integer pixelsamples" 128'

def bidir_tester(scene_name, scene, scene_path):
    filenames = []
    # test all configurations
    for vname, vparams in variants.items():
        workingDir = './' + scene_name + '/' + vname
        if not os.path.exists(workingDir):
            os.makedirs(workingDir)

        # delete images from old runs
        for f in glob.glob(workingDir + '/' + '*.exr'):
            os.remove(f)

        sc = set_integrator(scene, bdpt_integrator + vparams)
        sc = set_sampler(sc, experiment_sampler)
        with open(scene_path + 'scene.pbrtgen', 'w') as f:
            f.write(sc)

        time = run_and_time(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'img.exr'], workingDir)
        print(vname + ' took ' + str(time) + ' seconds')

        imgs = glob.glob(workingDir + '/' + '*.exr')
        filenames.extend(imgs)
    return filenames

filenames = run_tests('ref-bdpt.exr', reference_integrator, reference_sampler, bidir_tester, scenes)
show_results(filenames)